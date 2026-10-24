// Copyright (c) 2026 srounet
// SPDX-License-Identifier: MIT
//
// nanobind bindings exposing Detour's navmesh query engine (from recastnavigation,
// zlib-licensed, see THIRD_PARTY_NOTICES.md) as the `wow_navmesh._wow_navmesh` Python
// extension module. Reads the .mmap/.mmtile files produced by TrinityCore/AzerothCore's
// mmap generator for World of Warcraft 3.3.5a.
//
// Coordinate convention: every value crossing the Python boundary is in WoW world order
// (x, y, z). Detour (and the files on disk) use (y, z, x); wowToDetour()/detourToWow()
// are the only places that convert between the two.

#include <nanobind/nanobind.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourNode.h"
#include "DetourPathCorridor.h"

namespace nb = nanobind;
namespace fs = std::filesystem;

namespace {

// TrinityCore/AzerothCore's mmap generator writes tile files with this magic value,
// independent of the tile's dtPolyRef width. mmapVersion itself is bumped by both forks
// whenever generator *behavior* changes (e.g. a pathing fix requiring regeneration) --
// TrinityCore's 3.3.5 branch uses 15 and AzerothCore's master 20, for example. So we
// only reject versions old enough to predate the header prefix below, rather than pinning
// to one exact number that the next upstream bump would immediately break again.
//
// The header has *grown* across versions: version 4 wrote a bare 20 bytes, while version
// 20 appends the Recast build parameters the tile was generated with (36 more bytes). The
// 20-byte prefix below is stable, and the Detour payload is always the trailing
// header.size bytes -- so the payload offset is read off the file rather than assumed to
// be sizeof(MmapTileHeader), which keeps a future header growth from silently shifting
// every read.
constexpr unsigned int kMmapMagic = 0x4d4d4150;  // 'MMAP'
constexpr unsigned int kMmapVersionMin = 4;

// Header of each .mmap tile file produced by TrinityCore/AzerothCore's mmap generator.
// Only the stable 20-byte prefix; later versions append fields we don't need.
struct MmapTileHeader {
    unsigned int mmapMagic;
    unsigned int dtVersion;
    unsigned int mmapVersion;
    unsigned int size;
    unsigned char usesLiquids;  // not bool: any byte value can come off disk
    char padding[3];
};
static_assert(sizeof(MmapTileHeader) == 20, "mmtile header prefix must stay 20 bytes");

// The on-disk size of dtLink for either dtPolyRef width. dtLink packs a dtPolyRef, a
// uint32 "next" index, and four bytes of edge/side/bmin/bmax; the struct's natural
// alignment (4 bytes for a 32-bit ref, 8 for a 64-bit ref) leaves no extra padding in
// either case, so these are exact: 4+4+4 = 12, and 8+4+4 = 16.
constexpr int kLinkSize32 = 12;
constexpr int kLinkSize64 = 16;
static_assert(sizeof(dtPolyRef) == 4 || sizeof(dtPolyRef) == 8,
              "unexpected dtPolyRef width");
#ifdef DT_POLYREF64
static_assert(sizeof(dtLink) == kLinkSize64, "dtLink layout changed upstream");
#else
static_assert(sizeof(dtLink) == kLinkSize32, "dtLink layout changed upstream");
#endif

// Every other struct that DetourNavMeshBuilder serializes into a tile blob (dtMeshHeader,
// dtPoly, dtPolyDetail, dtBVNode, dtOffMeshConnection) is built solely from int/float/short
// members, so their sizes don't depend on the dtPolyRef width. dtLink is the only one that
// does, because it embeds a dtPolyRef directly. That means a tile's total serialized size
// tells us, unambiguously, which width it was built with -- see expectedTileSize() below.

// Returns the serialized size in bytes of a tile described by `h`, assuming each dtLink
// occupies `linkSize` bytes (kLinkSize32 or kLinkSize64).
//
// Mirrors the section-size math in DetourNavMeshBuilder.cpp / DetourNavMesh.cpp (down to
// using the same `int` arithmetic, since dtMeshHeader's counts are ints there too), with
// the dtLink size as a free parameter so we can test both possible tile layouts.
std::size_t expectedTileSize(const dtMeshHeader& h, int linkSize) {
    const int headerSize = dtAlign4(static_cast<int>(sizeof(dtMeshHeader)));
    const int vertsSize = dtAlign4(static_cast<int>(sizeof(float)) * 3 * h.vertCount);
    const int polysSize = dtAlign4(static_cast<int>(sizeof(dtPoly)) * h.polyCount);
    const int linksSize = dtAlign4(linkSize * h.maxLinkCount);
    const int detailMeshesSize = dtAlign4(static_cast<int>(sizeof(dtPolyDetail)) * h.detailMeshCount);
    const int detailVertsSize = dtAlign4(static_cast<int>(sizeof(float)) * 3 * h.detailVertCount);
    const int detailTrisSize = dtAlign4(static_cast<int>(sizeof(unsigned char)) * 4 * h.detailTriCount);
    const int bvTreeSize = dtAlign4(static_cast<int>(sizeof(dtBVNode)) * h.bvNodeCount);
    const int offMeshConsSize = dtAlign4(static_cast<int>(sizeof(dtOffMeshConnection)) * h.offMeshConCount);
    return static_cast<std::size_t>(headerSize) + vertsSize + polysSize + linksSize +
           detailMeshesSize + detailVertsSize + detailTrisSize + bvTreeSize + offMeshConsSize;
}

// Validates that a raw tile blob matches the dtPolyRef width this extension was built
// with.
//
// `data` points to `data_size` bytes of Detour payload read from the .mmtile file at
// `tile_path` (the path is only used in error messages). Without this check, a tile
// built with the other width still passes dtNavMesh::addTile()'s magic/version check
// (those don't encode polyref width) but every section after dtLink is read from the
// wrong offset -- silent corruption rather than a clean failure.
//
// Returns normally on success. Throws std::runtime_error naming the problem if the blob
// is too small, has the wrong magic/version, was built for the other dtPolyRef width, or
// matches neither layout (truncated or corrupt).
void checkTileLayout(const std::string& tile_path, const unsigned char* data, std::size_t data_size) {
    if (data_size < sizeof(dtMeshHeader))
        throw std::runtime_error("mmtile too small to contain a header: " + tile_path);

    dtMeshHeader header;
    std::memcpy(&header, data, sizeof(dtMeshHeader));

    if (header.magic != DT_NAVMESH_MAGIC)
        throw std::runtime_error("mmtile has wrong navmesh magic (corrupt or unrelated file): " +
                                  tile_path);
    if (header.version != DT_NAVMESH_VERSION)
        throw std::runtime_error(
            "mmtile has unsupported navmesh version " + std::to_string(header.version) +
            " (expected " + std::to_string(DT_NAVMESH_VERSION) + "): " + tile_path);

#ifdef DT_POLYREF64
    constexpr int kOurLinkSize = kLinkSize64;
    constexpr int kOtherLinkSize = kLinkSize32;
    constexpr const char* kOurVariant = "64-bit dtPolyRef (DT_POLYREF64, TrinityCore/AzerothCore)";
    constexpr const char* kOtherVariant = "32-bit dtPolyRef (plain Detour)";
#else
    constexpr int kOurLinkSize = kLinkSize32;
    constexpr int kOtherLinkSize = kLinkSize64;
    constexpr const char* kOurVariant = "32-bit dtPolyRef (plain Detour)";
    constexpr const char* kOtherVariant = "64-bit dtPolyRef (DT_POLYREF64, TrinityCore/AzerothCore)";
#endif

    const std::size_t expected_ours = expectedTileSize(header, kOurLinkSize);
    if (expected_ours == data_size)
        return;

    const std::size_t expected_other = expectedTileSize(header, kOtherLinkSize);
    if (expected_other == data_size) {
        throw std::runtime_error(
            "mmtile layout mismatch: " + tile_path + " was built for " +
            std::string(kOtherVariant) + " (size " + std::to_string(data_size) +
            " matches that layout), but this extension was built for " + kOurVariant +
            " (expected size " + std::to_string(expected_ours) + "). Rebuild with a matching "
            "WOW_NAVMESH_POLYREF64 setting.");
    }

    throw std::runtime_error(
        "mmtile has an unexpected size: " + tile_path + " is " + std::to_string(data_size) +
        " bytes, but the header implies " + std::to_string(expected_ours) + " (" + kOurVariant +
        ") or " + std::to_string(expected_other) + " (" + kOtherVariant +
        ") bytes. The file may be truncated or corrupt.");
}

// RAII wrappers so a throw partway through load_map() (e.g. from checkTileLayout) can't leak
// the in-progress dtNavMesh or a tile buffer that addTile() didn't take ownership of.
struct DtNavMeshDeleter {
    // Frees `m` with dtFreeNavMesh(). Null is a no-op.
    void operator()(dtNavMesh* m) const {
        if (m)
            dtFreeNavMesh(m);
    }
};
using DtNavMeshPtr = std::unique_ptr<dtNavMesh, DtNavMeshDeleter>;

struct DtAllocDeleter {
    // Frees `p` with dtFree(). Null is a no-op.
    void operator()(unsigned char* p) const {
        if (p)
            dtFree(p);
    }
};
using DtBuffer = std::unique_ptr<unsigned char, DtAllocDeleter>;

using Point3 = std::tuple<float, float, float>;

// mmap/mmtile files store coordinates as (y, z, x); WoW's world coordinates (and every
// public function in this module) use (x, y, z). These two helpers are the only place
// that swap ever happens, so no new query function can silently get it wrong.

// Writes WoW-order point `p` into `out` (a float[3]) in Detour (y, z, x) order.
inline void wowToDetour(const Point3& p, float* out) {
    out[0] = std::get<1>(p);
    out[1] = std::get<2>(p);
    out[2] = std::get<0>(p);
}
// Returns Detour-order point `p` (a float[3]) as a WoW-order (x, y, z) tuple.
inline Point3 detourToWow(const float* p) { return Point3(p[2], p[0], p[1]); }

// Outcome of NavMesh::find_path(): whether the path actually reaches the requested end,
// or only gets as close as the navmesh allows. Same distinction as the PATHFIND_NORMAL /
// PATHFIND_INCOMPLETE flags in the MaNGOS-family server cores' path generators.
enum class PathType {
    NotFound = 0,  // no path at all (start/end off the mesh, or no route between them)
    Normal = 1,    // path reaches the requested end point
    Partial = 2,   // path stops short of the end (unreachable, or a buffer/node pool ran out)
};

// Result of NavMesh::find_path(): the waypoints plus whether they reach the end.
struct PathResult {
    std::vector<Point3> points;
    PathType path_type;
    Point3 actual_end;  // last point of `points`, or the start if path_type is NotFound.
};

// Distance (world units) beyond which a nearest-poly match is considered "off the mesh"
// rather than a walkable point close enough to start/end from. Same threshold the
// MaNGOS-family server cores use.
constexpr float kFarFromPolyDistance = 7.0f;

// Computes the midpoint of the portal edge shared by adjacent polygons `from` and `to`
// in `nav`, writing it to `mid` (a float[3], Detour order).
//
// Mirrors dtNavMeshQuery::getEdgeMidPoint (private upstream, so reimplemented here).
// Used by find_path's centered=true mode to route through polygon interiors instead of
// the funnel algorithm's taut string-pull, which stays as close to a wall as the corridor
// allows. Returns false (skip this waypoint) for off-mesh connections or on any lookup
// failure -- the caller falls back to just not inserting a midpoint there.
bool portalMidpoint(const dtNavMesh* nav, dtPolyRef from, dtPolyRef to, float* mid) {
    const dtMeshTile* fromTile = nullptr;
    const dtPoly* fromPoly = nullptr;
    if (dtStatusFailed(nav->getTileAndPolyByRef(from, &fromTile, &fromPoly)))
        return false;
    const dtMeshTile* toTile = nullptr;
    const dtPoly* toPoly = nullptr;
    if (dtStatusFailed(nav->getTileAndPolyByRef(to, &toTile, &toPoly)))
        return false;
    if (fromPoly->getType() != DT_POLYTYPE_GROUND || toPoly->getType() != DT_POLYTYPE_GROUND)
        return false;

    const dtLink* link = nullptr;
    for (unsigned int i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromTile->links[i].next) {
        if (fromTile->links[i].ref == to) {
            link = &fromTile->links[i];
            break;
        }
    }
    if (!link)
        return false;

    const int v0 = fromPoly->verts[link->edge];
    const int v1 = fromPoly->verts[(link->edge + 1) % static_cast<int>(fromPoly->vertCount)];
    float left[3], right[3];
    dtVcopy(left, &fromTile->verts[v0 * 3]);
    dtVcopy(right, &fromTile->verts[v1 * 3]);

    // Tile-boundary links are clamped to the overlapping width of the two tiles' edges;
    // narrow that down the same way before averaging.
    if (link->side != 0xff && (link->bmin != 0 || link->bmax != 255)) {
        const float s = 1.0f / 255.0f;
        dtVlerp(left, &fromTile->verts[v0 * 3], &fromTile->verts[v1 * 3], link->bmin * s);
        dtVlerp(right, &fromTile->verts[v0 * 3], &fromTile->verts[v1 * 3], link->bmax * s);
    }

    mid[0] = (left[0] + right[0]) * 0.5f;
    mid[1] = (left[1] + right[1]) * 0.5f;
    mid[2] = (left[2] + right[2]) * 0.5f;
    return true;
}

// Explicit outcome of a Detour query, rather than collapsing every failure mode into an
// empty result. Distinguishes "no path exists" from "start/end isn't on the mesh" from
// "the result buffer was too small" from "Detour itself failed".
enum class PathStatus {
    Success = 0,
    Partial,
    NoPath,
    StartNotFound,
    EndNotFound,
    BufferFull,
    QueryFailed,
};

// Thin Python-friendly wrapper over dtQueryFilter. Kept intentionally close to the
// Detour type -- area costs are already extensible via setAreaCost, no need to invent a
// parallel mechanism for it.
class QueryFilter {
public:
    // Returns the poly flags a polygon must have at least one of to be traversable.
    unsigned short include_flags() const { return filter_.getIncludeFlags(); }
    // Sets the include flags; see include_flags().
    void set_include_flags(unsigned short f) { filter_.setIncludeFlags(f); }
    // Returns the poly flags that make a polygon non-traversable.
    unsigned short exclude_flags() const { return filter_.getExcludeFlags(); }
    // Sets the exclude flags; see exclude_flags().
    void set_exclude_flags(unsigned short f) { filter_.setExcludeFlags(f); }
    // Returns the traversal cost multiplier for area id `area` (0..DT_MAX_AREAS-1).
    // Throws std::invalid_argument if `area` is out of range.
    float area_cost(int area) const {
        checkArea(area);
        return filter_.getAreaCost(area);
    }
    // Sets the traversal cost multiplier for area id `area` to `cost`. Throws
    // std::invalid_argument if `area` is out of range.
    void set_area_cost(int area, float cost) {
        checkArea(area);
        filter_.setAreaCost(area, cost);
    }
    // Returns the wrapped Detour filter, for passing to dtNavMeshQuery calls.
    const dtQueryFilter& raw() const { return filter_; }

private:
    // Detour indexes its area cost array without a bounds check, so an out-of-range id
    // would read or write past it (into the include/exclude flags, then the heap).
    static void checkArea(int area) {
        if (area < 0 || area >= DT_MAX_AREAS)
            throw std::invalid_argument("area must be in [0, " + std::to_string(DT_MAX_AREAS) + ")");
    }

    dtQueryFilter filter_;
};

// Returns `filter`'s Detour filter, or an accept-everything default if it's null.
const dtQueryFilter& filterOrDefault(const QueryFilter* filter) {
    static const dtQueryFilter kDefault;
    return filter ? filter->raw() : kDefault;
}

// Defaults NavigationQuery methods fall back on when the caller passes no explicit
// override: nearest-poly search box half-extents (WoW axis order, like every other
// triple), polygon-corridor buffer size, and straight-path buffer size.
struct QueryConfig {
    std::tuple<float, float, float> nearest_poly_extents{50.0f, 50.0f, 50.0f};
    int max_path_polys = 512;
    int max_straight_path_points = 256;
};

// Result types returned by NavigationQuery/Path. Positions are WoW (x, y, z); a poly ref
// of 0 means "none". Field meanings are documented on the Python side
// (src/wow_navmesh/__init__.pyi).

// Result of NavigationQuery::find_nearest_poly().
struct NearestPolyResult {
    bool found = false;
    dtPolyRef poly_ref = 0;
    Point3 position{0.0f, 0.0f, 0.0f};
    float distance = -1.0f;
};

// Result of NavigationQuery::find_polygon_path().
struct PolygonPathResult {
    PathStatus status = PathStatus::QueryFailed;
    dtPolyRef start_poly = 0;
    dtPolyRef end_poly = 0;
    std::vector<dtPolyRef> polygons;
    bool reached_end = false;
};

// One point of a straight path; `flags` holds DT_STRAIGHTPATH_* bits.
struct StraightPathPoint {
    Point3 position{0.0f, 0.0f, 0.0f};
    unsigned char flags = 0;
    dtPolyRef poly_ref = 0;
};

// Result of NavigationQuery::find_straight_path().
struct StraightPathResult {
    PathStatus status = PathStatus::QueryFailed;
    std::vector<StraightPathPoint> points;
};

// Result of NavigationQuery::move_along_surface().
struct MoveAlongSurfaceResult {
    PathStatus status = PathStatus::QueryFailed;
    Point3 position{0.0f, 0.0f, 0.0f};
    std::vector<dtPolyRef> visited;
};

// Result of NavigationQuery::raycast().
struct RaycastResult {
    PathStatus status = PathStatus::QueryFailed;
    bool hit = false;
    Point3 position{0.0f, 0.0f, 0.0f};
    Point3 normal{0.0f, 0.0f, 0.0f};
    float t = 0.0f;
    std::vector<dtPolyRef> path;
};

// Result of NavigationQuery::find_distance_to_wall().
struct WallDistanceResult {
    float distance = -1.0f;
    Point3 position{0.0f, 0.0f, 0.0f};
    Point3 normal{0.0f, 0.0f, 0.0f};
};

// Result of Path::get_steer_target().
struct SteerTarget {
    Point3 position{0.0f, 0.0f, 0.0f};
    dtPolyRef poly_ref = 0;
    float distance = 0.0f;
    bool reached = false;
    bool off_mesh = false;
};

// Detour's DT_POLYTYPE_* poly kinds, exposed as-is: a standard walkable polygon vs. an
// off-mesh connection (jump/teleport/etc.) represented as a 2-vertex "polygon".
enum class PolyType {
    Ground = 0,
    OffMeshConnection = 1,
};

// Raw structural data for one dtPoly, enough to inspect a polygon from Python without
// touching Detour directly. center is the vertex centroid -- Detour has no dedicated
// per-poly center function, and centroid is well-defined for both ground polygons and
// the 2-vertex off-mesh connection "polygons" (their centroid is just the midpoint).
struct PolyInfo {
    dtPolyRef ref = 0;
    PolyType type = PolyType::Ground;
    unsigned short flags = 0;
    unsigned char area = 0;
    Point3 center{0.0f, 0.0f, 0.0f};
    std::vector<Point3> vertices;
    std::vector<dtPolyRef> neighbors;
    int tile_x = 0;
    int tile_y = 0;
    int tile_layer = 0;
};

// Structural data for one loaded dtMeshTile. uses_liquids comes from TrinityCore/
// AzerothCore's own MmapTileHeader (not part of Detour's dtMeshHeader) -- it describes
// the tile's liquid handling, not any individual polygon's area/flags.
struct TileInfo {
    int x = 0;
    int y = 0;
    int layer = 0;
    Point3 bounds_min{0.0f, 0.0f, 0.0f};
    Point3 bounds_max{0.0f, 0.0f, 0.0f};
    int poly_count = 0;
    bool uses_liquids = false;
};

// Raw dtOffMeshConnection data. flags/area come from the connection's own dtPoly --
// dtOffMeshConnection::flags itself is documented upstream as internal link bookkeeping,
// not the connection's user-defined flags.
struct OffMeshConnection {
    dtPolyRef ref = 0;
    Point3 start{0.0f, 0.0f, 0.0f};
    Point3 end{0.0f, 0.0f, 0.0f};
    float radius = 0.0f;
    unsigned short flags = 0;
    unsigned char area = 0;
    bool bidirectional = false;
};

// Returns the vertex centroid of `poly` (which must belong to `tile`) in WoW order.
inline Point3 polyCentroid(const dtMeshTile* tile, const dtPoly* poly) {
    float sum[3] = {0.0f, 0.0f, 0.0f};
    const int n = poly->vertCount;
    for (int i = 0; i < n; i++) {
        const float* v = &tile->verts[poly->verts[i] * 3];
        sum[0] += v[0];
        sum[1] += v[1];
        sum[2] += v[2];
    }
    if (n > 0) {
        sum[0] /= n;
        sum[1] /= n;
        sum[2] /= n;
    }
    return detourToWow(sum);
}

// Returns the vertices of `poly` (which must belong to `tile`) in WoW order.
inline std::vector<Point3> polyVertices(const dtMeshTile* tile, const dtPoly* poly) {
    std::vector<Point3> verts;
    verts.reserve(poly->vertCount);
    for (int i = 0; i < poly->vertCount; i++)
        verts.push_back(detourToWow(&tile->verts[poly->verts[i] * 3]));
    return verts;
}

// Returns the refs of the polygons linked to `poly` (which must belong to `tile`).
//
// Walks only resolved links (the chain is terminated by DT_NULL_LINK), so unconnected
// edges (neis[i] == 0, i.e. mesh border) never show up here -- no extra filtering needed.
// One ref can appear more than once: a poly split across several edges of a tile border
// gets one link per edge segment, all pointing at the same neighbour.
inline std::vector<dtPolyRef> polyNeighbors(const dtMeshTile* tile, const dtPoly* poly) {
    std::vector<dtPolyRef> neighbors;
    for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
        if (tile->links[i].ref)
            neighbors.push_back(tile->links[i].ref);
    return neighbors;
}

// Returns a PolyInfo for `poly`, whose ref is `ref` and which belongs to `tile`.
inline PolyInfo buildPolyInfo(dtPolyRef ref, const dtMeshTile* tile, const dtPoly* poly) {
    PolyInfo info;
    info.ref = ref;
    info.type = poly->getType() == DT_POLYTYPE_GROUND ? PolyType::Ground : PolyType::OffMeshConnection;
    info.flags = poly->flags;
    info.area = poly->getArea();
    info.center = polyCentroid(tile, poly);
    info.vertices = polyVertices(tile, poly);
    info.neighbors = polyNeighbors(tile, poly);
    info.tile_x = tile->header->x;
    info.tile_y = tile->header->y;
    info.tile_layer = tile->header->layer;
    return info;
}

// Returns a TileInfo for loaded `tile`. `uses_liquids` comes from the tile's
// MmapTileHeader, which Detour doesn't keep, so the caller supplies it.
inline TileInfo buildTileInfo(const dtMeshTile* tile, bool uses_liquids) {
    TileInfo info;
    const dtMeshHeader* h = tile->header;
    info.x = h->x;
    info.y = h->y;
    info.layer = h->layer;
    info.bounds_min = detourToWow(h->bmin);
    info.bounds_max = detourToWow(h->bmax);
    info.poly_count = h->polyCount;
    info.uses_liquids = uses_liquids;
    return info;
}

// Returns an OffMeshConnection for the `index`-th off-mesh connection of `tile`, which
// must be loaded in `mesh` (used to compute the connection's poly ref).
inline OffMeshConnection buildOffMeshConnection(const dtNavMesh* mesh, const dtMeshTile* tile, int index) {
    const dtOffMeshConnection& con = tile->offMeshCons[index];
    const dtPoly& poly = tile->polys[tile->header->offMeshBase + index];
    OffMeshConnection info;
    info.ref = mesh->getPolyRefBase(tile) + static_cast<dtPolyRef>(tile->header->offMeshBase + index);
    info.start = detourToWow(&con.pos[0]);
    info.end = detourToWow(&con.pos[3]);
    info.radius = con.rad;
    info.flags = poly.flags;
    info.area = poly.getArea();
    info.bidirectional = (con.flags & DT_OFFMESH_CON_BIDIR) != 0;
    return info;
}

class Path;    // forward decl; defined after NavigationQuery, which builds it.
class NavMesh; // forward decl; NavigationQuery/Path only need it to detect a reload/free
               // that has invalidated the raw pointers they were built from.

// Thin wrapper around dtNavMeshQuery. Non-owning: the dtNavMesh/dtNavMeshQuery it points
// to are owned by the NavMesh that created it. load_map()/free_map() on that NavMesh free
// and reallocate them, which would otherwise leave any previously-issued NavigationQuery
// holding dangling pointers; owner_/generation_ let every method detect that and raise
// instead of touching freed memory. (nanobind keep_alive on NavMesh::query() keeps the
// NavMesh Python object itself alive for as long as this one exists, so owner_ is always
// safe to dereference -- only *what it points to* may have changed.)
class NavigationQuery {
public:
    // Wraps `mesh`/`query` (not owned) using `config` defaults. `owner` and `generation`
    // identify the NavMesh state they came from, for ensureFresh().
    NavigationQuery(dtNavMesh* mesh, dtNavMeshQuery* query, QueryConfig config, const NavMesh* owner,
                     int generation)
        : mesh_(mesh), query_(query), config_(config), owner_(owner), generation_(generation) {}

    // Finds the polygon nearest to `position` within the search box `extents` (defaults
    // to config().nearest_poly_extents), honoring `filter` (nullable, default filter).
    // Returns a result with found == false, rather than throwing, if none is in range.
    NearestPolyResult find_nearest_poly(Point3 position,
                                         std::optional<std::tuple<float, float, float>> extents,
                                         const QueryFilter* filter) const {
        ensureFresh();
        const dtQueryFilter* f = &filterOrDefault(filter);
        float pos[3];
        wowToDetour(position, pos);
        float ext[3];
        extentsOrDefault(extents, ext);

        NearestPolyResult result;
        float resultPos[3] = {0.0f, 0.0f, 0.0f};
        dtStatus st = query_->findNearestPoly(pos, ext, f, &result.poly_ref, resultPos);
        result.found = dtStatusSucceed(st) && result.poly_ref != 0;
        if (result.found) {
            result.position = detourToWow(resultPos);
            result.distance = dtVdist(pos, resultPos);
        } else {
            result.poly_ref = 0;
        }
        return result;
    }

    // Finds the raw Detour polygon corridor from `start` to `end`: the nearest start/end
    // polygons plus the polygon chain between them, at most `max_polys` long (defaults
    // to config().max_path_polys). Failure modes are reported through the result's
    // status. Throws std::invalid_argument if max_polys is not positive.
    //
    // Kept separate from find_straight_path/find_path so callers who only need the
    // corridor (e.g. to feed it elsewhere) don't pay for straight-path generation.
    PolygonPathResult find_polygon_path(Point3 start, Point3 end, const QueryFilter* filter,
                                         std::optional<int> max_polys) const {
        ensureFresh();
        const dtQueryFilter* f = &filterOrDefault(filter);
        const int maxPolys = max_polys.value_or(config_.max_path_polys);
        if (maxPolys <= 0)
            throw std::invalid_argument("max_polys must be positive");

        float s[3];
        wowToDetour(start, s);
        float e[3];
        wowToDetour(end, e);
        float ext[3];
        extentsOrDefault(std::nullopt, ext);

        PolygonPathResult result;
        float startPt[3], endPt[3];
        dtStatus st = query_->findNearestPoly(s, ext, f, &result.start_poly, startPt);
        if (dtStatusFailed(st) || !result.start_poly) {
            result.status = PathStatus::StartNotFound;
            result.start_poly = 0;
            return result;
        }
        st = query_->findNearestPoly(e, ext, f, &result.end_poly, endPt);
        if (dtStatusFailed(st) || !result.end_poly) {
            result.status = PathStatus::EndNotFound;
            result.end_poly = 0;
            return result;
        }

        std::vector<dtPolyRef> polys(static_cast<std::size_t>(maxPolys));
        int count = 0;
        st = query_->findPath(result.start_poly, result.end_poly, startPt, endPt, f, polys.data(),
                               &count, maxPolys);
        if (dtStatusFailed(st)) {
            result.status = PathStatus::QueryFailed;
            return result;
        }
        if (count == 0) {
            result.status = PathStatus::NoPath;
            return result;
        }
        polys.resize(static_cast<std::size_t>(count));
        result.reached_end = polys.back() == result.end_poly;
        result.polygons = std::move(polys);
        // Detour reports a too-long corridor as a *successful* search with a detail flag
        // (keeping its first maxPolys polygons), so it has to be checked here.
        if (dtStatusDetail(st, DT_BUFFER_TOO_SMALL))
            result.status = PathStatus::BufferFull;
        else
            result.status = result.reached_end ? PathStatus::Success : PathStatus::Partial;
        return result;
    }

    // Runs Detour's findStraightPath() (funnel algorithm) from `start` to `end` over the
    // already-computed corridor `polygons`, returning at most `max_points` points
    // (defaults to config().max_straight_path_points). Does not itself locate start/end
    // polygons -- pass the corridor from find_polygon_path(). Throws
    // std::invalid_argument if polygons is empty or max_points is not positive.
    StraightPathResult find_straight_path(Point3 start, Point3 end,
                                           const std::vector<dtPolyRef>& polygons,
                                           std::optional<int> max_points) const {
        ensureFresh();
        if (polygons.empty())
            throw std::invalid_argument("polygons must be non-empty");
        const int maxPts = max_points.value_or(config_.max_straight_path_points);
        if (maxPts <= 0)
            throw std::invalid_argument("max_points must be positive");

        float s[3];
        wowToDetour(start, s);
        float e[3];
        wowToDetour(end, e);

        // One spare slot: Detour sets DT_BUFFER_TOO_SMALL whenever the buffer ends up
        // full, even if the point that filled it was the end, so only a point beyond
        // maxPts tells a cut-off path apart from one that fits exactly.
        const int capacity = maxPts + 1;
        std::vector<float> pts(static_cast<std::size_t>(capacity) * 3);
        std::vector<unsigned char> flags(static_cast<std::size_t>(capacity));
        std::vector<dtPolyRef> refs(static_cast<std::size_t>(capacity));
        int count = 0;
        dtStatus st = query_->findStraightPath(s, e, polygons.data(), static_cast<int>(polygons.size()),
                                                pts.data(), flags.data(), refs.data(), &count, capacity);

        StraightPathResult result;
        if (dtStatusFailed(st)) {
            result.status = PathStatus::QueryFailed;
            return result;
        }
        const bool truncated = count > maxPts;
        count = std::min(count, maxPts);
        result.points.reserve(static_cast<std::size_t>(count));
        for (int i = 0; i < count; i++) {
            StraightPathPoint p;
            p.position = detourToWow(&pts[static_cast<std::size_t>(i) * 3]);
            p.flags = flags[static_cast<std::size_t>(i)];
            p.poly_ref = refs[static_cast<std::size_t>(i)];
            result.points.push_back(p);
        }
        if (truncated)
            result.status = PathStatus::BufferFull;
        else
            result.status = dtStatusDetail(st, DT_PARTIAL_RESULT) ? PathStatus::Partial : PathStatus::Success;
        return result;
    }

    // Slides from `current` (inside polygon `start_poly`) toward `desired` across the
    // walkable surface without leaving the mesh, via dtNavMeshQuery::moveAlongSurface(),
    // and snaps the result's height onto the surface. Records up to `max_visited`
    // (default 32) visited polygons. Central to steering: callers use it to advance an
    // agent's actual position each tick. Throws std::invalid_argument if start_poly is 0
    // or max_visited is not positive.
    MoveAlongSurfaceResult move_along_surface(Point3 current, Point3 desired, dtPolyRef start_poly,
                                               const QueryFilter* filter,
                                               std::optional<int> max_visited) const {
        ensureFresh();
        if (!start_poly)
            throw std::invalid_argument("start_poly must be non-zero");
        const dtQueryFilter* f = &filterOrDefault(filter);
        const int maxVisited = max_visited.value_or(32);
        if (maxVisited <= 0)
            throw std::invalid_argument("max_visited must be positive");

        float s[3];
        wowToDetour(current, s);
        float e[3];
        wowToDetour(desired, e);
        float resultPos[3] = {0.0f, 0.0f, 0.0f};
        // moveAlongSurface() searches a 64-node pool, so 65 slots always hold the whole
        // visited list: the polygon the move ends in is known even when the caller asks
        // for fewer, and a list longer than maxVisited is truncation, not an exact fit
        // (Detour flags DT_BUFFER_TOO_SMALL for both).
        constexpr int kMoveSearchNodes = 64;
        std::vector<dtPolyRef> visited(static_cast<std::size_t>(std::max(maxVisited, kMoveSearchNodes) + 1));
        int visitedCount = 0;
        dtStatus st = query_->moveAlongSurface(start_poly, s, e, f, resultPos, visited.data(),
                                                &visitedCount, static_cast<int>(visited.size()));

        MoveAlongSurfaceResult result;
        if (dtStatusFailed(st) || visitedCount == 0)
            return result;
        // Detour leaves resultPos at the height it was asked to move to ("resultPos is not
        // projected onto the surface"); snap it onto the polygon the move ended in, like
        // dtPathCorridor::movePosition() does.
        float onSurface[3];
        if (dtStatusSucceed(query_->closestPointOnPoly(visited[visitedCount - 1], resultPos, onSurface, nullptr)))
            resultPos[1] = onSurface[1];
        result.status = visitedCount > maxVisited ? PathStatus::BufferFull : PathStatus::Success;
        result.position = detourToWow(resultPos);
        visited.resize(static_cast<std::size_t>(std::min(visitedCount, maxVisited)));
        result.visited = std::move(visited);
        return result;
    }

    // Casts a ray along the navmesh surface from `start` toward `end` using Detour's
    // raycast -- not a geometric approximation. If `start_poly` isn't given, it's found
    // via a nearest-poly search first. Records up to `max_path` (defaults to
    // config().max_path_polys) crossed polygons. Failure modes are reported through the
    // result's status (the hit fields are then meaningless). Throws
    // std::invalid_argument if max_path is not positive.
    RaycastResult raycast(Point3 start, Point3 end, std::optional<dtPolyRef> start_poly,
                           const QueryFilter* filter, std::optional<int> max_path) const {
        ensureFresh();
        const int maxPath = max_path.value_or(config_.max_path_polys);
        if (maxPath <= 0)
            throw std::invalid_argument("max_path must be positive");
        const dtQueryFilter* f = &filterOrDefault(filter);
        float s[3];
        wowToDetour(start, s);
        float e[3];
        wowToDetour(end, e);

        dtPolyRef startRef = start_poly.value_or(0);
        if (!startRef) {
            float ext[3];
            extentsOrDefault(std::nullopt, ext);
            float nearestPt[3];
            query_->findNearestPoly(s, ext, f, &startRef, nearestPt);
        }

        RaycastResult result;
        if (!startRef) {
            result.status = PathStatus::StartNotFound;
            return result;
        }

        std::vector<dtPolyRef> path(static_cast<std::size_t>(maxPath));
        int pathCount = 0;
        float t = 0.0f;
        float hitNormal[3] = {0.0f, 0.0f, 0.0f};
        dtStatus st = query_->raycast(startRef, s, e, f, &t, hitNormal, path.data(), &pathCount, maxPath);
        if (dtStatusFailed(st))
            return result;

        result.status = dtStatusDetail(st, DT_BUFFER_TOO_SMALL) ? PathStatus::BufferFull : PathStatus::Success;
        result.hit = t < 1.0f;
        result.t = t;
        float hitPos[3];
        dtVlerp(hitPos, s, e, t < 1.0f ? t : 1.0f);
        result.position = detourToWow(hitPos);
        result.normal = detourToWow(hitNormal);
        path.resize(static_cast<std::size_t>(pathCount));
        result.path = std::move(path);
        return result;
    }

    // Returns the point on polygon `poly_ref` closest to `position` (using the detail
    // mesh for height). Throws std::runtime_error if poly_ref is invalid.
    Point3 closest_point_on_poly(dtPolyRef poly_ref, Point3 position) const {
        ensureFresh();
        float pos[3];
        wowToDetour(position, pos);
        float closest[3];
        bool overPoly = false;
        if (dtStatusFailed(query_->closestPointOnPoly(poly_ref, pos, closest, &overPoly)))
            throw std::runtime_error("closestPointOnPoly failed: invalid poly_ref");
        return detourToWow(closest);
    }

    // Returns `position` itself if it lies inside polygon `poly_ref`'s horizontal
    // bounds, otherwise the closest point on its boundary. Throws std::runtime_error if poly_ref
    // is invalid.
    Point3 closest_point_on_poly_boundary(dtPolyRef poly_ref, Point3 position) const {
        ensureFresh();
        float pos[3];
        wowToDetour(position, pos);
        float closest[3];
        if (dtStatusFailed(query_->closestPointOnPolyBoundary(poly_ref, pos, closest)))
            throw std::runtime_error("closestPointOnPolyBoundary failed: invalid poly_ref");
        return detourToWow(closest);
    }

    // Returns the surface height of polygon `poly_ref` at `position`, from its detail
    // mesh. Throws std::runtime_error if poly_ref is invalid or position lies outside
    // the polygon.
    float get_poly_height(dtPolyRef poly_ref, Point3 position) const {
        ensureFresh();
        float pos[3];
        wowToDetour(position, pos);
        float height = 0.0f;
        if (dtStatusFailed(query_->getPolyHeight(poly_ref, pos, &height)))
            throw std::runtime_error(
                "getPolyHeight failed: invalid poly_ref, or position outside its (x, y) bounds");
        return height;
    }

    // Measures the distance from `position` to the nearest navmesh wall within
    // `max_radius`, starting the search from `start_poly` (found via a nearest-poly
    // search if not given). Throws std::invalid_argument if max_radius is not positive.
    //
    // distance stays
    // -1 (position/normal untouched) only when no polygon at all is found near position --
    // a common non-error, so this doesn't throw for it. If a polygon is found but no wall
    // lies within max_radius (open area), Detour itself returns distance == max_radius
    // with position left at whatever was passed in -- that's upstream dtNavMeshQuery's own
    // behavior, not a distinct "not found" case; check `distance < max_radius` to tell a
    // real wall hit apart from "nothing that close".
    WallDistanceResult find_distance_to_wall(Point3 position, std::optional<dtPolyRef> start_poly,
                                              float max_radius, const QueryFilter* filter) const {
        ensureFresh();
        if (max_radius <= 0)
            throw std::invalid_argument("max_radius must be positive");
        const dtQueryFilter* f = &filterOrDefault(filter);
        float pos[3];
        wowToDetour(position, pos);

        dtPolyRef ref = start_poly.value_or(0);
        if (!ref) {
            float ext[3];
            extentsOrDefault(std::nullopt, ext);
            float nearestPt[3];
            query_->findNearestPoly(pos, ext, f, &ref, nearestPt);
        }

        WallDistanceResult result;
        if (!ref)
            return result;

        float dist = 0.0f, hitPos[3] = {0.0f, 0.0f, 0.0f}, hitNormal[3] = {0.0f, 0.0f, 0.0f};
        if (dtStatusFailed(query_->findDistanceToWall(ref, pos, max_radius, f, &dist, hitPos, hitNormal)))
            return result;
        result.distance = dist;
        result.position = detourToWow(hitPos);
        result.normal = detourToWow(hitNormal);
        return result;
    }

    // World-navigation introspection: raw dtPoly/dtMeshTile/dtOffMeshConnection data for
    // Python, without exposing Detour pointers. Never throws for an *invalid* poly_ref --
    // returns None (or an empty list) instead, since callers are expected to probe refs of
    // unknown provenance. Does throw (via ensureFresh()) if the query object itself is
    // stale, i.e. the map was reloaded or freed out from under it.

    // Returns all structural data for polygon `ref`, or nullopt if it's invalid.
    std::optional<PolyInfo> get_poly(dtPolyRef ref) const {
        ensureFresh();
        const dtMeshTile* tile = nullptr;
        const dtPoly* poly = nullptr;
        if (!ref || dtStatusFailed(mesh_->getTileAndPolyByRef(ref, &tile, &poly)))
            return std::nullopt;
        return buildPolyInfo(ref, tile, poly);
    }

    // Returns the TileInfo of the tile containing polygon `ref`, or nullopt if it's
    // invalid. Declared here, defined after NavMesh (below) since it needs
    // owner_->tileUsesLiquids() (a NavMesh method) to be a complete-type call.
    std::optional<TileInfo> get_tile_info(dtPolyRef ref) const;

    // Returns a PolyInfo for every polygon of the tile at (`tile_x`, `tile_y`,
    // `tile_layer`), or an empty vector if no such tile is loaded.
    std::vector<PolyInfo> get_tile_polys(int tile_x, int tile_y, int tile_layer) const {
        ensureFresh();
        const dtMeshTile* tile = mesh_->getTileAt(tile_x, tile_y, tile_layer);
        std::vector<PolyInfo> result;
        if (!tile || !tile->header)
            return result;
        const dtPolyRef base = mesh_->getPolyRefBase(tile);
        result.reserve(static_cast<std::size_t>(tile->header->polyCount));
        for (int i = 0; i < tile->header->polyCount; i++)
            result.push_back(buildPolyInfo(base + static_cast<dtPolyRef>(i), tile, &tile->polys[i]));
        return result;
    }

    // Returns the off-mesh connections of the tile at (`tile_x`, `tile_y`, `tile_layer`),
    // or of every loaded tile if tile_x/tile_y are both unset. Throws
    // std::invalid_argument if only one of tile_x/tile_y is set.
    std::vector<OffMeshConnection> get_offmesh_connections(std::optional<int> tile_x,
                                                             std::optional<int> tile_y,
                                                             int tile_layer) const {
        ensureFresh();
        if (tile_x.has_value() != tile_y.has_value())
            throw std::invalid_argument("tile_x and tile_y must both be given, or both omitted");
        std::vector<OffMeshConnection> result;
        auto collect = [&](const dtMeshTile* tile) {
            if (!tile || !tile->header)
                return;
            for (int i = 0; i < tile->header->offMeshConCount; i++)
                result.push_back(buildOffMeshConnection(mesh_, tile, i));
        };
        if (tile_x.has_value() && tile_y.has_value()) {
            collect(mesh_->getTileAt(*tile_x, *tile_y, tile_layer));
            return result;
        }
        const dtNavMesh* mesh = mesh_;
        for (int i = 0; i < mesh->getMaxTiles(); i++)
            collect(mesh->getTile(i));
        return result;
    }

    // Returns a PolyInfo for every polygon passing `filter` within an axis-aligned box of
    // half-extent `radius` around `center`. Uses Detour's own BV-tree-accelerated
    // queryPolygons -- not a linear scan over every polygon in the mesh. Throws
    // std::invalid_argument if radius or max_polys (defaults to config().max_path_polys)
    // is not positive, and std::runtime_error if more than max_polys polygons are in
    // range.
    std::vector<PolyInfo> sample_polys(Point3 center, float radius, const QueryFilter* filter,
                                        std::optional<int> max_polys) const {
        ensureFresh();
        if (radius <= 0)
            throw std::invalid_argument("radius must be positive");
        const int maxPolys = max_polys.value_or(config_.max_path_polys);
        if (maxPolys <= 0)
            throw std::invalid_argument("max_polys must be positive");
        const dtQueryFilter* f = &filterOrDefault(filter);
        float c[3];
        wowToDetour(center, c);
        float ext[3] = {radius, radius, radius};
        std::vector<dtPolyRef> refs(static_cast<std::size_t>(maxPolys));
        int count = 0;
        dtStatus st = query_->queryPolygons(c, ext, f, refs.data(), &count, maxPolys);
        std::vector<PolyInfo> result;
        if (dtStatusFailed(st))
            return result;
        // queryPolygons reports overflow as a detail flag on a *success* status; without
        // this the caller would silently get an arbitrary maxPolys-sized subset.
        if (dtStatusDetail(st, DT_BUFFER_TOO_SMALL))
            throw std::runtime_error("sample_polys: more than " + std::to_string(maxPolys) +
                                      " polygons in range; raise max_polys or shrink radius");
        result.reserve(static_cast<std::size_t>(count));
        for (int i = 0; i < count; i++) {
            const dtMeshTile* tile = nullptr;
            const dtPoly* poly = nullptr;
            if (dtStatusSucceed(mesh_->getTileAndPolyByRef(refs[i], &tile, &poly)))
                result.push_back(buildPolyInfo(refs[i], tile, poly));
        }
        return result;
    }

    // Finds a path from `start` to `end`: polygon corridor (at most `max_polys` long) +
    // straight path + steering state, bundled into one Path object instead of a bare
    // point list. Failure modes are reported through the Path's status.
    Path find_path(Point3 start, Point3 end, const QueryFilter* filter,
                    std::optional<int> max_polys) const;

    // Returns the defaults this query falls back on.
    const QueryConfig& config() const { return config_; }
    // Replaces the defaults this query falls back on (does not affect the owning NavMesh).
    void set_config(QueryConfig config) { config_ = config; }

private:
    // Writes `extents`, or config().nearest_poly_extents if unset, into `out` (a
    // float[3]) in Detour axis order.
    void extentsOrDefault(std::optional<std::tuple<float, float, float>> extents, float* out) const {
        wowToDetour(extents ? *extents : config_.nearest_poly_extents, out);
    }

    // Throws std::runtime_error if the owning NavMesh has loaded or freed a map since
    // this query was created. Declared here, defined after NavMesh (below) since it
    // needs NavMesh::generation() to be a complete-type call.
    void ensureFresh() const;

    dtNavMesh* mesh_;
    dtNavMeshQuery* query_;
    QueryConfig config_;
    const NavMesh* owner_;
    int generation_;
};

// Bundles everything a caller needs to act on one find_path() result: the polygon
// corridor, the straight (funnel) path, and enough live state (a dtPathCorridor) to ask
// for a steering target as the agent moves.
class Path {
public:
    // Builds a Path from one query's results. `query` (not owned) is kept for steering;
    // `filter` is copied so later steering uses the same filter as the original search.
    // `owner`/`generation` identify the NavMesh state it came from, for ensureFresh().
    // When `corridor` is non-empty, initializes a dtPathCorridor sized for at least
    // `max_polys` polygons (plus steering headroom), positioned at `start` and ending at
    // `actual_end`.
    Path(dtNavMeshQuery* query, PathStatus status, Point3 start, Point3 requested_end,
         Point3 actual_end, dtPolyRef start_poly, dtPolyRef end_poly, std::vector<dtPolyRef> corridor,
         std::vector<StraightPathPoint> straight_path, int max_polys, dtQueryFilter filter,
         const NavMesh* owner, int generation)
        : query_(query),
          status_(status),
          start_(start),
          requested_end_(requested_end),
          actual_end_(actual_end),
          start_poly_(start_poly),
          end_poly_(end_poly),
          corridor_(std::move(corridor)),
          straight_path_(std::move(straight_path)),
          filter_(filter),
          owner_(owner),
          generation_(generation) {
        if (!corridor_.empty() && query_) {
            corridor_state_ = std::make_unique<dtPathCorridor>();
            // movePosition() merges up to 16 polygons it moved through into the front of
            // the corridor without checking its capacity (dtMergeCorridorStartMoved), so
            // a corridor with less room than that overflows the heap on the first move.
            constexpr int kMoveMergeHeadroom = 16;
            corridor_state_->init(std::max(max_polys, static_cast<int>(corridor_.size())) + kMoveMergeHeadroom);
            float startPos[3];
            wowToDetour(start_, startPos);
            corridor_state_->reset(start_poly_, startPos);
            float endPos[3];
            wowToDetour(actual_end_, endPos);
            corridor_state_->setCorridor(endPos, corridor_.data(), static_cast<int>(corridor_.size()));
        }
    }

    // Returns true if the path is usable, i.e. its status is SUCCESS, PARTIAL or
    // BUFFER_FULL.
    bool is_valid() const {
        return status_ == PathStatus::Success || status_ == PathStatus::Partial ||
               status_ == PathStatus::BufferFull;
    }
    // Returns true if the path reaches the requested end (status SUCCESS).
    bool is_complete() const { return status_ == PathStatus::Success; }
    // Returns the outcome of the query that built this path.
    PathStatus status() const { return status_; }
    // Returns the start position, clamped onto the start polygon if a corridor was found.
    Point3 start_position() const { return start_; }
    // Returns the end position as originally requested.
    Point3 requested_end_position() const { return requested_end_; }
    // Returns where the path actually ends: the requested end snapped onto the navmesh,
    // or the closest point the corridor reaches if it stops short.
    Point3 actual_end_position() const { return actual_end_; }
    // Returns the polygon the path starts in, or 0 if none was found.
    dtPolyRef start_poly() const { return start_poly_; }
    // Returns the polygon nearest the requested end, or 0 if none was found.
    dtPolyRef end_poly() const { return end_poly_; }
    // Returns the polygon corridor as originally found (not trimmed by steering).
    const std::vector<dtPolyRef>& polygon_corridor() const { return corridor_; }
    // Returns the straight (funnel) path as originally found.
    const std::vector<StraightPathPoint>& straight_path() const { return straight_path_; }

    // Returns the next point to steer toward from `current`, or nullopt if the path has
    // no corridor. A corner horizontally within `min_target_distance` (and vertically
    // within `max_target_distance`) of the agent counts as passed and is skipped. Throws
    // std::invalid_argument if either distance is not positive.
    //
    // Same algorithm as getSteerTarget() in Detour's NavMeshTesterTool sample (also used
    // by the TrinityCore/AzerothCore path generators), driven by a dtPathCorridor:
    // advances corridor_state_ to current_position via movePosition()
    // -- which clamps to the mesh and trims polygons already passed -- then asks it for
    // corners via findCorners(), and skips any corner already within min_target_distance.
    // A naive rewrite would call findStraightPath() directly over the *original, static*
    // polygon corridor on every call; that silently assumes the agent is still standing
    // in corridor_[0], which stops holding a few steps after the agent has moved past it
    // (the funnel then anchors itself to the wrong end of the corridor and its output
    // stops advancing). dtPathCorridor exists specifically to avoid re-deriving that
    // trimming logic by hand.
    std::optional<SteerTarget> get_steer_target(Point3 current, float min_target_distance,
                                                 float max_target_distance) const {
        ensureFresh();
        if (!query_ || !corridor_state_)
            return std::nullopt;
        if (min_target_distance <= 0 || max_target_distance <= 0)
            throw std::invalid_argument("min_target_distance and max_target_distance must be positive");

        float pos[3];
        wowToDetour(current, pos);
        corridor_state_->movePosition(pos, query_, &filter_);

        constexpr int kMaxCorners = 3;
        float cornerVerts[kMaxCorners * 3];
        unsigned char cornerFlags[kMaxCorners];
        dtPolyRef cornerPolys[kMaxCorners];
        const int nCorners = corridor_state_->findCorners(cornerVerts, cornerFlags, cornerPolys,
                                                            kMaxCorners, query_, &filter_);
        const float* agentPos = corridor_state_->getPos();  // mesh-clamped by movePosition()

        // The one "close enough" test for both skipping a corner and arriving at the end.
        auto withinReach = [&](const float* pt) {
            const float dx = pt[0] - agentPos[0];
            const float dz = pt[2] - agentPos[2];
            return (dx * dx + dz * dz) < min_target_distance * min_target_distance &&
                   std::fabs(pt[1] - agentPos[1]) < max_target_distance;
        };

        if (nCorners == 0) {
            // dtPathCorridor::findCorners() legitimately prunes its corner list down to
            // zero once the agent is within its own 0.01-unit tolerance of the final
            // point -- there's nothing left to steer toward because it has arrived.
            // Without this check that "arrived" case would be indistinguishable from a
            // genuinely broken corridor, both surfacing as None.
            float endPos[3];
            wowToDetour(actual_end_, endPos);
            if (withinReach(endPos)) {
                SteerTarget target;
                target.position = actual_end_;
                target.poly_ref = end_poly_;
                target.distance = dtVdist(agentPos, endPos);
                target.off_mesh = false;
                target.reached = true;
                return target;
            }
            return std::nullopt;
        }

        int idx = 0;
        while (idx < nCorners && (cornerFlags[idx] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) == 0 &&
               withinReach(&cornerVerts[idx * 3]))
            idx++;
        // Every corner returned is already within reach; if the last one is the end, the
        // agent has arrived.
        const bool passedAll = idx == nCorners;
        if (passedAll)
            idx = nCorners - 1;

        const float* pt = &cornerVerts[idx * 3];
        SteerTarget target;
        target.position = detourToWow(pt);
        target.poly_ref = cornerPolys[idx];
        target.distance = dtVdist(agentPos, pt);
        target.off_mesh = (cornerFlags[idx] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0;
        target.reached = passedAll && (cornerFlags[idx] & DT_STRAIGHTPATH_END) != 0;
        return target;
    }

private:
    // Throws std::runtime_error if the owning NavMesh has loaded or freed a map since
    // this path was created. Declared here, defined after NavMesh (below) since it needs
    // NavMesh::generation() to be a complete-type call.
    void ensureFresh() const;

    dtNavMeshQuery* query_;
    PathStatus status_;
    Point3 start_;
    Point3 requested_end_;
    Point3 actual_end_;
    dtPolyRef start_poly_;
    dtPolyRef end_poly_;
    std::vector<dtPolyRef> corridor_;
    std::vector<StraightPathPoint> straight_path_;
    std::unique_ptr<dtPathCorridor> corridor_state_;
    dtQueryFilter filter_;
    const NavMesh* owner_;
    int generation_;
};

// Status mapping: the corridor's status (SUCCESS, PARTIAL or BUFFER_FULL), unless the
// straight path over it fails, is cut off or comes back partial -- except that a
// cut-off corridor stays BUFFER_FULL.
inline Path NavigationQuery::find_path(Point3 start, Point3 end, const QueryFilter* filter,
                                        std::optional<int> max_polys) const {
    ensureFresh();
    PolygonPathResult poly = find_polygon_path(start, end, filter, max_polys);
    const int maxPolys = max_polys.value_or(config_.max_path_polys);

    if (poly.polygons.empty())
        return Path(query_, poly.status, start, end, start, poly.start_poly, poly.end_poly, {}, {},
                    maxPolys, filterOrDefault(filter), owner_, generation_);

    const Point3 clampedStart = closest_point_on_poly(poly.start_poly, start);
    const Point3 clampedEnd = closest_point_on_poly(poly.end_poly, end);
    StraightPathResult straight = find_straight_path(clampedStart, clampedEnd, poly.polygons, std::nullopt);
    // Where the corridor ends, computed the way findStraightPath() places its end point,
    // so it stays right when the straight path itself was cut off.
    const Point3 actualEnd = closest_point_on_poly_boundary(poly.polygons.back(), clampedEnd);

    PathStatus status = poly.status;
    if (straight.status != PathStatus::Success && status != PathStatus::BufferFull)
        status = straight.status;

    return Path(query_, status, clampedStart, end, actualEnd, poly.start_poly, poly.end_poly,
                poly.polygons, straight.points, maxPolys, filterOrDefault(filter), owner_, generation_);
}

// Owns the dtNavMesh/dtNavMeshQuery pair for at most one loaded map, read from a
// directory of .mmap/.mmtile files. Not copyable. Not thread-safe: Detour's query object
// carries mutable search state.
class NavMesh {
public:
    // Binds to the mmaps directory `mmaps_path`. Loads nothing until load_map(). Each
    // search may expand up to `max_nodes` polygons (Detour's node pool size). Throws
    // std::invalid_argument if max_nodes is outside [1, 65535], Detour's limit.
    NavMesh(fs::path mmaps_path, int max_nodes) : mmaps_path_(std::move(mmaps_path)), max_nodes_(max_nodes) {
        if (max_nodes <= 0 || max_nodes > DT_NULL_IDX)
            throw std::invalid_argument("max_nodes must be in [1, " + std::to_string(DT_NULL_IDX) + "]");
    }

    // Frees the loaded map, if any.
    ~NavMesh() { free_map(); }

    NavMesh(const NavMesh&) = delete;
    NavMesh& operator=(const NavMesh&) = delete;

    // Loads map `map_id` from `{map_id:03}.mmap` and every `{map_id:03}{x:02}{y:02}.mmtile`
    // present in the mmaps directory, replacing the current map. The current map is
    // freed first, so on failure nothing is loaded. Throws std::runtime_error if the
    // header is missing/invalid, a tile is invalid (see checkTileLayout()), no tile is
    // found, or Detour fails to initialize; std::bad_alloc if allocation fails.
    void load_map(unsigned int map_id) {
        free_map();

        char stem[8];
        std::snprintf(stem, sizeof(stem), "%03u", map_id);

        const fs::path mmap_path = mmaps_path_ / (std::string(stem) + ".mmap");
        // u8string(), not string(): the latter converts to the ANSI code page on Windows
        // and throws for characters it can't represent.
        const std::string mmap_path_str = mmap_path.u8string();

        std::ifstream f(mmap_path, std::ios::binary);
        if (!f)
            throw std::runtime_error("mmap header not found: " + mmap_path_str);

        // Newer generators prefix dtNavMeshParams with 'MMAP' magic + uint32 mmapVersion; older
        // files are the bare params from byte 0.
        unsigned int magic = 0;
        f.read(reinterpret_cast<char*>(&magic), sizeof(magic));
        if (!f)
            throw std::runtime_error("failed to read mmap header: " + mmap_path_str);
        if (magic == kMmapMagic) {
            unsigned int mmap_version = 0;
            f.read(reinterpret_cast<char*>(&mmap_version), sizeof(mmap_version));
            if (!f || mmap_version < kMmapVersionMin)
                throw std::runtime_error("unsupported mmap format version: " + mmap_path_str);
        } else {
            f.seekg(0);
        }

        dtNavMeshParams params;
        f.read(reinterpret_cast<char*>(&params), sizeof(params));
        if (!f)
            throw std::runtime_error("failed to read mmap params: " + mmap_path_str);
        f.close();

        DtNavMeshPtr mesh(dtAllocNavMesh());
        if (!mesh)
            throw std::bad_alloc();

        if (dtStatusFailed(mesh->init(&params)))
            throw std::runtime_error("dtNavMesh::init failed for map " + std::to_string(map_id));

        // Filled locally and moved in only once the whole map has loaded, like the mesh.
        std::unordered_map<std::uint64_t, bool> tile_uses_liquids;
        int tiles_loaded = 0;
        for (unsigned int x = 0; x < 64; x++) {
            for (unsigned int y = 0; y < 64; y++) {
                char tile_stem[16];
                std::snprintf(tile_stem, sizeof(tile_stem), "%03u%02u%02u", map_id, x, y);
                const fs::path tile_path = mmaps_path_ / (std::string(tile_stem) + ".mmtile");
                const std::string tile_path_str = tile_path.u8string();

                std::ifstream tf(tile_path, std::ios::binary);
                if (!tf)
                    continue;  // sparse 64x64 grid: most tiles simply don't exist.

                MmapTileHeader header;
                tf.read(reinterpret_cast<char*>(&header), sizeof(header));
                if (!tf)
                    throw std::runtime_error("mmtile header is truncated: " + tile_path_str);
                if (header.mmapMagic != kMmapMagic)
                    throw std::runtime_error("mmtile has wrong mmap magic (not a TrinityCore/"
                                              "AzerothCore mmap file?): " + tile_path_str);
                if (header.mmapVersion < kMmapVersionMin)
                    throw std::runtime_error(
                        "mmtile has unsupported mmap format version " +
                        std::to_string(header.mmapVersion) + " (expected >= " +
                        std::to_string(kMmapVersionMin) + "): " + tile_path_str);
                if (static_cast<int>(header.dtVersion) != DT_NAVMESH_VERSION)
                    throw std::runtime_error(
                        "mmtile wrapper header reports Detour version " +
                        std::to_string(header.dtVersion) + " (expected " +
                        std::to_string(DT_NAVMESH_VERSION) + "): " + tile_path_str);

                // The Detour payload is the trailing header.size bytes, so whatever sits
                // between our 20-byte prefix and it is header fields a newer generator
                // added. Deriving the offset this way reads those versions correctly
                // instead of starting the payload short by exactly that many bytes.
                const std::uintmax_t file_size = fs::file_size(tile_path);
                if (file_size < header.size + sizeof(MmapTileHeader))
                    throw std::runtime_error("mmtile is smaller than its own declared payload "
                                              "size: " + tile_path_str);
                const std::uintmax_t payload_offset = file_size - header.size;

                DtBuffer data(static_cast<unsigned char*>(dtAlloc(header.size, DT_ALLOC_PERM)));
                if (!data)
                    throw std::bad_alloc();

                tf.seekg(static_cast<std::streamoff>(payload_offset), std::ios::beg);
                tf.read(reinterpret_cast<char*>(data.get()), header.size);
                if (!tf)
                    throw std::runtime_error("mmtile data is truncated: " + tile_path_str);
                tf.close();

                checkTileLayout(tile_path_str, data.get(), header.size);

                // Key off the tile's own dtMeshHeader, not the file name indices: the
                // generator derives dtMeshHeader::x/y from the Detour-axis tile origin,
                // which is transposed relative to the WoW grid indices in the name.
                const dtMeshHeader* meshHeader = reinterpret_cast<const dtMeshHeader*>(data.get());
                tile_uses_liquids[tileKey(meshHeader->x, meshHeader->y, meshHeader->layer)] =
                    header.usesLiquids != 0;

                dtStatus status =
                    mesh->addTile(data.get(), static_cast<int>(header.size), DT_TILE_FREE_DATA, 0, nullptr);
                if (dtStatusFailed(status))
                    throw std::runtime_error("dtNavMesh::addTile failed for " + tile_path_str +
                                              " (status=" + std::to_string(status) + ")");
                data.release();  // ownership transferred to the tile (DT_TILE_FREE_DATA).
                tiles_loaded++;
            }
        }

        if (tiles_loaded == 0)
            throw std::runtime_error("no tiles found for map " + std::to_string(map_id));

        dtNavMeshQuery* query = dtAllocNavMeshQuery();
        if (!query)
            throw std::bad_alloc();
        if (dtStatusFailed(query->init(mesh.get(), max_nodes_))) {
            dtFreeNavMeshQuery(query);
            throw std::runtime_error("dtNavMeshQuery::init failed for map " + std::to_string(map_id));
        }

        nav_mesh_ = mesh.release();
        nav_query_ = query;
        map_id_ = map_id;
        tile_uses_liquids_ = std::move(tile_uses_liquids);
        generation_++;
    }

    // Frees the current map, if any. Safe to call repeatedly.
    //
    // Every call frees the current map's dtNavMesh/dtNavMeshQuery (if any) and bumps
    // generation_, so any NavigationQuery/Path built from the old ones will raise
    // instead of touching freed memory (see NavigationQuery::ensureFresh() below).
    void free_map() {
        if (nav_query_) {
            dtFreeNavMeshQuery(nav_query_);
            nav_query_ = nullptr;
        }
        if (nav_mesh_) {
            dtFreeNavMesh(nav_mesh_);
            nav_mesh_ = nullptr;
        }
        map_id_.reset();
        tile_uses_liquids_.clear();
        generation_++;
    }

    // Returns a counter bumped by every load_map()/free_map(), used to detect stale
    // NavigationQuery/Path objects.
    int generation() const { return generation_; }

    // Returns the uses_liquids flag of the loaded tile at (`x`, `y`, `layer`), or false
    // if no such tile is loaded.
    //
    // uses_liquids is TrinityCore/AzerothCore's own per-tile MmapTileHeader flag, not
    // part of Detour's dtMeshHeader -- tracked separately here, keyed by tile grid coords.
    bool tileUsesLiquids(int x, int y, int layer) const {
        const auto it = tile_uses_liquids_.find(tileKey(x, y, layer));
        return it != tile_uses_liquids_.end() && it->second;
    }

    // Returns a TileInfo for every loaded tile. Throws std::runtime_error if no map is
    // loaded.
    std::vector<TileInfo> get_loaded_tiles() const {
        if (!nav_mesh_)
            throw std::runtime_error("no map loaded; call load_map() first");
        std::vector<TileInfo> result;
        const dtNavMesh* mesh = nav_mesh_;
        for (int i = 0; i < mesh->getMaxTiles(); i++) {
            const dtMeshTile* tile = mesh->getTile(i);
            if (!tile || !tile->header)
                continue;
            result.push_back(buildTileInfo(tile, tileUsesLiquids(tile->header->x, tile->header->y,
                                                       tile->header->layer)));
        }
        return result;
    }

    // Returns true if a map is loaded.
    bool is_loaded() const { return nav_mesh_ != nullptr; }

    // Returns the loaded map's id, or nullopt if none is loaded.
    std::optional<unsigned int> map_id() const { return map_id_; }

    // Returns the mmaps directory this NavMesh reads from, as UTF-8.
    std::string mmaps_path() const { return mmaps_path_.u8string(); }

    // Returns a NavigationQuery bound to the currently loaded map. Cheap to construct
    // (two pointers + a small config struct copy), so a fresh one is handed out per call
    // rather than cached; the config it carries is a snapshot of query_config() at the
    // time of the call. Throws std::runtime_error if no map is loaded.
    NavigationQuery query() const {
        if (!nav_mesh_ || !nav_query_)
            throw std::runtime_error("no map loaded; call load_map() first");
        return NavigationQuery(nav_mesh_, nav_query_, query_config_, this, generation_);
    }

    // Returns the defaults handed to each NavigationQuery created by query().
    const QueryConfig& query_config() const { return query_config_; }
    // Replaces the defaults handed to NavigationQuery objects created from now on.
    void set_query_config(QueryConfig config) { query_config_ = config; }

    // Finds a path from `start` to `end` and returns its waypoints in WoW world
    // coordinates, together with whether it reaches the end (PathType). Returns empty
    // points and PathType::NotFound if no path could be found, and PathType::Partial if
    // the points stop short of the end for any reason (unreachable, or a buffer or the
    // node pool ran out).
    //
    // max_points: maximum number of straight-path points before subdivision.
    // waypoint_distance: if set, subdivides segments to maintain max distance between waypoints.
    // search_extent: half-extent, on every axis, of the nearest-poly search box.
    // centered: route through polygon portal-edge midpoints instead of the funnel
    // algorithm's taut string-pull. Trades a longer, less direct path for staying away
    // from walls (the string-pull is the shortest path, so it hugs corners whenever the
    // corridor allows it). max_points is ignored in this mode: the point count is fixed
    // by the polygon corridor (one midpoint per portal crossed, plus start/end).
    //
    // Uses a fixed 512-polygon corridor buffer. Throws std::runtime_error if no map is
    // loaded, std::invalid_argument if max_points, waypoint_distance or search_extent is
    // not positive.
    PathResult find_path(Point3 start, Point3 end, int max_points = 256,
                          std::optional<float> waypoint_distance = std::nullopt,
                          float search_extent = 50.0f, bool centered = false) {
        if (!nav_mesh_ || !nav_query_)
            throw std::runtime_error("no map loaded; call load_map() first");
        if (max_points <= 0)
            throw std::invalid_argument("max_points must be positive");
        if (waypoint_distance && *waypoint_distance <= 0)
            throw std::invalid_argument("waypoint_distance must be positive");
        if (search_extent <= 0)
            throw std::invalid_argument("search_extent must be positive");

        float s[3];
        wowToDetour(start, s);
        float e[3];
        wowToDetour(end, e);
        float extents[3] = {search_extent, search_extent, search_extent};
        dtQueryFilter filter;

        dtPolyRef start_poly, end_poly;
        float start_pt[3], end_pt[3];

        dtStatus status = nav_query_->findNearestPoly(s, extents, &filter, &start_poly, start_pt);
        if (dtStatusFailed(status) || !start_poly)
            return {{}, PathType::NotFound, start};

        status = nav_query_->findNearestPoly(e, extents, &filter, &end_poly, end_pt);
        if (dtStatusFailed(status) || !end_poly)
            return {{}, PathType::NotFound, start};

        // Farther than this from the nearest poly means the requested point is off the
        // mesh entirely, so any path found only gets close rather than truly arriving.
        const bool far_from_poly = dtVdist(s, start_pt) > kFarFromPolyDistance ||
                                    dtVdist(e, end_pt) > kFarFromPolyDistance;

        std::vector<dtPolyRef> path_polys(512);
        int path_count = 0;
        status = nav_query_->findPath(start_poly, end_poly, start_pt, end_pt, &filter,
                                       path_polys.data(), &path_count,
                                       static_cast<int>(path_polys.size()));
        if (dtStatusFailed(status) || path_count == 0)
            return {{}, PathType::NotFound, start};

        // Also false when the corridor outgrew its buffer or the search ran out of nodes.
        const bool reaches_end = path_polys[path_count - 1] == end_poly;
        // A corridor stopping short can only end on its own last polygon; clamp there
        // the way findStraightPath() does, so centered mode doesn't jump to end_poly.
        if (!reaches_end) {
            float clamped[3];
            if (dtStatusSucceed(nav_query_->closestPointOnPolyBoundary(path_polys[path_count - 1], end_pt, clamped)))
                dtVcopy(end_pt, clamped);
        }
        bool truncated = false;

        std::vector<Point3> result;

        if (centered) {
            result.push_back(detourToWow(start_pt));
            for (int i = 1; i < path_count; i++) {
                float mid[3];
                if (portalMidpoint(nav_mesh_, path_polys[i - 1], path_polys[i], mid))
                    result.push_back(detourToWow(mid));
            }
            result.push_back(detourToWow(end_pt));
        } else {
            // One spare slot, see NavigationQuery::find_straight_path().
            const int capacity = max_points + 1;
            std::vector<float> points(static_cast<size_t>(capacity) * 3);
            int point_count = 0;
            status = nav_query_->findStraightPath(start_pt, end_pt, path_polys.data(), path_count,
                                                   points.data(), nullptr, nullptr, &point_count,
                                                   capacity);
            if (dtStatusFailed(status) || point_count == 0)
                return {{}, PathType::NotFound, start};
            truncated = point_count > max_points;
            point_count = std::min(point_count, max_points);

            result.reserve(static_cast<size_t>(point_count));
            for (int i = 0; i < point_count; i++)
                result.push_back(detourToWow(&points[i * 3]));
        }

        const PathType path_type =
            (reaches_end && !far_from_poly && !truncated) ? PathType::Normal : PathType::Partial;
        const Point3 actual_end = result.back();

        if (waypoint_distance) {
            std::vector<Point3> subdivided;
            subdivided.push_back(result[0]);
            for (size_t i = 1; i < result.size(); i++) {
                const Point3 prev = subdivided.back();  // a copy: emplace_back() below reallocates
                const auto& curr = result[i];
                float dx = std::get<0>(curr) - std::get<0>(prev);
                float dy = std::get<1>(curr) - std::get<1>(prev);
                float dz = std::get<2>(curr) - std::get<2>(prev);
                float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

                if (dist > *waypoint_distance) {
                    int segments = static_cast<int>(std::ceil(dist / *waypoint_distance));
                    for (int j = 1; j < segments; j++) {
                        float t = static_cast<float>(j) / segments;
                        subdivided.emplace_back(
                            std::get<0>(prev) + dx * t,
                            std::get<1>(prev) + dy * t,
                            std::get<2>(prev) + dz * t);
                    }
                }
                subdivided.push_back(curr);
            }
            return {std::move(subdivided), path_type, actual_end};
        }

        return {std::move(result), path_type, actual_end};
    }

private:
    // Packs tile grid coordinates into a single tile_uses_liquids_ map key.
    static std::uint64_t tileKey(int x, int y, int layer) {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x) & 0xFFFFFFu) << 40) |
               (static_cast<std::uint64_t>(static_cast<std::uint32_t>(y) & 0xFFFFFFu) << 16) |
               (static_cast<std::uint32_t>(layer) & 0xFFFFu);
    }

    fs::path mmaps_path_;
    int max_nodes_;
    dtNavMesh* nav_mesh_ = nullptr;
    dtNavMeshQuery* nav_query_ = nullptr;
    std::optional<unsigned int> map_id_;
    QueryConfig query_config_;
    std::unordered_map<std::uint64_t, bool> tile_uses_liquids_;
    int generation_ = 0;
};

// Defined here, now that NavMesh::generation() is available: any NavigationQuery/Path
// built before the most recent load_map()/free_map() call on its owner is stale, and
// must refuse to touch the (freed and possibly reallocated) pointers it was built from.
inline void NavigationQuery::ensureFresh() const {
    if (!owner_ || owner_->generation() != generation_)
        throw std::runtime_error(
            "stale NavigationQuery: the owning NavMesh's map was reloaded or freed since "
            "this was obtained from NavMesh.query");
}

inline void Path::ensureFresh() const {
    if (!owner_ || owner_->generation() != generation_)
        throw std::runtime_error(
            "stale Path: the owning NavMesh's map was reloaded or freed since this Path was "
            "created");
}

inline std::optional<TileInfo> NavigationQuery::get_tile_info(dtPolyRef ref) const {
    ensureFresh();
    const dtMeshTile* tile = nullptr;
    const dtPoly* poly = nullptr;
    if (!ref || dtStatusFailed(mesh_->getTileAndPolyByRef(ref, &tile, &poly)))
        return std::nullopt;
    return buildTileInfo(tile, owner_->tileUsesLiquids(tile->header->x, tile->header->y,
                                                    tile->header->layer));
}

}  // namespace

// Docstrings below follow Google Python style and mirror src/wow_navmesh/__init__.pyi;
// keep the two in sync.
NB_MODULE(_wow_navmesh, m) {
    m.doc() = "Python bindings for Detour navmesh pathfinding over TrinityCore/AzerothCore mmaps.";

    nb::enum_<PathType>(m, "PathType", "Outcome of NavMesh.find_path().")
        .value("NOT_FOUND", PathType::NotFound, "No path at all between start and end.")
        .value("NORMAL", PathType::Normal, "Path reaches the requested end point.")
        .value("PARTIAL", PathType::Partial,
               "Path stops short of the requested end: the end is unreachable, or the path "
               "outgrew max_points, the 512-polygon corridor or the search's node pool.");

    nb::class_<PathResult>(m, "PathResult", R"doc(Result of NavMesh.find_path().

Attributes:
    points: Waypoints from start to end. Empty when path_type is NOT_FOUND.
    path_type: Whether the path reaches the requested end.
    actual_end: The last waypoint: the requested end snapped onto the navmesh for a
        NORMAL path, where the path stops for a PARTIAL one, and start for NOT_FOUND.)doc")
        .def_ro("points", &PathResult::points)
        .def_ro("path_type", &PathResult::path_type)
        .def_ro("actual_end", &PathResult::actual_end)
        .def("__repr__", [](const PathResult& self) {
            return "PathResult(points=<" + std::to_string(self.points.size()) +
                   " points>, path_type=" +
                   (self.path_type == PathType::Normal   ? "NORMAL"
                    : self.path_type == PathType::Partial ? "PARTIAL"
                                                           : "NOT_FOUND") +
                   ")";
        });

    nb::enum_<PathStatus>(m, "PathStatus", "Outcome of a NavigationQuery or Path operation.")
        .value("SUCCESS", PathStatus::Success, "The path reaches the requested end point.")
        .value("PARTIAL", PathStatus::Partial,
               "The path stops short of the end: the end is unreachable, or the search ran out "
               "of nodes (see NavMesh's max_nodes).")
        .value("NO_PATH", PathStatus::NoPath, "No route exists between start and end.")
        .value("START_NOT_FOUND", PathStatus::StartNotFound, "No polygon found near the start point.")
        .value("END_NOT_FOUND", PathStatus::EndNotFound, "No polygon found near the end point.")
        .value("BUFFER_FULL", PathStatus::BufferFull,
               "The result buffer was too small; the result holds as much as fit, from the start.")
        .value("QUERY_FAILED", PathStatus::QueryFailed, "The underlying Detour query failed.");

    nb::class_<QueryFilter>(m, "QueryFilter",
                            R"doc(Polygon filter and per-area traversal costs (wraps Detour's dtQueryFilter).

A polygon is traversable if its flags share at least one bit with include_flags and
none with exclude_flags.)doc")
        .def(nb::init<>(), "Creates a filter that accepts every polygon, with all area costs at 1.0.")
        .def_prop_rw("include_flags", &QueryFilter::include_flags, &QueryFilter::set_include_flags,
                     "Poly flags a polygon must have at least one of. Defaults to 0xFFFF.")
        .def_prop_rw("exclude_flags", &QueryFilter::exclude_flags, &QueryFilter::set_exclude_flags,
                     "Poly flags that make a polygon non-traversable. Defaults to 0.")
        .def("area_cost", &QueryFilter::area_cost, nb::arg("area"),
             R"doc(Returns the traversal cost multiplier for an area.

Args:
    area: Area id, 0 to 63.

Returns:
    The cost multiplier.

Raises:
    ValueError: If area is not in 0 to 63.)doc")
        .def("set_area_cost", &QueryFilter::set_area_cost, nb::arg("area"), nb::arg("cost"),
             R"doc(Sets the traversal cost multiplier for an area.

Args:
    area: Area id, 0 to 63.
    cost: Cost multiplier; higher makes the area less attractive to path through.

Raises:
    ValueError: If area is not in 0 to 63.)doc");

    nb::class_<QueryConfig>(m, "NavMeshQueryConfig",
                            "Defaults used by NavigationQuery methods that aren't given an explicit "
                            "override.")
        .def(nb::init<>(), "Creates a config holding the default values.")
        .def_rw("nearest_poly_extents", &QueryConfig::nearest_poly_extents,
                "Half-extents (x, y, z) of the nearest-poly search box. Defaults to "
                "(50.0, 50.0, 50.0).")
        .def_rw("max_path_polys", &QueryConfig::max_path_polys,
                "Polygon corridor buffer size. Defaults to 512.")
        .def_rw("max_straight_path_points", &QueryConfig::max_straight_path_points,
                "Straight-path buffer size. Defaults to 256.");

    nb::class_<NearestPolyResult>(m, "NearestPolyResult", "Result of NavigationQuery.find_nearest_poly().")
        .def_ro("found", &NearestPolyResult::found, "Whether a polygon was found within the search box.")
        .def_ro("poly_ref", &NearestPolyResult::poly_ref, "The nearest polygon, or 0 if not found.")
        .def_ro("position", &NearestPolyResult::position,
                "The closest point on that polygon. Only meaningful if found.")
        .def_ro("distance", &NearestPolyResult::distance,
                "Distance from the query point to position, or -1.0 if not found.");

    nb::class_<PolygonPathResult>(m, "PolygonPathResult", "Result of NavigationQuery.find_polygon_path().")
        .def_ro("status", &PolygonPathResult::status,
                "SUCCESS, PARTIAL, START_NOT_FOUND, END_NOT_FOUND, NO_PATH, BUFFER_FULL or "
                "QUERY_FAILED.")
        .def_ro("start_poly", &PolygonPathResult::start_poly, "Polygon nearest the start, or 0 if not found.")
        .def_ro("end_poly", &PolygonPathResult::end_poly, "Polygon nearest the end, or 0 if not found.")
        .def_ro("polygons", &PolygonPathResult::polygons,
                "The polygon corridor from start_poly toward end_poly. Only its first max_polys "
                "polygons if status is BUFFER_FULL.")
        .def_ro("reached_end", &PolygonPathResult::reached_end, "Whether the corridor ends in end_poly.");

    nb::class_<StraightPathPoint>(m, "StraightPathPoint", "One point of a straight path.")
        .def_ro("position", &StraightPathPoint::position, "The point.")
        .def_ro("flags", &StraightPathPoint::flags,
                "DT_STRAIGHTPATH_* bits: 0x01 start, 0x02 end, 0x04 off-mesh connection.")
        .def_ro("poly_ref", &StraightPathPoint::poly_ref, "The polygon the path enters at this point.");

    nb::class_<StraightPathResult>(m, "StraightPathResult", "Result of NavigationQuery.find_straight_path().")
        .def_ro("status", &StraightPathResult::status, "SUCCESS, PARTIAL, BUFFER_FULL or QUERY_FAILED.")
        .def_ro("points", &StraightPathResult::points,
                "The straight path, from start to end. Only its first max_points points if "
                "status is BUFFER_FULL.");

    nb::class_<MoveAlongSurfaceResult>(m, "MoveAlongSurfaceResult",
                                       "Result of NavigationQuery.move_along_surface().")
        .def_ro("status", &MoveAlongSurfaceResult::status,
                "SUCCESS, BUFFER_FULL (more than max_visited polygons were crossed; position is "
                "still right) or QUERY_FAILED.")
        .def_ro("position", &MoveAlongSurfaceResult::position,
                "Where the move ended, with its height snapped onto the navmesh surface.")
        .def_ro("visited", &MoveAlongSurfaceResult::visited,
                "Polygons crossed during the move. Unless status is BUFFER_FULL, the last one "
                "contains position.");

    nb::class_<RaycastResult>(m, "RaycastResult", R"doc(Result of NavigationQuery.raycast().

The hit fields are only meaningful if status is SUCCESS or BUFFER_FULL.)doc")
        .def_ro("status", &RaycastResult::status,
                "SUCCESS, BUFFER_FULL (more than max_path polygons were crossed; the hit fields "
                "are still right), START_NOT_FOUND or QUERY_FAILED.")
        .def_ro("hit", &RaycastResult::hit, "Whether the ray hit a navmesh wall before reaching end.")
        .def_ro("position", &RaycastResult::position, "The hit point, or end if nothing was hit.")
        .def_ro("normal", &RaycastResult::normal,
                "The wall normal at the hit point, (0, 0, 0) if nothing was hit.")
        .def_ro("t", &RaycastResult::t,
                "Fraction of the way from start to end where the hit occurred. A very large "
                "value if nothing was hit.")
        .def_ro("path", &RaycastResult::path,
                "Polygons crossed by the ray. Only the first max_path if status is BUFFER_FULL.");

    nb::class_<WallDistanceResult>(m, "WallDistanceResult", "Result of NavigationQuery.find_distance_to_wall().")
        .def_ro("distance", &WallDistanceResult::distance,
                "Distance to the nearest wall; equal to max_radius if no wall is that close, "
                "-1.0 if no polygon was found near the position.")
        .def_ro("position", &WallDistanceResult::position,
                "The nearest wall point. Only meaningful if distance < max_radius.")
        .def_ro("normal", &WallDistanceResult::normal,
                "The wall normal at position. Only meaningful if distance < max_radius.");

    nb::enum_<PolyType>(m, "PolyType", "Kind of a navmesh polygon.")
        .value("GROUND", PolyType::Ground,
               "A standard convex walkable polygon that is part of the mesh surface.")
        .value("OFFMESH_CONNECTION", PolyType::OffMeshConnection,
               "A 2-vertex off-mesh connection (jump/teleport/etc.).");

    nb::class_<PolyInfo>(m, "PolyInfo", R"doc(Raw structural data for one navmesh polygon.

area, flags and geometry are raw navmesh data. They don't by themselves mean "road",
"water", "building", etc.; that classification is left to a layer built on top of this
API.)doc")
        .def_ro("ref", &PolyInfo::ref, "The polygon's reference.")
        .def_ro("type", &PolyInfo::type, "Ground polygon or off-mesh connection.")
        .def_ro("flags", &PolyInfo::flags, "Poly flags, as set by the mmap generator.")
        .def_ro("area", &PolyInfo::area, "Area id, as set by the mmap generator.")
        .def_ro("center", &PolyInfo::center,
                "Vertex centroid (Detour has no dedicated per-poly center function).")
        .def_ro("vertices", &PolyInfo::vertices, "The polygon's vertices.")
        .def_ro("neighbors", &PolyInfo::neighbors,
                "Polygons reachable across this polygon's edges. Border edges are never "
                "included. A ref can repeat: a neighbour touching several edge segments of a "
                "tile border has one link per segment.")
        .def_ro("tile_x", &PolyInfo::tile_x, "X grid coordinate of the containing tile.")
        .def_ro("tile_y", &PolyInfo::tile_y, "Y grid coordinate of the containing tile.")
        .def_ro("tile_layer", &PolyInfo::tile_layer, "Layer of the containing tile.");

    nb::class_<TileInfo>(m, "TileInfo", "Structural data for one loaded navmesh tile.")
        .def_ro("x", &TileInfo::x, "Tile grid x coordinate (Detour's, not the .mmtile file name's).")
        .def_ro("y", &TileInfo::y, "Tile grid y coordinate (Detour's, not the .mmtile file name's).")
        .def_ro("layer", &TileInfo::layer, "Tile layer.")
        .def_ro("bounds_min", &TileInfo::bounds_min, "Minimum corner of the tile's bounding box.")
        .def_ro("bounds_max", &TileInfo::bounds_max, "Maximum corner of the tile's bounding box.")
        .def_ro("poly_count", &TileInfo::poly_count, "Number of polygons in the tile.")
        .def_ro("uses_liquids", &TileInfo::uses_liquids,
                "The tile's mmap header flag for liquid handling. A tile-level flag, not a "
                "per-polygon \"is water\" classification.");

    nb::class_<OffMeshConnection>(m, "OffMeshConnection",
                                  "Raw off-mesh connection data (jump/ferry/teleporter/door/etc.).\n\n"
                                  "No WoW-specific semantics attached.")
        .def_ro("ref", &OffMeshConnection::ref, "The connection's polygon reference.")
        .def_ro("start", &OffMeshConnection::start, "Start point.")
        .def_ro("end", &OffMeshConnection::end, "End point.")
        .def_ro("radius", &OffMeshConnection::radius, "Endpoint radius.")
        .def_ro("flags", &OffMeshConnection::flags, "Poly flags of the connection.")
        .def_ro("area", &OffMeshConnection::area, "Area id of the connection.")
        .def_ro("bidirectional", &OffMeshConnection::bidirectional,
                "Whether the connection can be traversed end to start too.");

    nb::class_<SteerTarget>(m, "SteerTarget", "Result of Path.get_steer_target().")
        .def_ro("position", &SteerTarget::position, "The point to steer toward.")
        .def_ro("poly_ref", &SteerTarget::poly_ref, "The polygon at that point.")
        .def_ro("distance", &SteerTarget::distance, "Distance from the agent to position.")
        .def_ro("reached", &SteerTarget::reached, "Whether the agent has arrived at the end of the path.")
        .def_ro("off_mesh", &SteerTarget::off_mesh,
                "Whether position is the start of an off-mesh connection.");

    nb::class_<Path>(m, "Path", R"doc(A polygon corridor and straight path, plus state to steer along them.

Obtained from NavigationQuery.find_path(). Becomes stale, and raises RuntimeError when
used, once its NavMesh loads or frees a map.)doc")
        .def("is_valid", &Path::is_valid,
             "Returns whether the path is usable, i.e. status is SUCCESS, PARTIAL or BUFFER_FULL.")
        .def("is_complete", &Path::is_complete,
             "Returns whether the path reaches the requested end, i.e. status is SUCCESS.")
        .def_prop_ro("status", &Path::status, "Outcome of the query that built this path.")
        .def_prop_ro("start_position", &Path::start_position,
                     "Start position, clamped onto the start polygon if a corridor was found.")
        .def_prop_ro("requested_end_position", &Path::requested_end_position,
                     "End position as originally requested.")
        .def_prop_ro("actual_end_position", &Path::actual_end_position,
                     "Where the path actually ends: the requested end snapped onto the navmesh, or "
                     "the closest point the corridor reaches if it stops short.")
        .def_prop_ro("start_poly", &Path::start_poly, "Polygon the path starts in, or 0 if none was found.")
        .def_prop_ro("end_poly", &Path::end_poly,
                     "Polygon nearest the requested end, or 0 if none was found.")
        .def_prop_ro("polygon_corridor", &Path::polygon_corridor,
                     "The polygon corridor as originally found (not trimmed by steering).")
        .def_prop_ro("straight_path", &Path::straight_path,
                     "The straight (funnel) path as originally found.")
        .def("get_steer_target", &Path::get_steer_target, nb::arg("current"),
             nb::arg("min_target_distance") = 0.5f, nb::arg("max_target_distance") = 6.0f,
             R"doc(Returns the next point to steer toward along this path.

Each call advances the path's internal corridor to current, so call it with the agent's
actual position as it moves. This only computes where to go; moving the agent is up to
the caller (e.g. with NavigationQuery.move_along_surface()).

Args:
    current: The agent's current position.
    min_target_distance: Horizontal radius within which a corner counts as reached and
        is skipped in favor of the next one.
    max_target_distance: Height (z) tolerance for that same check.

Returns:
    The steering target, or None if the path has no corridor (e.g. NO_PATH). The
    target's reached attribute is True once the agent has arrived.

Raises:
    ValueError: If either distance is not positive.
    RuntimeError: If the path is stale.)doc");

    nb::class_<NavigationQuery>(m, "NavigationQuery",
                                R"doc(Navmesh queries over the map loaded in a NavMesh (wraps dtNavMeshQuery).

Obtained from NavMesh.query. Becomes stale, and raises RuntimeError when used, once its
NavMesh loads or frees a map. Not thread-safe.)doc")
        .def("find_nearest_poly", &NavigationQuery::find_nearest_poly, nb::arg("position"),
             nb::arg("extents") = nb::none(), nb::arg("filter") = nb::none(),
             R"doc(Finds the polygon nearest to a position.

Args:
    position: The point to search around.
    extents: Half-extents (x, y, z) of the search box. Defaults to
        config.nearest_poly_extents.
    filter: Polygons to consider. Defaults to all.

Returns:
    The nearest polygon; found is False if none is within the search box.)doc")
        .def("find_polygon_path", &NavigationQuery::find_polygon_path, nb::arg("start"),
             nb::arg("end"), nb::arg("filter") = nb::none(), nb::arg("max_polys") = nb::none(),
             R"doc(Finds the raw polygon corridor between two points, without a point path.

Args:
    start: Start point.
    end: End point.
    filter: Polygons the corridor may cross. Defaults to all.
    max_polys: Maximum corridor length. Defaults to config.max_path_polys.

Returns:
    The corridor; check status for failures.

Raises:
    ValueError: If max_polys is not positive.)doc")
        .def("find_straight_path", &NavigationQuery::find_straight_path, nb::arg("start"),
             nb::arg("end"), nb::arg("polygons"), nb::arg("max_points") = nb::none(),
             R"doc(Runs Detour's funnel algorithm over an already-computed polygon corridor.

Args:
    start: Start point, inside polygons[0].
    end: End point, inside polygons[-1].
    polygons: The corridor, e.g. from find_polygon_path().
    max_points: Maximum number of points. Defaults to config.max_straight_path_points.

Returns:
    The straight path; check status for failures.

Raises:
    ValueError: If polygons is empty or max_points is not positive.)doc")
        .def("move_along_surface", &NavigationQuery::move_along_surface, nb::arg("current"),
             nb::arg("desired"), nb::arg("start_poly"), nb::arg("filter") = nb::none(),
             nb::arg("max_visited") = nb::none(),
             R"doc(Slides from current toward desired across the walkable surface.

The move never leaves the navmesh: it stops or slides along walls in the way. The
resulting position's height is snapped onto the navmesh surface.

Args:
    current: Start position.
    desired: Position to move toward.
    start_poly: Polygon containing current.
    filter: Polygons the move may cross. Defaults to all.
    max_visited: Maximum number of visited polygons to record. Defaults to 32.

Returns:
    Where the move ended and the polygons it crossed.

Raises:
    ValueError: If start_poly is 0 or max_visited is not positive.)doc")
        .def("raycast", &NavigationQuery::raycast, nb::arg("start"), nb::arg("end"),
             nb::arg("start_poly") = nb::none(), nb::arg("filter") = nb::none(),
             nb::arg("max_path") = nb::none(),
             R"doc(Casts a ray along the navmesh surface from start toward end.

This is Detour's navmesh raycast, not a geometric one: it reports where the surface
ends between the two points, ignoring height.

Args:
    start: Ray origin.
    end: Ray target.
    start_poly: Polygon containing start. Found with find_nearest_poly() if omitted.
    filter: Polygons the ray may cross. Defaults to all.
    max_path: Maximum number of crossed polygons to record. Defaults to
        config.max_path_polys.

Returns:
    The hit, if any; check status for failures (no start polygon found, or an invalid
    start_poly).

Raises:
    ValueError: If max_path is not positive.)doc")
        .def("closest_point_on_poly", &NavigationQuery::closest_point_on_poly, nb::arg("poly_ref"),
             nb::arg("position"),
             R"doc(Returns the point on a polygon closest to a position.

Args:
    poly_ref: The polygon.
    position: The point to project.

Returns:
    The closest point, on the polygon's detail surface.

Raises:
    RuntimeError: If poly_ref is invalid.)doc")
        .def("closest_point_on_poly_boundary", &NavigationQuery::closest_point_on_poly_boundary,
             nb::arg("poly_ref"), nb::arg("position"),
             R"doc(Returns the closest point on a polygon's boundary.

Args:
    poly_ref: The polygon.
    position: The point to project.

Returns:
    position itself if it lies inside the polygon's horizontal (x, y) bounds, otherwise
    the closest point on the polygon's boundary.

Raises:
    RuntimeError: If poly_ref is invalid.)doc")
        .def("get_poly_height", &NavigationQuery::get_poly_height, nb::arg("poly_ref"),
             nb::arg("position"),
             R"doc(Returns the surface height of a polygon at a position.

Args:
    poly_ref: The polygon.
    position: The point; only its horizontal coordinates are used.

Returns:
    The height, from the polygon's detail mesh.

Raises:
    RuntimeError: If poly_ref is invalid or position lies outside the polygon.)doc")
        .def("find_distance_to_wall", &NavigationQuery::find_distance_to_wall, nb::arg("position"),
             nb::arg("start_poly") = nb::none(), nb::arg("max_radius") = 10.0f,
             nb::arg("filter") = nb::none(),
             R"doc(Measures the distance from a position to the nearest navmesh wall.

Args:
    position: The point to measure from.
    start_poly: Polygon containing position. Found with find_nearest_poly() if omitted.
    max_radius: Search radius.
    filter: Polygons the search may cross. Defaults to all.

Returns:
    The nearest wall. distance equals max_radius if no wall is that close, and is -1.0
    if no polygon was found near position.

Raises:
    ValueError: If max_radius is not positive.)doc")
        .def("find_path", &NavigationQuery::find_path, nb::arg("start"), nb::arg("end"),
             nb::arg("filter") = nb::none(), nb::arg("max_polys") = nb::none(), nb::keep_alive<0, 1>(),
             R"doc(Finds a path between two points, ready for steering.

Runs find_polygon_path(), then find_straight_path() over its corridor.

Args:
    start: Start point.
    end: End point.
    filter: Polygons the path may cross. Defaults to all.
    max_polys: Maximum corridor length. Defaults to config.max_path_polys.

Returns:
    The path; check status or is_valid() for failures.

Raises:
    ValueError: If max_polys is not positive.)doc")
        .def("get_poly", &NavigationQuery::get_poly, nb::arg("poly_ref"),
             R"doc(Returns all structural data for a polygon.

Args:
    poly_ref: The polygon.

Returns:
    The polygon's data, or None if poly_ref is not a valid polygon.)doc")
        .def("get_tile_info", &NavigationQuery::get_tile_info, nb::arg("poly_ref"),
             "Returns the tile containing a polygon, or None if poly_ref is invalid.\n\n"
             "Args:\n    poly_ref: The polygon.")
        .def("get_tile_polys", &NavigationQuery::get_tile_polys, nb::arg("tile_x"), nb::arg("tile_y"),
             nb::arg("tile_layer") = 0,
             R"doc(Returns every polygon of a tile.

Args:
    tile_x: Tile grid x coordinate (as in TileInfo.x).
    tile_y: Tile grid y coordinate (as in TileInfo.y).
    tile_layer: Tile layer.

Returns:
    The tile's polygons, or an empty list if no such tile is loaded.)doc")
        .def("get_offmesh_connections", &NavigationQuery::get_offmesh_connections,
             nb::arg("tile_x") = nb::none(), nb::arg("tile_y") = nb::none(), nb::arg("tile_layer") = 0,
             R"doc(Returns the off-mesh connections of one tile, or of every loaded tile.

Args:
    tile_x: Tile grid x coordinate. Omit together with tile_y for all tiles.
    tile_y: Tile grid y coordinate. Omit together with tile_x for all tiles.
    tile_layer: Tile layer. Ignored when listing all tiles.

Returns:
    The off-mesh connections.

Raises:
    ValueError: If only one of tile_x/tile_y is given.)doc")
        .def("sample_polys", &NavigationQuery::sample_polys, nb::arg("center"), nb::arg("radius"),
             nb::arg("filter") = nb::none(), nb::arg("max_polys") = nb::none(),
             R"doc(Returns every polygon within radius of center.

This is an axis-aligned box query, not an exact circle: polygons slightly beyond radius
at the corners of the box can be included.

Args:
    center: Center of the search box.
    radius: Half-extent of the search box on every axis.
    filter: Polygons to include. Defaults to all.
    max_polys: Maximum number of polygons. Defaults to config.max_path_polys.

Returns:
    The polygons in range.

Raises:
    ValueError: If radius or max_polys is not positive.
    RuntimeError: If more than max_polys polygons are in range.)doc")
        .def_prop_rw("config", &NavigationQuery::config, &NavigationQuery::set_config,
                     "Defaults used by methods that aren't given an explicit override. A copy of "
                     "NavMesh.query_config taken when this query was created.");

    nb::class_<NavMesh>(m, "NavMesh", R"doc(A Detour navmesh bound to a directory of .mmap/.mmtile files.

Holds at most one loaded map at a time. Not thread-safe; use one NavMesh per thread.
Can be used as a context manager, which frees the loaded map on exit.)doc")
        .def(nb::init<fs::path, int>(), nb::arg("mmaps_path"), nb::arg("max_nodes") = 2048,
             R"doc(Binds to an mmaps directory. Nothing is loaded until load_map().

Args:
    mmaps_path: Directory containing the .mmap/.mmtile files.
    max_nodes: Node pool size for path searches, up to 65535. A search that runs out
        of nodes returns a PARTIAL path; raise this for long cross-continent routes.

Raises:
    ValueError: If max_nodes is not in 1 to 65535.)doc")
        .def("load_map", &NavMesh::load_map, nb::arg("map_id"),
             R"doc(Loads the navmesh for a map, replacing any map currently loaded.

The current map is freed first, so on failure no map is loaded. Any NavigationQuery or
Path obtained before this call becomes stale.

Args:
    map_id: WoW map id, e.g. 0 for Eastern Kingdoms.

Raises:
    RuntimeError: If the mmap files are missing, truncated, or built for a different
        dtPolyRef layout than this extension was compiled for.)doc")
        .def("free_map", &NavMesh::free_map,
             "Releases the currently loaded map, if any.\n\n"
             "Any NavigationQuery or Path obtained before this call becomes stale.")
        .def("find_path", &NavMesh::find_path, nb::arg("start"), nb::arg("end"),
             nb::arg("max_points") = 256, nb::arg("waypoint_distance") = nb::none(),
             nb::arg("search_extent") = 50.0f, nb::arg("centered") = false,
             R"doc(Finds a path between two points.

Args:
    start: Start point.
    end: End point.
    max_points: Maximum number of waypoints before subdivision. Ignored when centered
        is True.
    waypoint_distance: If set, subdivides segments so consecutive waypoints are at most
        this far apart.
    search_extent: Half-extent of the box searched for the nearest polygon to start and
        end. Increase for looser matching, decrease for tighter.
    centered: If True, route through polygon portal midpoints instead of the
        shortest-path funnel algorithm. Produces a longer, less direct path that stays
        away from walls instead of hugging corners.

Returns:
    The path. points is empty and path_type is NOT_FOUND if no path was found;
    path_type is PARTIAL if the points stop short of end.

Raises:
    RuntimeError: If no map is loaded.
    ValueError: If max_points, waypoint_distance or search_extent is not positive.)doc")
        .def("get_loaded_tiles", &NavMesh::get_loaded_tiles,
             "Returns every currently loaded tile.\n\n"
             "Raises:\n    RuntimeError: If no map is loaded.")
        .def_prop_ro("is_loaded", &NavMesh::is_loaded, "Whether a map is loaded.")
        .def_prop_ro("map_id", &NavMesh::map_id, "The loaded map's id, or None if no map is loaded.")
        .def_prop_ro("mmaps_path", &NavMesh::mmaps_path, "The mmaps directory this NavMesh reads from.")
        .def_prop_ro("query", &NavMesh::query, nb::keep_alive<0, 1>(),
                     "A new NavigationQuery bound to the currently loaded map.\n\n"
                     "Raises:\n    RuntimeError: If no map is loaded.")
        .def_prop_rw("query_config", &NavMesh::query_config, &NavMesh::set_query_config,
                     "Defaults copied into each NavigationQuery obtained from query.")
        .def(
            "__enter__", [](NavMesh& self) -> NavMesh& { return self; }, nb::rv_policy::reference,
            "Returns this NavMesh.")
        .def(
            "__exit__",
            [](NavMesh& self, nb::object exc_type, nb::object exc_value, nb::object traceback) {
                (void)exc_type;
                (void)exc_value;
                (void)traceback;
                self.free_map();
            },
            nb::arg("exc_type").none(), nb::arg("exc_value").none(), nb::arg("traceback").none(),
            "Frees the loaded map. Does not suppress exceptions.")
        .def("__repr__", [](const NavMesh& self) {
            return nb::str("NavMesh(mmaps_path={!r}, map_id={!r})").format(self.mmaps_path(), self.map_id());
        });
}
