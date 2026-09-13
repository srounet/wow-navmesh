#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

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
#include "DetourPathCorridor.h"

namespace nb = nanobind;
namespace fs = std::filesystem;

namespace {

// TrinityCore/AzerothCore's mmap generator writes tile files with this magic value,
// independent of the tile's dtPolyRef width. mmapVersion itself is bumped by both forks
// whenever generator *behavior* changes (e.g. a pathing fix requiring regeneration) —
// TrinityCore's 3.3.5 branch is at 15 and AzerothCore's master at 20 as of writing, while
// MmapTileHeader's on-disk layout has stayed the same since at least version 4. So we
// only reject versions old enough to predate that header shape, rather than pinning to
// one exact number that the next upstream bump would immediately break again.
constexpr unsigned int kMmapMagic = 0x4d4d4150;  // 'MMAP'
constexpr unsigned int kMmapVersionMin = 4;

// Header of each .mmap tile file produced by TrinityCore/AzerothCore's mmap generator.
struct MmapTileHeader {
    unsigned int mmapMagic;
    unsigned int dtVersion;
    unsigned int mmapVersion;
    unsigned int size;
    bool usesLiquids;
    char padding[3];
};

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

// Validates that a raw tile blob (as read from a .mmtile file, right after the
// MmapTileHeader) matches the dtPolyRef width this extension was built with. Without this
// check, a tile built with the other width still passes dtNavMesh::addTile()'s magic/version
// check (those don't encode polyref width) but every section after dtLink is read from the
// wrong offset -- silent corruption rather than a clean failure. Returns on success; throws
// std::runtime_error with a diagnostic message otherwise.
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
    void operator()(dtNavMesh* m) const {
        if (m)
            dtFreeNavMesh(m);
    }
};
using DtNavMeshPtr = std::unique_ptr<dtNavMesh, DtNavMeshDeleter>;

struct DtAllocDeleter {
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
inline void wowToDetour(const Point3& p, float* out) {
    out[0] = std::get<1>(p);
    out[1] = std::get<2>(p);
    out[2] = std::get<0>(p);
}
inline Point3 detourToWow(const float* p) { return Point3(p[2], p[0], p[1]); }

// Mirrors MaNGOS PathFinder's PATHFIND_* distinction: whether the path actually reaches
// the requested end, or only gets as close as the navmesh allows.
enum class PathType {
    NotFound = 0,  // no path at all (start/end off the mesh, or no route between them)
    Normal = 1,    // path reaches the requested end point
    Partial = 2,   // path only reaches as close to the end as the navmesh allows
};

struct PathResult {
    std::vector<Point3> points;
    PathType path_type;
    Point3 actual_end;  // last point actually reached; differs from the requested end
                         // when path_type is Partial or NotFound.
};

// Distance (world units) beyond which a nearest-poly match is considered "off the mesh"
// rather than a walkable point close enough to start/end from. Matches MaNGOS PathFinder.
constexpr float kFarFromPolyDistance = 7.0f;

// Midpoint of the shared portal edge between two adjacent polygons in a path corridor,
// mirroring dtNavMeshQuery::getEdgeMidPoint (private upstream, so reimplemented here).
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

// Explicit outcome of a Detour query, replacing the old habit of collapsing every
// failure mode into an empty result. Distinguishes "no path exists" from "start/end
// isn't on the mesh" from "the result buffer was too small" from "Detour itself failed".
enum class PathStatus {
    Success = 0,
    Partial,
    NoPath,
    StartNotFound,
    EndNotFound,
    InvalidStart,
    InvalidEnd,
    BufferFull,
    QueryFailed,
};

// Thin Python-friendly wrapper over dtQueryFilter. Kept intentionally close to the
// Detour type -- area costs are already extensible via setAreaCost, no need to invent a
// parallel mechanism for it.
class QueryFilter {
public:
    unsigned short include_flags() const { return filter_.getIncludeFlags(); }
    void set_include_flags(unsigned short f) { filter_.setIncludeFlags(f); }
    unsigned short exclude_flags() const { return filter_.getExcludeFlags(); }
    void set_exclude_flags(unsigned short f) { filter_.setExcludeFlags(f); }
    float area_cost(int area) const { return filter_.getAreaCost(area); }
    void set_area_cost(int area, float cost) { filter_.setAreaCost(area, cost); }
    const dtQueryFilter& raw() const { return filter_; }

private:
    dtQueryFilter filter_;
};

// Replaces the magic constants (50-unit search extent, 512-poly buffer) that used to be
// hardcoded inside find_path().
struct QueryConfig {
    std::tuple<float, float, float> nearest_poly_extents{50.0f, 50.0f, 50.0f};
    int max_path_polys = 512;
    int max_straight_path_points = 256;
};

struct NearestPolyResult {
    bool found = false;
    dtPolyRef poly_ref = 0;
    Point3 position{0.0f, 0.0f, 0.0f};
    float distance = -1.0f;
};

struct PolygonPathResult {
    PathStatus status = PathStatus::QueryFailed;
    dtPolyRef start_poly = 0;
    dtPolyRef end_poly = 0;
    std::vector<dtPolyRef> polygons;
    bool reached_end = false;
};

struct StraightPathPoint {
    Point3 position{0.0f, 0.0f, 0.0f};
    unsigned char flags = 0;
    dtPolyRef poly_ref = 0;
};

struct StraightPathResult {
    PathStatus status = PathStatus::QueryFailed;
    std::vector<StraightPathPoint> points;
};

struct MoveAlongSurfaceResult {
    PathStatus status = PathStatus::QueryFailed;
    Point3 position{0.0f, 0.0f, 0.0f};
    std::vector<dtPolyRef> visited;
};

struct RaycastResult {
    bool hit = false;
    Point3 position{0.0f, 0.0f, 0.0f};
    Point3 normal{0.0f, 0.0f, 0.0f};
    float t = 0.0f;
    std::vector<dtPolyRef> path;
};

struct WallDistanceResult {
    float distance = -1.0f;
    Point3 position{0.0f, 0.0f, 0.0f};
    Point3 normal{0.0f, 0.0f, 0.0f};
};

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

inline std::vector<Point3> polyVertices(const dtMeshTile* tile, const dtPoly* poly) {
    std::vector<Point3> verts;
    verts.reserve(poly->vertCount);
    for (int i = 0; i < poly->vertCount; i++)
        verts.push_back(detourToWow(&tile->verts[poly->verts[i] * 3]));
    return verts;
}

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
    NavigationQuery(dtNavMesh* mesh, dtNavMeshQuery* query, QueryConfig config, const NavMesh* owner,
                     int generation)
        : mesh_(mesh), query_(query), config_(config), owner_(owner), generation_(generation) {}

    NearestPolyResult find_nearest_poly(Point3 position,
                                         std::optional<std::tuple<float, float, float>> extents,
                                         const QueryFilter* filter) const {
        ensureFresh();
        const dtQueryFilter defaultFilter;
        const dtQueryFilter* f = filter ? &filter->raw() : &defaultFilter;
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

    // Raw Detour corridor: start/end polygon plus the polygon chain between them. Kept
    // separate from find_straight_path/find_path so callers who only need the corridor
    // (e.g. to feed it elsewhere) don't pay for straight-path generation they don't want.
    PolygonPathResult find_polygon_path(Point3 start, Point3 end, const QueryFilter* filter,
                                         std::optional<int> max_polys) const {
        ensureFresh();
        const dtQueryFilter defaultFilter;
        const dtQueryFilter* f = filter ? &filter->raw() : &defaultFilter;
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
            result.status =
                dtStatusDetail(st, DT_BUFFER_TOO_SMALL) ? PathStatus::BufferFull : PathStatus::QueryFailed;
            return result;
        }
        if (count == 0) {
            result.status = PathStatus::NoPath;
            return result;
        }
        polys.resize(static_cast<std::size_t>(count));
        result.reached_end = polys.back() == result.end_poly;
        result.polygons = std::move(polys);
        result.status = result.reached_end ? PathStatus::Success : PathStatus::Partial;
        return result;
    }

    // Detour's findStraightPath() over an already-computed polygon corridor. Does not
    // itself locate start/end polygons -- pass the corridor from find_polygon_path().
    StraightPathResult find_straight_path(Point3 start, Point3 end,
                                           const std::vector<dtPolyRef>& polygons,
                                           const QueryFilter* filter,
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

        std::vector<float> pts(static_cast<std::size_t>(maxPts) * 3);
        std::vector<unsigned char> flags(static_cast<std::size_t>(maxPts));
        std::vector<dtPolyRef> refs(static_cast<std::size_t>(maxPts));
        int count = 0;
        dtStatus st = query_->findStraightPath(s, e, polygons.data(), static_cast<int>(polygons.size()),
                                                pts.data(), flags.data(), refs.data(), &count, maxPts);

        StraightPathResult result;
        if (dtStatusFailed(st)) {
            result.status =
                dtStatusDetail(st, DT_BUFFER_TOO_SMALL) ? PathStatus::BufferFull : PathStatus::QueryFailed;
            return result;
        }
        result.points.reserve(static_cast<std::size_t>(count));
        for (int i = 0; i < count; i++) {
            StraightPathPoint p;
            p.position = detourToWow(&pts[static_cast<std::size_t>(i) * 3]);
            p.flags = flags[static_cast<std::size_t>(i)];
            p.poly_ref = refs[static_cast<std::size_t>(i)];
            result.points.push_back(p);
        }
        result.status = dtStatusDetail(st, DT_PARTIAL_RESULT) ? PathStatus::Partial : PathStatus::Success;
        return result;
    }

    // dtNavMeshQuery::moveAlongSurface(): slides a point across the walkable surface
    // toward a desired position, without leaving the mesh. Central to steering: callers
    // use it to advance an agent's actual position each tick.
    MoveAlongSurfaceResult move_along_surface(Point3 current, Point3 desired, dtPolyRef start_poly,
                                               const QueryFilter* filter,
                                               std::optional<int> max_visited) const {
        ensureFresh();
        if (!start_poly)
            throw std::invalid_argument("start_poly must be non-zero");
        const dtQueryFilter defaultFilter;
        const dtQueryFilter* f = filter ? &filter->raw() : &defaultFilter;
        const int maxVisited = max_visited.value_or(32);
        if (maxVisited <= 0)
            throw std::invalid_argument("max_visited must be positive");

        float s[3];
        wowToDetour(current, s);
        float e[3];
        wowToDetour(desired, e);
        float resultPos[3] = {0.0f, 0.0f, 0.0f};
        std::vector<dtPolyRef> visited(static_cast<std::size_t>(maxVisited));
        int visitedCount = 0;
        dtStatus st = query_->moveAlongSurface(start_poly, s, e, f, resultPos, visited.data(),
                                                &visitedCount, maxVisited);

        MoveAlongSurfaceResult result;
        result.status = dtStatusFailed(st) ? PathStatus::QueryFailed : PathStatus::Success;
        result.position = detourToWow(resultPos);
        visited.resize(static_cast<std::size_t>(visitedCount));
        result.visited = std::move(visited);
        return result;
    }

    // Detour's navmesh-surface raycast -- not a geometric approximation. If start_poly
    // isn't given, it's found via find_nearest_poly first.
    RaycastResult raycast(Point3 start, Point3 end, std::optional<dtPolyRef> start_poly,
                           const QueryFilter* filter, std::optional<int> max_path) const {
        ensureFresh();
        const dtQueryFilter defaultFilter;
        const dtQueryFilter* f = filter ? &filter->raw() : &defaultFilter;
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
        if (!startRef)
            return result;

        const int maxPath = max_path.value_or(config_.max_path_polys);
        if (maxPath <= 0)
            throw std::invalid_argument("max_path must be positive");
        std::vector<dtPolyRef> path(static_cast<std::size_t>(maxPath));
        int pathCount = 0;
        float t = 0.0f;
        float hitNormal[3] = {0.0f, 0.0f, 0.0f};
        dtStatus st = query_->raycast(startRef, s, e, f, &t, hitNormal, path.data(), &pathCount, maxPath);
        if (dtStatusFailed(st))
            return result;

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

    Point3 closest_point_on_poly_boundary(dtPolyRef poly_ref, Point3 position) const {
        ensureFresh();
        float pos[3];
        wowToDetour(position, pos);
        float closest[3];
        if (dtStatusFailed(query_->closestPointOnPolyBoundary(poly_ref, pos, closest)))
            throw std::runtime_error("closestPointOnPolyBoundary failed: invalid poly_ref");
        return detourToWow(closest);
    }

    float get_poly_height(dtPolyRef poly_ref, Point3 position) const {
        ensureFresh();
        float pos[3];
        wowToDetour(position, pos);
        float height = 0.0f;
        if (dtStatusFailed(query_->getPolyHeight(poly_ref, pos, &height)))
            throw std::runtime_error(
                "getPolyHeight failed: invalid poly_ref, or position outside its xz-bounds");
        return height;
    }

    // Distance from a point to the nearest navmesh wall within max_radius. distance stays
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
        const dtQueryFilter defaultFilter;
        const dtQueryFilter* f = filter ? &filter->raw() : &defaultFilter;
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
    std::optional<PolyInfo> get_poly(dtPolyRef ref) const {
        ensureFresh();
        const dtMeshTile* tile = nullptr;
        const dtPoly* poly = nullptr;
        if (!ref || dtStatusFailed(mesh_->getTileAndPolyByRef(ref, &tile, &poly)))
            return std::nullopt;
        return buildPolyInfo(ref, tile, poly);
    }

    std::optional<PolyType> get_poly_type(dtPolyRef ref) const {
        ensureFresh();
        const dtMeshTile* tile = nullptr;
        const dtPoly* poly = nullptr;
        if (!ref || dtStatusFailed(mesh_->getTileAndPolyByRef(ref, &tile, &poly)))
            return std::nullopt;
        return poly->getType() == DT_POLYTYPE_GROUND ? PolyType::Ground : PolyType::OffMeshConnection;
    }

    std::optional<int> get_poly_area(dtPolyRef ref) const {
        ensureFresh();
        const dtMeshTile* tile = nullptr;
        const dtPoly* poly = nullptr;
        if (!ref || dtStatusFailed(mesh_->getTileAndPolyByRef(ref, &tile, &poly)))
            return std::nullopt;
        return static_cast<int>(poly->getArea());
    }

    std::optional<int> get_poly_flags(dtPolyRef ref) const {
        ensureFresh();
        const dtMeshTile* tile = nullptr;
        const dtPoly* poly = nullptr;
        if (!ref || dtStatusFailed(mesh_->getTileAndPolyByRef(ref, &tile, &poly)))
            return std::nullopt;
        return static_cast<int>(poly->flags);
    }

    std::optional<Point3> get_poly_center(dtPolyRef ref) const {
        ensureFresh();
        const dtMeshTile* tile = nullptr;
        const dtPoly* poly = nullptr;
        if (!ref || dtStatusFailed(mesh_->getTileAndPolyByRef(ref, &tile, &poly)))
            return std::nullopt;
        return polyCentroid(tile, poly);
    }

    std::vector<Point3> get_poly_vertices(dtPolyRef ref) const {
        ensureFresh();
        const dtMeshTile* tile = nullptr;
        const dtPoly* poly = nullptr;
        if (!ref || dtStatusFailed(mesh_->getTileAndPolyByRef(ref, &tile, &poly)))
            return {};
        return polyVertices(tile, poly);
    }

    std::vector<dtPolyRef> get_poly_neighbors(dtPolyRef ref) const {
        ensureFresh();
        const dtMeshTile* tile = nullptr;
        const dtPoly* poly = nullptr;
        if (!ref || dtStatusFailed(mesh_->getTileAndPolyByRef(ref, &tile, &poly)))
            return {};
        return polyNeighbors(tile, poly);
    }

    // Declared here, defined after NavMesh (below) since it needs owner_->tileUsesLiquids()
    // (a NavMesh method) to be a complete-type call.
    std::optional<TileInfo> get_tile_info(dtPolyRef ref) const;

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

    // Leaving both tile_x/tile_y unset enumerates every loaded tile's connections.
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

    // Box query around center using Detour's own BV-tree-accelerated queryPolygons --
    // not a linear scan over every polygon in the mesh.
    std::vector<PolyInfo> sample_polys(Point3 center, float radius, const QueryFilter* filter,
                                        std::optional<int> max_polys) const {
        ensureFresh();
        if (radius <= 0)
            throw std::invalid_argument("radius must be positive");
        const int maxPolys = max_polys.value_or(config_.max_path_polys);
        if (maxPolys <= 0)
            throw std::invalid_argument("max_polys must be positive");
        const dtQueryFilter defaultFilter;
        const dtQueryFilter* f = filter ? &filter->raw() : &defaultFilter;
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

    // High-level convenience: polygon corridor + straight path + steering state, bundled
    // into one Path object instead of a bare point list.
    Path find_path(Point3 start, Point3 end, const QueryFilter* filter,
                    std::optional<int> max_polys) const;

    const QueryConfig& config() const { return config_; }
    void set_config(QueryConfig config) { config_ = config; }

private:
    void extentsOrDefault(std::optional<std::tuple<float, float, float>> extents, float* out) const {
        const auto& e = extents ? *extents : config_.nearest_poly_extents;
        out[0] = std::get<0>(e);
        out[1] = std::get<1>(e);
        out[2] = std::get<2>(e);
    }

    // Declared here, defined after NavMesh (below) since it needs NavMesh::generation()
    // to be a complete-type call.
    void ensureFresh() const;

    dtNavMesh* mesh_;
    dtNavMeshQuery* query_;
    QueryConfig config_;
    const NavMesh* owner_;
    int generation_;
};

// Bundles everything a caller needs to act on one find_path() result: the polygon
// corridor, the straight (funnel) path, and enough live state (a dtPathCorridor) to ask
// for a steering target as the agent moves. Replaces the old "throw away everything but
// a point list" behavior of NavMesh::find_path().
class Path {
public:
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
            corridor_state_->init(std::max(max_polys, static_cast<int>(corridor_.size())));
            float startPos[3];
            wowToDetour(start_, startPos);
            corridor_state_->reset(start_poly_, startPos);
            float endPos[3];
            wowToDetour(actual_end_, endPos);
            corridor_state_->setCorridor(endPos, corridor_.data(), static_cast<int>(corridor_.size()));
        }
    }

    bool is_valid() const {
        return status_ != PathStatus::NoPath && status_ != PathStatus::StartNotFound &&
               status_ != PathStatus::EndNotFound && status_ != PathStatus::QueryFailed &&
               status_ != PathStatus::BufferFull;
    }
    bool is_complete() const { return status_ == PathStatus::Success; }
    PathStatus status() const { return status_; }
    Point3 start_position() const { return start_; }
    Point3 requested_end_position() const { return requested_end_; }
    Point3 actual_end_position() const { return actual_end_; }
    dtPolyRef start_poly() const { return start_poly_; }
    dtPolyRef end_poly() const { return end_poly_; }
    const std::vector<dtPolyRef>& polygon_corridor() const { return corridor_; }
    const std::vector<StraightPathPoint>& straight_path() const { return straight_path_; }

    // Ports AzerothCore PathGenerator::GetSteerTarget / dtCrowd's own per-tick steering:
    // advances corridor_state_ (a dtPathCorridor) to current_position via movePosition()
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

        if (nCorners == 0) {
            // dtPathCorridor::findCorners() legitimately prunes its corner list down to
            // zero once the agent is within its own 0.01-unit tolerance of the final
            // point -- there's nothing left to steer toward because it has arrived.
            // Without this check that "arrived" case would be indistinguishable from a
            // genuinely broken corridor, both surfacing as None.
            float endPos[3];
            wowToDetour(actual_end_, endPos);
            const float distToEnd = dtVdist(agentPos, endPos);
            if (distToEnd < min_target_distance) {
                SteerTarget target;
                target.position = actual_end_;
                target.poly_ref = end_poly_;
                target.distance = distToEnd;
                target.off_mesh = false;
                target.reached = true;
                return target;
            }
            return std::nullopt;
        }

        int idx = 0;
        while (idx < nCorners) {
            const float* pt = &cornerVerts[idx * 3];
            const float dx = pt[0] - agentPos[0];
            const float dz = pt[2] - agentPos[2];
            const bool horizontallyClose = (dx * dx + dz * dz) < min_target_distance * min_target_distance;
            const bool verticallyClose = std::fabs(pt[1] - agentPos[1]) < max_target_distance;
            const bool offMesh = (cornerFlags[idx] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0;
            if (offMesh || !(horizontallyClose && verticallyClose))
                break;
            idx++;
        }
        if (idx >= nCorners)
            idx = nCorners - 1;

        const float* pt = &cornerVerts[idx * 3];
        float endPos[3];
        wowToDetour(actual_end_, endPos);

        SteerTarget target;
        target.position = detourToWow(pt);
        target.poly_ref = cornerPolys[idx];
        target.distance = dtVdist(agentPos, pt);
        target.off_mesh = (cornerFlags[idx] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0;
        target.reached = (cornerFlags[idx] & DT_STRAIGHTPATH_END) != 0 &&
                          dtVdist(agentPos, endPos) < min_target_distance;
        return target;
    }

private:
    // Declared here, defined after NavMesh (below) since it needs NavMesh::generation()
    // to be a complete-type call.
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

inline Path NavigationQuery::find_path(Point3 start, Point3 end, const QueryFilter* filter,
                                        std::optional<int> max_polys) const {
    ensureFresh();
    const dtQueryFilter resolvedFilter = filter ? filter->raw() : dtQueryFilter();
    PolygonPathResult poly = find_polygon_path(start, end, filter, max_polys);
    const int maxPolys = max_polys.value_or(config_.max_path_polys);

    if (poly.polygons.empty())
        return Path(query_, poly.status, start, end, start, poly.start_poly, poly.end_poly, {}, {},
                    maxPolys, resolvedFilter, owner_, generation_);

    const Point3 clampedStart = closest_point_on_poly(poly.start_poly, start);
    const Point3 clampedEnd = closest_point_on_poly(poly.end_poly, end);
    StraightPathResult straight = find_straight_path(clampedStart, clampedEnd, poly.polygons, filter,
                                                      std::nullopt);
    const Point3 actualEnd = straight.points.empty() ? clampedStart : straight.points.back().position;

    PathStatus status = poly.status;
    if (poly.reached_end)
        status = straight.status == PathStatus::Success ? PathStatus::Success : PathStatus::Partial;
    else
        status = PathStatus::Partial;

    return Path(query_, status, clampedStart, end, actualEnd, poly.start_poly, poly.end_poly,
                poly.polygons, straight.points, maxPolys, resolvedFilter, owner_, generation_);
}

// Wraps a dtNavMesh/dtNavMeshQuery pair for one loaded map. mmap tile files store
// coordinates as (y, z, x) instead of WoW's (x, y, z); the swap happens at the
// find_path boundary so callers only ever see WoW-order coordinates.
class NavMesh {
public:
    explicit NavMesh(std::string mmaps_path) : mmaps_path_(std::move(mmaps_path)) {}

    ~NavMesh() { free_map(); }

    NavMesh(const NavMesh&) = delete;
    NavMesh& operator=(const NavMesh&) = delete;

    void load_map(unsigned int map_id) {
        free_map();

        char stem[8];
        std::snprintf(stem, sizeof(stem), "%03u", map_id);

        const fs::path mmaps_dir(mmaps_path_);
        const fs::path mmap_path = mmaps_dir / (std::string(stem) + ".mmap");

        std::ifstream f(mmap_path, std::ios::binary);
        if (!f)
            throw std::runtime_error("mmap header not found: " + mmap_path.string());

        dtNavMeshParams params;
        f.read(reinterpret_cast<char*>(&params), sizeof(params));
        if (!f)
            throw std::runtime_error("failed to read mmap header: " + mmap_path.string());
        f.close();

        DtNavMeshPtr mesh(dtAllocNavMesh());
        if (!mesh)
            throw std::bad_alloc();

        if (dtStatusFailed(mesh->init(&params)))
            throw std::runtime_error("dtNavMesh::init failed for map " + std::to_string(map_id));

        int tiles_loaded = 0;
        for (unsigned int x = 0; x < 64; x++) {
            for (unsigned int y = 0; y < 64; y++) {
                char tile_stem[16];
                std::snprintf(tile_stem, sizeof(tile_stem), "%03u%02u%02u", map_id, x, y);
                const fs::path tile_path = mmaps_dir / (std::string(tile_stem) + ".mmtile");
                const std::string tile_path_str = tile_path.string();

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

                DtBuffer data(static_cast<unsigned char*>(dtAlloc(header.size, DT_ALLOC_PERM)));
                if (!data)
                    throw std::bad_alloc();

                tf.read(reinterpret_cast<char*>(data.get()), header.size);
                if (!tf)
                    throw std::runtime_error("mmtile data is truncated: " + tile_path_str);
                tf.close();

                checkTileLayout(tile_path_str, data.get(), header.size);

                // Key off the tile's own dtMeshHeader, not the file name indices: the
                // generator derives dtMeshHeader::x/y from the Detour-axis tile origin,
                // which is transposed relative to the WoW grid indices in the name.
                const dtMeshHeader* meshHeader = reinterpret_cast<const dtMeshHeader*>(data.get());
                tile_uses_liquids_[tileKey(meshHeader->x, meshHeader->y, meshHeader->layer)] =
                    header.usesLiquids;

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
        if (dtStatusFailed(query->init(mesh.get(), 2048))) {
            dtFreeNavMeshQuery(query);
            throw std::runtime_error("dtNavMeshQuery::init failed for map " + std::to_string(map_id));
        }

        nav_mesh_ = mesh.release();
        nav_query_ = query;
        map_id_ = map_id;
        generation_++;
    }

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

    int generation() const { return generation_; }

    // uses_liquids is TrinityCore/AzerothCore's own per-tile MmapTileHeader flag, not
    // part of Detour's dtMeshHeader -- tracked separately here, keyed by tile grid coords.
    bool tileUsesLiquids(int x, int y, int layer) const {
        const auto it = tile_uses_liquids_.find(tileKey(x, y, layer));
        return it != tile_uses_liquids_.end() && it->second;
    }

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

    bool is_loaded() const { return nav_mesh_ != nullptr; }

    std::optional<unsigned int> map_id() const { return map_id_; }

    const std::string& mmaps_path() const { return mmaps_path_; }

    // Returns a NavigationQuery bound to the currently loaded map. Cheap to construct
    // (two pointers + a small config struct copy), so a fresh one is handed out per call
    // rather than cached; the config it carries is a snapshot of query_config() at the
    // time of the call.
    NavigationQuery query() const {
        if (!nav_mesh_ || !nav_query_)
            throw std::runtime_error("no map loaded; call load_map() first");
        return NavigationQuery(nav_mesh_, nav_query_, query_config_, this, generation_);
    }

    const QueryConfig& query_config() const { return query_config_; }
    void set_query_config(QueryConfig config) { query_config_ = config; }

    // Returns the straight path between start and end as a list of (x, y, z) points
    // in WoW world coordinates. Returns an empty list if no path could be found.
    // waypoint_distance: if set, subdivides segments to maintain max distance between waypoints.
    // centered: route through polygon portal-edge midpoints instead of the funnel
    // algorithm's taut string-pull. Trades a longer, less direct path for staying away
    // from walls (the string-pull is the shortest path, so it hugs corners whenever the
    // corridor allows it). max_points is ignored in this mode: the point count is fixed
    // by the polygon corridor (one midpoint per portal crossed, plus start/end).
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

        const bool reaches_end = path_polys[path_count - 1] == end_poly;
        const PathType path_type =
            (reaches_end && !far_from_poly) ? PathType::Normal : PathType::Partial;

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
            std::vector<float> points(static_cast<size_t>(max_points) * 3);
            int point_count = 0;
            status = nav_query_->findStraightPath(start_pt, end_pt, path_polys.data(), path_count,
                                                   points.data(), nullptr, nullptr, &point_count,
                                                   max_points);
            if (dtStatusFailed(status) || point_count == 0)
                return {{}, PathType::NotFound, start};

            result.reserve(static_cast<size_t>(point_count));
            for (int i = 0; i < point_count; i++)
                result.push_back(detourToWow(&points[i * 3]));
        }

        const Point3 actual_end = result.back();

        if (waypoint_distance) {
            std::vector<Point3> subdivided;
            subdivided.push_back(result[0]);
            for (size_t i = 1; i < result.size(); i++) {
                const auto& prev = subdivided.back();
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
    static std::uint64_t tileKey(int x, int y, int layer) {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x) & 0xFFFFFFu) << 40) |
               (static_cast<std::uint64_t>(static_cast<std::uint32_t>(y) & 0xFFFFFFu) << 16) |
               (static_cast<std::uint32_t>(layer) & 0xFFFFu);
    }

    std::string mmaps_path_;
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

NB_MODULE(_wow_navmesh, m) {
    m.doc() = "Python bindings for Detour navmesh pathfinding over TrinityCore/AzerothCore mmaps";

    nb::enum_<PathType>(m, "PathType")
        .value("NOT_FOUND", PathType::NotFound, "No path at all between start and end.")
        .value("NORMAL", PathType::Normal, "Path reaches the requested end point.")
        .value("PARTIAL", PathType::Partial,
               "Path only reaches as close to the end as the navmesh allows.");

    nb::class_<PathResult>(m, "PathResult")
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

    nb::enum_<PathStatus>(m, "PathStatus")
        .value("SUCCESS", PathStatus::Success, "The path reaches the requested end point.")
        .value("PARTIAL", PathStatus::Partial,
               "The path only reaches as close to the end as the navmesh allows.")
        .value("NO_PATH", PathStatus::NoPath, "No route exists between start and end.")
        .value("START_NOT_FOUND", PathStatus::StartNotFound, "No polygon found near the start point.")
        .value("END_NOT_FOUND", PathStatus::EndNotFound, "No polygon found near the end point.")
        .value("INVALID_START", PathStatus::InvalidStart, "The start polygon reference is invalid.")
        .value("INVALID_END", PathStatus::InvalidEnd, "The end polygon reference is invalid.")
        .value("BUFFER_FULL", PathStatus::BufferFull,
               "The result buffer was too small to hold the full result.")
        .value("QUERY_FAILED", PathStatus::QueryFailed, "The underlying Detour query failed.");

    nb::class_<QueryFilter>(m, "QueryFilter")
        .def(nb::init<>())
        .def_prop_rw("include_flags", &QueryFilter::include_flags, &QueryFilter::set_include_flags)
        .def_prop_rw("exclude_flags", &QueryFilter::exclude_flags, &QueryFilter::set_exclude_flags)
        .def("area_cost", &QueryFilter::area_cost, nb::arg("area"))
        .def("set_area_cost", &QueryFilter::set_area_cost, nb::arg("area"), nb::arg("cost"));

    nb::class_<QueryConfig>(m, "NavMeshQueryConfig")
        .def(nb::init<>())
        .def_rw("nearest_poly_extents", &QueryConfig::nearest_poly_extents)
        .def_rw("max_path_polys", &QueryConfig::max_path_polys)
        .def_rw("max_straight_path_points", &QueryConfig::max_straight_path_points);

    nb::class_<NearestPolyResult>(m, "NearestPolyResult")
        .def_ro("found", &NearestPolyResult::found)
        .def_ro("poly_ref", &NearestPolyResult::poly_ref)
        .def_ro("position", &NearestPolyResult::position)
        .def_ro("distance", &NearestPolyResult::distance);

    nb::class_<PolygonPathResult>(m, "PolygonPathResult")
        .def_ro("status", &PolygonPathResult::status)
        .def_ro("start_poly", &PolygonPathResult::start_poly)
        .def_ro("end_poly", &PolygonPathResult::end_poly)
        .def_ro("polygons", &PolygonPathResult::polygons)
        .def_ro("reached_end", &PolygonPathResult::reached_end);

    nb::class_<StraightPathPoint>(m, "StraightPathPoint")
        .def_ro("position", &StraightPathPoint::position)
        .def_ro("flags", &StraightPathPoint::flags)
        .def_ro("poly_ref", &StraightPathPoint::poly_ref);

    nb::class_<StraightPathResult>(m, "StraightPathResult")
        .def_ro("status", &StraightPathResult::status)
        .def_ro("points", &StraightPathResult::points);

    nb::class_<MoveAlongSurfaceResult>(m, "MoveAlongSurfaceResult")
        .def_ro("status", &MoveAlongSurfaceResult::status)
        .def_ro("position", &MoveAlongSurfaceResult::position)
        .def_ro("visited", &MoveAlongSurfaceResult::visited);

    nb::class_<RaycastResult>(m, "RaycastResult")
        .def_ro("hit", &RaycastResult::hit)
        .def_ro("position", &RaycastResult::position)
        .def_ro("normal", &RaycastResult::normal)
        .def_ro("t", &RaycastResult::t)
        .def_ro("path", &RaycastResult::path);

    nb::class_<WallDistanceResult>(m, "WallDistanceResult")
        .def_ro("distance", &WallDistanceResult::distance)
        .def_ro("position", &WallDistanceResult::position)
        .def_ro("normal", &WallDistanceResult::normal);

    nb::enum_<PolyType>(m, "PolyType")
        .value("GROUND", PolyType::Ground,
               "A standard convex walkable polygon that is part of the mesh surface.")
        .value("OFFMESH_CONNECTION", PolyType::OffMeshConnection,
               "A 2-vertex off-mesh connection (jump/teleport/etc.).");

    nb::class_<PolyInfo>(m, "PolyInfo")
        .def_ro("ref", &PolyInfo::ref)
        .def_ro("type", &PolyInfo::type)
        .def_ro("flags", &PolyInfo::flags)
        .def_ro("area", &PolyInfo::area)
        .def_ro("center", &PolyInfo::center)
        .def_ro("vertices", &PolyInfo::vertices)
        .def_ro("neighbors", &PolyInfo::neighbors)
        .def_ro("tile_x", &PolyInfo::tile_x)
        .def_ro("tile_y", &PolyInfo::tile_y)
        .def_ro("tile_layer", &PolyInfo::tile_layer);

    nb::class_<TileInfo>(m, "TileInfo")
        .def_ro("x", &TileInfo::x)
        .def_ro("y", &TileInfo::y)
        .def_ro("layer", &TileInfo::layer)
        .def_ro("bounds_min", &TileInfo::bounds_min)
        .def_ro("bounds_max", &TileInfo::bounds_max)
        .def_ro("poly_count", &TileInfo::poly_count)
        .def_ro("uses_liquids", &TileInfo::uses_liquids);

    nb::class_<OffMeshConnection>(m, "OffMeshConnection")
        .def_ro("ref", &OffMeshConnection::ref)
        .def_ro("start", &OffMeshConnection::start)
        .def_ro("end", &OffMeshConnection::end)
        .def_ro("radius", &OffMeshConnection::radius)
        .def_ro("flags", &OffMeshConnection::flags)
        .def_ro("area", &OffMeshConnection::area)
        .def_ro("bidirectional", &OffMeshConnection::bidirectional);

    nb::class_<SteerTarget>(m, "SteerTarget")
        .def_ro("position", &SteerTarget::position)
        .def_ro("poly_ref", &SteerTarget::poly_ref)
        .def_ro("distance", &SteerTarget::distance)
        .def_ro("reached", &SteerTarget::reached)
        .def_ro("off_mesh", &SteerTarget::off_mesh);

    nb::class_<Path>(m, "Path")
        .def("is_valid", &Path::is_valid)
        .def("is_complete", &Path::is_complete)
        .def_prop_ro("status", &Path::status)
        .def_prop_ro("start_position", &Path::start_position)
        .def_prop_ro("requested_end_position", &Path::requested_end_position)
        .def_prop_ro("actual_end_position", &Path::actual_end_position)
        .def_prop_ro("start_poly", &Path::start_poly)
        .def_prop_ro("end_poly", &Path::end_poly)
        .def_prop_ro("polygon_corridor", &Path::polygon_corridor)
        .def_prop_ro("straight_path", &Path::straight_path)
        .def("get_steer_target", &Path::get_steer_target, nb::arg("current"),
             nb::arg("min_target_distance") = 0.5f, nb::arg("max_target_distance") = 6.0f,
             "Next steering target along this path's corridor from current_position. Returns "
             "None if the path has no corridor (e.g. NO_PATH). min_target_distance is the "
             "horizontal radius within which a corner is considered 'reached' and skipped in "
             "favor of the next one; max_target_distance bounds vertical (y) drift the same way.");

    nb::class_<NavigationQuery>(m, "NavigationQuery")
        .def("find_nearest_poly", &NavigationQuery::find_nearest_poly, nb::arg("position"),
             nb::arg("extents") = nb::none(), nb::arg("filter") = nb::none(),
             "Find the polygon nearest to position, searching within extents (defaults to the "
             "query's configured nearest_poly_extents).")
        .def("find_polygon_path", &NavigationQuery::find_polygon_path, nb::arg("start"),
             nb::arg("end"), nb::arg("filter") = nb::none(), nb::arg("max_polys") = nb::none(),
             "Raw Detour polygon corridor between start and end. Does not compute a point path.")
        .def("find_straight_path", &NavigationQuery::find_straight_path, nb::arg("start"),
             nb::arg("end"), nb::arg("polygons"), nb::arg("filter") = nb::none(),
             nb::arg("max_points") = nb::none(),
             "Detour's findStraightPath() over an already-computed polygon corridor.")
        .def("move_along_surface", &NavigationQuery::move_along_surface, nb::arg("current"),
             nb::arg("desired"), nb::arg("start_poly"), nb::arg("filter") = nb::none(),
             nb::arg("max_visited") = nb::none(),
             "Slide from current toward desired across the walkable surface, without leaving "
             "the mesh.")
        .def("raycast", &NavigationQuery::raycast, nb::arg("start"), nb::arg("end"),
             nb::arg("start_poly") = nb::none(), nb::arg("filter") = nb::none(),
             nb::arg("max_path") = nb::none(),
             "Navmesh-surface raycast from start toward end (Detour's raycast, not a "
             "geometric approximation).")
        .def("closest_point_on_poly", &NavigationQuery::closest_point_on_poly, nb::arg("poly_ref"),
             nb::arg("position"),
             "Closest point to position that lies on poly_ref (may be interior to the "
             "polygon).")
        .def("closest_point_on_poly_boundary", &NavigationQuery::closest_point_on_poly_boundary,
             nb::arg("poly_ref"), nb::arg("position"),
             "Closest point to position on poly_ref's boundary; equals closest_point_on_poly "
             "only when position is outside the polygon's xz-bounds.")
        .def("get_poly_height", &NavigationQuery::get_poly_height, nb::arg("poly_ref"),
             nb::arg("position"),
             "Surface height of poly_ref at position, from detail mesh data (most accurate "
             "height query).")
        .def("find_distance_to_wall", &NavigationQuery::find_distance_to_wall, nb::arg("position"),
             nb::arg("start_poly") = nb::none(), nb::arg("max_radius") = 10.0f,
             nb::arg("filter") = nb::none(),
             "Distance from position to the nearest navmesh wall within max_radius.")
        .def("find_path", &NavigationQuery::find_path, nb::arg("start"), nb::arg("end"),
             nb::arg("filter") = nb::none(), nb::arg("max_polys") = nb::none(), nb::keep_alive<0, 1>(),
             "Full pipeline: polygon corridor + straight path, bundled into a Path object "
             "that also supports get_steer_target().")
        .def("get_poly", &NavigationQuery::get_poly, nb::arg("poly_ref"),
             "Raw structural data (type/flags/area/center/vertices/neighbors/tile) for "
             "poly_ref, or None if it's not a currently valid polygon reference.")
        .def("get_poly_type", &NavigationQuery::get_poly_type, nb::arg("poly_ref"))
        .def("get_poly_area", &NavigationQuery::get_poly_area, nb::arg("poly_ref"))
        .def("get_poly_flags", &NavigationQuery::get_poly_flags, nb::arg("poly_ref"))
        .def("get_poly_center", &NavigationQuery::get_poly_center, nb::arg("poly_ref"))
        .def("get_poly_vertices", &NavigationQuery::get_poly_vertices, nb::arg("poly_ref"),
             "Polygon vertices in WoW (x, y, z) order. Empty list if poly_ref is invalid.")
        .def("get_poly_neighbors", &NavigationQuery::get_poly_neighbors, nb::arg("poly_ref"),
             "PolyRefs of polygons reachable across this polygon's edges. Empty list if "
             "poly_ref is invalid.")
        .def("get_tile_info", &NavigationQuery::get_tile_info, nb::arg("poly_ref"),
             "TileInfo for the tile containing poly_ref, or None if poly_ref is invalid.")
        .def("get_tile_polys", &NavigationQuery::get_tile_polys, nb::arg("tile_x"), nb::arg("tile_y"),
             nb::arg("tile_layer") = 0,
             "Every PolyInfo in the given tile. Empty list if no such tile is loaded.")
        .def("get_offmesh_connections", &NavigationQuery::get_offmesh_connections,
             nb::arg("tile_x") = nb::none(), nb::arg("tile_y") = nb::none(), nb::arg("tile_layer") = 0,
             "Off-mesh connections in the given tile, or across every loaded tile if "
             "tile_x/tile_y are omitted.")
        .def("sample_polys", &NavigationQuery::sample_polys, nb::arg("center"), nb::arg("radius"),
             nb::arg("filter") = nb::none(), nb::arg("max_polys") = nb::none(),
             "PolyInfo for every polygon within radius of center (an axis-aligned box "
             "query, not an exact circle). Raises RuntimeError if more than max_polys "
             "polygons are in range.")
        .def_prop_rw("config", &NavigationQuery::config, &NavigationQuery::set_config);

    nb::class_<NavMesh>(m, "NavMesh")
        .def(nb::init<std::string>(), nb::arg("mmaps_path"),
             "Create a NavMesh bound to a directory of .mmap/.mmtile files.")
        .def("load_map", &NavMesh::load_map, nb::arg("map_id"),
             "Load the navmesh for the given map id, replacing any map currently loaded. "
             "Raises RuntimeError if the mmap files are missing or invalid.")
        .def("free_map", &NavMesh::free_map, "Release the currently loaded map, if any.")
        .def("find_path", &NavMesh::find_path, nb::arg("start"), nb::arg("end"),
             nb::arg("max_points") = 256, nb::arg("waypoint_distance") = nb::none(),
             nb::arg("search_extent") = 50.0f, nb::arg("centered") = false,
             "Find a path between two (x, y, z) points in world coordinates. Returns a "
             "PathResult with the straight-path points, a path_type (NORMAL/PARTIAL/NOT_FOUND), "
             "and actual_end (the point the path actually reaches). waypoint_distance "
             "subdivides segments for precise bot navigation. search_extent controls polygon "
             "search radius (default 50). centered=True routes through polygon portal "
             "midpoints instead of the shortest-path funnel, trading a longer/less direct "
             "path for staying away from walls (max_points is ignored in this mode). "
             "Raises RuntimeError if no map is loaded.")
        .def("get_loaded_tiles", &NavMesh::get_loaded_tiles,
             "TileInfo for every currently loaded tile. Raises RuntimeError if no map is "
             "loaded.")
        .def_prop_ro("is_loaded", &NavMesh::is_loaded)
        .def_prop_ro("map_id", &NavMesh::map_id)
        .def_prop_ro("mmaps_path", &NavMesh::mmaps_path)
        .def_prop_ro("query", &NavMesh::query, nb::keep_alive<0, 1>(),
                     "A NavigationQuery bound to the currently loaded map. Raises RuntimeError "
                     "if no map is loaded.")
        .def_prop_rw("query_config", &NavMesh::query_config, &NavMesh::set_query_config,
                     "Default NavMeshQueryConfig (nearest-poly extents, buffer sizes) used by "
                     "NavigationQuery methods that aren't given an explicit override.")
        .def(
            "__enter__", [](NavMesh& self) -> NavMesh& { return self; }, nb::rv_policy::reference)
        .def(
            "__exit__",
            [](NavMesh& self, nb::object exc_type, nb::object exc_value, nb::object traceback) {
                (void)exc_type;
                (void)exc_value;
                (void)traceback;
                self.free_map();
            },
            nb::arg("exc_type").none(), nb::arg("exc_value").none(), nb::arg("traceback").none())
        .def("__repr__", [](const NavMesh& self) {
            return "NavMesh(mmaps_path=" + self.mmaps_path() +
                   ", map_id=" + (self.map_id() ? std::to_string(*self.map_id()) : "None") + ")";
        });
}
