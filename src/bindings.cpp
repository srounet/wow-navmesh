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
#include <vector>

#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

namespace nb = nanobind;
namespace fs = std::filesystem;

namespace {

// TrinityCore/AzerothCore's mmap generator writes tile files with these magic/version
// values, independent of the tile's dtPolyRef width.
constexpr unsigned int kMmapMagic = 0x4d4d4150;  // 'MMAP'
constexpr unsigned int kMmapVersion = 5;

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
                if (header.mmapVersion != kMmapVersion)
                    throw std::runtime_error(
                        "mmtile has unsupported mmap format version " +
                        std::to_string(header.mmapVersion) + " (expected " +
                        std::to_string(kMmapVersion) + "): " + tile_path_str);
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
    }

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
    }

    bool is_loaded() const { return nav_mesh_ != nullptr; }

    std::optional<unsigned int> map_id() const { return map_id_; }

    const std::string& mmaps_path() const { return mmaps_path_; }

    // Returns the straight path between start and end as a list of (x, y, z) points
    // in WoW world coordinates. Returns an empty list if no path could be found.
    // waypoint_distance: if set, subdivides segments to maintain max distance between waypoints.
    std::vector<Point3> find_path(Point3 start, Point3 end, int max_points = 256,
                                   std::optional<float> waypoint_distance = std::nullopt,
                                   float search_extent = 50.0f) {
        if (!nav_mesh_ || !nav_query_)
            throw std::runtime_error("no map loaded; call load_map() first");
        if (max_points <= 0)
            throw std::invalid_argument("max_points must be positive");
        if (waypoint_distance && *waypoint_distance <= 0)
            throw std::invalid_argument("waypoint_distance must be positive");
        if (search_extent <= 0)
            throw std::invalid_argument("search_extent must be positive");

        float s[3] = {std::get<1>(start), std::get<2>(start), std::get<0>(start)};
        float e[3] = {std::get<1>(end), std::get<2>(end), std::get<0>(end)};
        float extents[3] = {search_extent, search_extent, search_extent};
        dtQueryFilter filter;

        dtPolyRef start_poly, end_poly;
        float start_pt[3], end_pt[3];

        dtStatus status = nav_query_->findNearestPoly(s, extents, &filter, &start_poly, start_pt);
        if (dtStatusFailed(status) || !start_poly)
            return {};

        status = nav_query_->findNearestPoly(e, extents, &filter, &end_poly, end_pt);
        if (dtStatusFailed(status) || !end_poly)
            return {};

        std::vector<dtPolyRef> path_polys(512);
        int path_count = 0;
        status = nav_query_->findPath(start_poly, end_poly, start_pt, end_pt, &filter,
                                       path_polys.data(), &path_count,
                                       static_cast<int>(path_polys.size()));
        if (dtStatusFailed(status) || path_count == 0)
            return {};

        std::vector<float> points(static_cast<size_t>(max_points) * 3);
        int point_count = 0;
        status = nav_query_->findStraightPath(start_pt, end_pt, path_polys.data(), path_count,
                                               points.data(), nullptr, nullptr, &point_count,
                                               max_points);
        if (dtStatusFailed(status) || point_count == 0)
            return {};

        std::vector<Point3> result;
        result.reserve(static_cast<size_t>(point_count));
        for (int i = 0; i < point_count; i++)
            result.emplace_back(points[i * 3 + 2], points[i * 3], points[i * 3 + 1]);

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
            return subdivided;
        }

        return result;
    }

private:
    std::string mmaps_path_;
    dtNavMesh* nav_mesh_ = nullptr;
    dtNavMeshQuery* nav_query_ = nullptr;
    std::optional<unsigned int> map_id_;
};

}  // namespace

NB_MODULE(_wow_navmesh, m) {
    m.doc() = "Python bindings for Detour navmesh pathfinding over TrinityCore/AzerothCore mmaps";

    nb::class_<NavMesh>(m, "NavMesh")
        .def(nb::init<std::string>(), nb::arg("mmaps_path"),
             "Create a NavMesh bound to a directory of .mmap/.mmtile files.")
        .def("load_map", &NavMesh::load_map, nb::arg("map_id"),
             "Load the navmesh for the given map id, replacing any map currently loaded. "
             "Raises RuntimeError if the mmap files are missing or invalid.")
        .def("free_map", &NavMesh::free_map, "Release the currently loaded map, if any.")
        .def("find_path", &NavMesh::find_path, nb::arg("start"), nb::arg("end"),
             nb::arg("max_points") = 256, nb::arg("waypoint_distance") = nb::none(),
             nb::arg("search_extent") = 50.0f,
             "Find a path between two (x, y, z) points in world coordinates. Returns a list "
             "of (x, y, z) tuples describing the straight path, or an empty list if no path "
             "was found. waypoint_distance subdivides segments for precise bot navigation. "
             "search_extent controls polygon search radius (default 50). Raises RuntimeError "
             "if no map is loaded.")
        .def_prop_ro("is_loaded", &NavMesh::is_loaded)
        .def_prop_ro("map_id", &NavMesh::map_id)
        .def_prop_ro("mmaps_path", &NavMesh::mmaps_path)
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
