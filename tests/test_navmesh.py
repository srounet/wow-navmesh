# Copyright (c) 2026 srounet
# SPDX-License-Identifier: MIT
"""Tests for wow_navmesh.

Three tiers:

- Tests that need no navmesh at all.
- Tests against a small synthetic navmesh tile written by _write_grid_mmaps(). Like the
  first tier, they need no game data and run in CI.
- Tests against a real TrinityCore/AzerothCore mmaps directory. Point the
  WOW_NAVMESH_TEST_MMAPS environment variable at one (containing e.g. 000.mmap +
  0002239.mmtile for Eastern Kingdoms) to enable them.
"""

import math
import os
import struct

import pytest

import wow_navmesh as wn


def test_repr_and_initial_state(tmp_path):
    nm = wn.NavMesh(str(tmp_path))
    assert nm.is_loaded is False
    assert nm.map_id is None
    assert nm.mmaps_path == str(tmp_path)
    assert repr(nm) == f"NavMesh(mmaps_path={str(tmp_path)!r}, map_id=None)"


def test_find_path_without_loaded_map_raises(tmp_path):
    nm = wn.NavMesh(str(tmp_path))
    with pytest.raises(RuntimeError):
        nm.find_path((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))


def test_load_map_missing_directory_raises(tmp_path):
    nm = wn.NavMesh(str(tmp_path / "does-not-exist"))
    with pytest.raises(RuntimeError):
        nm.load_map(0)


def test_find_path_checks_loaded_map_before_max_points(tmp_path):
    # find_path() checks "is a map loaded" before validating max_points, so an invalid
    # max_points still surfaces as the "no map loaded" RuntimeError here.
    nm = wn.NavMesh(str(tmp_path))
    with pytest.raises(RuntimeError):
        nm.find_path((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), max_points=0)


def test_free_map_is_idempotent(tmp_path):
    nm = wn.NavMesh(str(tmp_path))
    nm.free_map()
    nm.free_map()
    assert nm.is_loaded is False


def test_context_manager_frees_map_on_exit(tmp_path):
    nm = wn.NavMesh(str(tmp_path))
    with nm as ctx:
        assert ctx is nm
    assert nm.is_loaded is False


def test_query_without_loaded_map_raises(tmp_path):
    nm = wn.NavMesh(str(tmp_path))
    with pytest.raises(RuntimeError):
        nm.query


def test_get_loaded_tiles_without_map_raises(tmp_path):
    nm = wn.NavMesh(str(tmp_path))
    with pytest.raises(RuntimeError):
        nm.get_loaded_tiles()


@pytest.mark.parametrize("max_nodes", [0, -1, 65536])
def test_max_nodes_out_of_range_raises(tmp_path, max_nodes):
    with pytest.raises(ValueError):
        wn.NavMesh(tmp_path, max_nodes=max_nodes)


def test_query_config_defaults_and_round_trip(tmp_path):
    nm = wn.NavMesh(str(tmp_path))
    assert nm.query_config.nearest_poly_extents == (50.0, 50.0, 50.0)
    assert nm.query_config.max_path_polys == 512
    assert nm.query_config.max_straight_path_points == 256

    cfg = wn.NavMeshQueryConfig()
    cfg.nearest_poly_extents = (10.0, 20.0, 30.0)
    cfg.max_path_polys = 1024
    cfg.max_straight_path_points = 128
    nm.query_config = cfg

    assert nm.query_config.nearest_poly_extents == (10.0, 20.0, 30.0)
    assert nm.query_config.max_path_polys == 1024
    assert nm.query_config.max_straight_path_points == 128


def test_query_filter_flags_and_area_cost():
    f = wn.QueryFilter()
    assert f.include_flags == 0xFFFF
    assert f.exclude_flags == 0
    assert f.area_cost(0) == 1.0

    f.include_flags = 0x0001
    f.exclude_flags = 0x0002
    f.set_area_cost(3, 2.5)
    f.set_area_cost(63, 4.0)

    assert f.include_flags == 0x0001
    assert f.exclude_flags == 0x0002
    assert f.area_cost(3) == 2.5
    assert f.area_cost(63) == 4.0


@pytest.mark.parametrize("area", [-1, 64])
def test_query_filter_rejects_out_of_range_area(area):
    # Detour doesn't bounds-check area ids: area 64 used to land in include_flags.
    f = wn.QueryFilter()
    with pytest.raises(ValueError):
        f.set_area_cost(area, 1.0)
    with pytest.raises(ValueError):
        f.area_cost(area)
    assert (f.include_flags, f.exclude_flags) == (0xFFFF, 0)


def test_path_status_enum_values():
    assert {s.name for s in wn.PathStatus} == {
        "SUCCESS",
        "PARTIAL",
        "NO_PATH",
        "START_NOT_FOUND",
        "END_NOT_FOUND",
        "BUFFER_FULL",
        "QUERY_FAILED",
    }


# --- Tests against a synthetic tile -------------------------------------------------
#
# A flat (z = 0) 10x10 grid of 10x10-unit cells with a wall in column 5 (rows 0-7). The
# only way past the wall is the water gap at its top (rows 8-9), so WEST -> EAST bends
# around the wall's two top corners. ISLAND is a lone cell no path can reach.

GRID_WATER = {(5, 8), (5, 9)}
GRID_CELLS = [(i, j) for i in range(10) for j in range(10) if i != 5 or j >= 8] + [(20, 0)]
WEST = (25.0, 25.0, 0.0)
EAST = (75.0, 25.0, 0.0)
ISLAND = (205.0, 5.0, 0.0)
WALL_CORNERS = [(50.0, 80.0, 0.0), (60.0, 80.0, 0.0)]


def _write_grid_mmaps(dir_path, cells, water=(), link_size=16, header_extra_bytes=0):
    """Writes map 0 as one synthetic navmesh tile: a flat grid of square polygons.

    Cell (i, j) spans WoW x in [10i, 10i + 10] and y in [10j, 10j + 10] at z = 0.
    Neighbouring cells share an edge, so paths can cross between them.

    Args:
        dir_path: pathlib.Path of the directory to write 000.mmap and 0000000.mmtile into.
        cells: The (i, j) cells to make walkable; the rest are walls.
        water: Cells flagged as water (flags 0x08, area 8) instead of ground (0x01, 1),
            the values TrinityCore's generator uses.
        link_size: sizeof(dtLink): 16 with DT_POLYREF64, 12 without (see link_size()).
        header_extra_bytes: Trailing MmapTileHeader bytes beyond the 20-byte prefix
            (0 = the original version-4 shape, 36 = AzerothCore's version 20).
    """
    index = {cell: n for n, cell in enumerate(cells)}
    verts = {}  # grid corner (i, j) -> vertex index, in insertion order

    def vert(i, j):
        return verts.setdefault((i, j), len(verts))

    polys = b""
    for i, j in cells:
        # The generator's winding. Edge k runs from corner k to corner k + 1.
        corners = [vert(i, j), vert(i + 1, j), vert(i + 1, j + 1), vert(i, j + 1)]
        neis = [index[c] + 1 if c in index else 0 for c in [(i, j - 1), (i + 1, j), (i, j + 1), (i - 1, j)]]
        flags, area = (0x08, 8) if (i, j) in water else (0x01, 1)
        polys += struct.pack("<I6H6HHBB", 0, *corners, 0, 0, *neis, 0, 0, flags, 4, area)

    n, nv = len(cells), len(verts)
    # Detour stores (wow y, wow z, wow x).
    coords = [(10.0 * j, 0.0, 10.0 * i) for i, j in verts]
    bmin = [min(c[k] for c in coords) for k in range(3)]
    bmax = [max(c[k] for c in coords) for k in range(3)]
    payload = b"".join([
        # dtMeshHeader: magic, version, x, y, layer, userId, polyCount, vertCount,
        # maxLinkCount, detailMeshCount, detailVertCount, detailTriCount, bvNodeCount,
        # offMeshConCount, offMeshBase, walkableHeight/Radius/Climb, bmin, bmax,
        # bvQuantFactor.
        struct.pack("<5iI9i10f", 0x444E4156, 7, 0, 0, 0, 0, n, nv, 4 * n, n, 0, 2 * n, 0, 0, n,
                    2.0, 0.6, 1.0, *bmin, *bmax, 1.0),
        struct.pack(f"<{3 * nv}f", *(x for c in coords for x in c)),
        polys,
        b"\0" * (link_size * 4 * n),  # links: built by addTile()
        # One detail mesh of two triangles per polygon, over the polygon's own corners;
        # the last byte flags which triangle edges lie on the polygon boundary.
        b"".join(struct.pack("<IIBB2x", 0, 2 * k, 0, 2) for k in range(n)),
        bytes([0, 1, 2, 0x05, 0, 2, 3, 0x14]) * n,
    ])
    # dtNavMeshParams: orig, tileWidth, tileHeight, maxTiles, maxPolys.
    (dir_path / "000.mmap").write_bytes(struct.pack("<5f2i", 0.0, 0.0, 0.0, 1000.0, 1000.0, 1, 1024))
    header = struct.pack("<4IB3x", 0x4D4D4150, 7, 20, len(payload), 1)
    (dir_path / "0000000.mmtile").write_bytes(header + b"\0" * header_extra_bytes + payload)


@pytest.fixture(scope="session")
def link_size(tmp_path_factory):
    """sizeof(dtLink) this build expects: 16 with DT_POLYREF64 (the default), else 12."""
    probe = tmp_path_factory.mktemp("probe")
    _write_grid_mmaps(probe, [(0, 0)])
    try:
        wn.NavMesh(probe).load_map(0)
        return 16
    except RuntimeError:
        return 12


@pytest.fixture
def grid(tmp_path, link_size):
    """A NavMesh with the synthetic grid loaded as map 0."""
    _write_grid_mmaps(tmp_path, GRID_CELLS, GRID_WATER, link_size)
    nm = wn.NavMesh(tmp_path)
    nm.load_map(0)
    return nm


def _close(a, b, tol=1e-3):
    return math.dist(a, b) < tol


@pytest.mark.parametrize("header_extra_bytes", [0, 36])
def test_mmtile_payload_offset_is_derived_from_file_size(tmp_path, link_size, header_extra_bytes):
    # The mmap generator has grown MmapTileHeader across versions (20 bytes at version 4,
    # 56 at version 20). Both must locate the Detour payload -- reading it at a hardcoded
    # sizeof(MmapTileHeader) starts 36 bytes early on the newer shape and reports the
    # resulting garbage as a bad navmesh magic.
    _write_grid_mmaps(tmp_path, GRID_CELLS, GRID_WATER, link_size, header_extra_bytes)
    nm = wn.NavMesh(str(tmp_path))
    nm.load_map(0)

    tiles = nm.get_loaded_tiles()
    assert len(tiles) == 1
    assert (tiles[0].x, tiles[0].y, tiles[0].layer) == (0, 0, 0)
    assert tiles[0].poly_count == len(GRID_CELLS)
    # Also pins the uses_liquids lookup to the tile's own dtMeshHeader coords.
    assert tiles[0].uses_liquids is True


def test_non_ascii_and_pathlike_mmaps_path(tmp_path, link_size):
    # The path used to be decoded with the ANSI code page, so this failed with "mmap header
    # not found", and a pathlib.Path was rejected outright.
    mmaps = tmp_path / "café-地図"
    mmaps.mkdir()
    _write_grid_mmaps(mmaps, GRID_CELLS, GRID_WATER, link_size)
    for path in (str(mmaps), mmaps):
        nm = wn.NavMesh(path)
        nm.load_map(0)
        assert nm.mmaps_path == str(mmaps)
        assert repr(nm) == f"NavMesh(mmaps_path={str(mmaps)!r}, map_id=0)"


def test_find_path_bends_around_the_wall(grid):
    result = grid.find_path(WEST, EAST)
    assert result.path_type == wn.PathType.NORMAL
    assert _close(result.points[0], WEST) and _close(result.points[-1], EAST)
    for corner in WALL_CORNERS:
        assert any(_close(p, corner) for p in result.points)
    assert result.actual_end == result.points[-1]


def test_find_path_points_cut_off_are_partial(grid):
    n = len(grid.find_path(WEST, EAST).points)
    # max_points that fits exactly is still NORMAL; one fewer used to be NORMAL too.
    assert grid.find_path(WEST, EAST, max_points=n).path_type == wn.PathType.NORMAL
    result = grid.find_path(WEST, EAST, max_points=n - 1)
    assert result.path_type == wn.PathType.PARTIAL
    assert len(result.points) == n - 1
    assert result.actual_end == result.points[-1]


def test_find_path_waypoint_distance_subdivides_evenly(grid):
    corners = grid.find_path(WEST, EAST).points
    result = grid.find_path(WEST, EAST, waypoint_distance=3.0)
    gaps = [math.dist(a, b) for a, b in zip(result.points, result.points[1:])]
    assert max(gaps) <= 3.0 + 1e-3
    for corner in corners:
        assert any(_close(p, corner) for p in result.points)
    length = sum(math.dist(a, b) for a, b in zip(corners, corners[1:]))
    assert sum(gaps) == pytest.approx(length, abs=1e-2)


def test_find_path_partial_ends_on_the_mesh_in_both_modes(grid):
    straight = grid.find_path(WEST, ISLAND)
    centered = grid.find_path(WEST, ISLAND, centered=True)
    assert straight.path_type == centered.path_type == wn.PathType.PARTIAL
    # Centered mode used to append the unreachable end itself, 100+ units off the mesh.
    assert _close(centered.points[-1], straight.points[-1])
    assert straight.points[-1][0] == pytest.approx(100.0)


def test_find_nearest_poly_extents_are_wow_order(grid):
    q = grid.query
    above = (WEST[0], WEST[1], 5.0)  # 5 units above the ground
    assert q.find_nearest_poly(above, extents=(1.0, 1.0, 10.0)).found is True
    assert q.find_nearest_poly(above, extents=(10.0, 10.0, 1.0)).found is False


def test_find_polygon_path_reports_buffer_full(grid):
    q = grid.query
    full = q.find_polygon_path(WEST, EAST)
    assert full.status == wn.PathStatus.SUCCESS and full.reached_end
    exact = q.find_polygon_path(WEST, EAST, max_polys=len(full.polygons))
    assert exact.status == wn.PathStatus.SUCCESS

    cut = q.find_polygon_path(WEST, EAST, max_polys=3)
    assert cut.status == wn.PathStatus.BUFFER_FULL
    assert cut.polygons == full.polygons[:3]


def test_find_straight_path_reports_buffer_full(grid):
    q = grid.query
    corridor = q.find_polygon_path(WEST, EAST).polygons
    n = len(q.find_straight_path(WEST, EAST, corridor).points)
    exact = q.find_straight_path(WEST, EAST, corridor, max_points=n)
    assert exact.status == wn.PathStatus.SUCCESS
    assert exact.points[-1].flags & 0x02  # DT_STRAIGHTPATH_END

    cut = q.find_straight_path(WEST, EAST, corridor, max_points=n - 1)
    assert cut.status == wn.PathStatus.BUFFER_FULL
    assert [p.position for p in cut.points] == [p.position for p in exact.points[:-1]]


def test_navigation_query_find_path_statuses(grid):
    q = grid.query
    assert q.find_path(WEST, EAST).status == wn.PathStatus.SUCCESS
    assert q.find_path(WEST, ISLAND).status == wn.PathStatus.PARTIAL
    assert q.find_path((-500.0, -500.0, 0.0), EAST).status == wn.PathStatus.START_NOT_FOUND

    cfg = q.config
    cfg.max_straight_path_points = 3
    q.config = cfg
    path = q.find_path(WEST, EAST)
    # Used to be SUCCESS / is_complete() with the path ending at the third point.
    assert path.status == wn.PathStatus.BUFFER_FULL
    assert path.is_valid() and not path.is_complete()
    assert _close(path.actual_end_position, EAST)


def test_query_filter_excludes_water(grid):
    q = grid.query
    no_water = wn.QueryFilter()
    no_water.exclude_flags = 0x08
    assert q.find_path(WEST, EAST).status == wn.PathStatus.SUCCESS
    assert q.find_path(WEST, EAST, filter=no_water).status == wn.PathStatus.PARTIAL


def test_max_nodes_limits_the_search(tmp_path, link_size):
    _write_grid_mmaps(tmp_path, GRID_CELLS, GRID_WATER, link_size)
    small = wn.NavMesh(tmp_path, max_nodes=4)
    small.load_map(0)
    assert small.query.find_polygon_path(WEST, EAST).status == wn.PathStatus.PARTIAL
    assert small.find_path(WEST, EAST).path_type == wn.PathType.PARTIAL


def test_move_along_surface_snaps_height_and_reports_buffer_full(grid):
    q = grid.query
    start = q.find_nearest_poly(WEST)
    target = (WEST[0], 75.0, 50.0)  # five cells north, 50 units up in the air

    moved = q.move_along_surface(start.position, target, start.poly_ref)
    assert moved.status == wn.PathStatus.SUCCESS
    assert _close(moved.position, (WEST[0], 75.0, 0.0))  # z used to stay at 50
    assert len(moved.visited) == 6
    assert moved.visited[-1] == q.find_nearest_poly(moved.position).poly_ref

    exact = q.move_along_surface(start.position, target, start.poly_ref, max_visited=6)
    assert exact.status == wn.PathStatus.SUCCESS
    cut = q.move_along_surface(start.position, target, start.poly_ref, max_visited=5)
    assert cut.status == wn.PathStatus.BUFFER_FULL
    assert cut.visited == moved.visited[:5]
    assert cut.position == moved.position


def test_raycast_statuses(grid):
    q = grid.query
    ray = q.raycast(WEST, EAST)
    assert ray.status == wn.PathStatus.SUCCESS
    assert ray.hit is True
    assert ray.position[0] == pytest.approx(50.0)  # the wall's west face

    assert q.raycast(WEST, (25.0, 75.0, 0.0)).hit is False
    cut = q.raycast(WEST, EAST, max_path=1)
    assert cut.status == wn.PathStatus.BUFFER_FULL
    assert cut.hit is True and len(cut.path) == 1

    # Both used to look like a clear line of sight: hit False, position (0, 0, 0).
    assert q.raycast((-500.0, -500.0, 0.0), EAST).status == wn.PathStatus.START_NOT_FOUND
    assert q.raycast(WEST, EAST, start_poly=12345).status == wn.PathStatus.QUERY_FAILED


def test_steer_target_with_tiny_corridor_does_not_overflow(grid):
    # max_polys=1 sized the steering corridor at one polygon, and moving the agent merges
    # up to 16 into it: a heap overflow that crashed the interpreter.
    path = grid.query.find_path(WEST, EAST, max_polys=1)
    assert path.status == wn.PathStatus.BUFFER_FULL
    assert path.get_steer_target((25.0, 45.0, 0.0)) is not None


def test_steering_loop_reaches_destination(grid):
    # The README's steering loop, moving at most 2 units per tick.
    q = grid.query
    path = q.find_path(WEST, EAST)
    position, poly = path.start_position, path.start_poly
    for _ in range(500):
        steer = path.get_steer_target(position)
        assert steer is not None
        if steer.reached:
            break
        dx, dy = steer.position[0] - position[0], steer.position[1] - position[1]
        scale = min(1.0, 2.0 / max(math.hypot(dx, dy), 1e-6))  # walk at most 2 units per tick
        moved = q.move_along_surface(position, (position[0] + dx * scale, position[1] + dy * scale,
                                                steer.position[2]), poly)
        position, poly = moved.position, moved.visited[-1]
    assert steer.reached
    assert math.dist(position, EAST) < 0.5


def test_get_poly_and_tiles(grid):
    q = grid.query
    nearest = q.find_nearest_poly(WEST)
    poly = q.get_poly(nearest.poly_ref)
    assert poly.type == wn.PolyType.GROUND
    assert (poly.flags, poly.area) == (0x01, 1)
    assert _close(poly.center, (25.0, 25.0, 0.0))
    assert len(poly.vertices) == 4
    assert sorted(len(q.get_poly(n).neighbors) for n in poly.neighbors) == [4, 4, 4, 4]
    assert nearest.poly_ref in q.get_poly(poly.neighbors[0]).neighbors

    water = q.find_nearest_poly((55.0, 85.0, 0.0))
    assert (q.get_poly(water.poly_ref).flags, q.get_poly(water.poly_ref).area) == (0x08, 8)
    assert q.get_poly(0) is None
    assert q.get_poly(0xFFFFFFFF) is None
    assert q.get_tile_info(0) is None
    tile = q.get_tile_info(nearest.poly_ref)
    assert len(q.get_tile_polys(tile.x, tile.y, tile.layer)) == len(GRID_CELLS)
    assert q.get_offmesh_connections() == []
    assert any(p.ref == nearest.poly_ref for p in q.sample_polys(WEST, 6.0))


# --- Tests against a real mmaps directory -----------------------------------------

MMAPS_PATH = os.environ.get("WOW_NAVMESH_TEST_MMAPS")

requires_real_mmaps = pytest.mark.skipif(
    not MMAPS_PATH, reason="set WOW_NAVMESH_TEST_MMAPS to a real mmaps directory to run"
)

START = (4900.0, -4200.0, -500.0)
END = (5025.0, -3825.0, -500.0)


@pytest.fixture(scope="module")
def real():
    """Map 0 of WOW_NAVMESH_TEST_MMAPS, loaded once for the tests that only query it.

    Tests that reload or free the map build their own NavMesh instead.
    """
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    return nm


@requires_real_mmaps
def test_load_real_map(real):
    assert real.is_loaded is True
    assert real.map_id == 0


@requires_real_mmaps
def test_find_path_rejects_non_positive_max_points(real):
    with pytest.raises(ValueError):
        real.find_path((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), max_points=0)
    with pytest.raises(ValueError):
        real.find_path((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), max_points=-1)


@requires_real_mmaps
def test_find_path_returns_polyline_near_endpoints(real):
    result = real.find_path(START, END, max_points=64)
    assert len(result.points) >= 2
    assert result.path_type in (wn.PathType.NORMAL, wn.PathType.PARTIAL)


@requires_real_mmaps
def test_find_path_centered_returns_polyline_near_endpoints(real):
    result = real.find_path(START, END, centered=True)
    path = result.points

    assert len(path) >= 2
    assert result.path_type in (wn.PathType.NORMAL, wn.PathType.PARTIAL)
    assert result.actual_end == path[-1]
    # The straight path may snap to the walkable surface, so allow the extents
    # (50 units) used internally for the nearest-poly search.
    assert abs(path[0][0] - START[0]) < 50
    assert abs(path[0][1] - START[1]) < 50
    assert abs(path[-1][0] - END[0]) < 50
    assert abs(path[-1][1] - END[1]) < 50


@requires_real_mmaps
def test_find_nearest_poly(real):
    result = real.query.find_nearest_poly(START)
    assert result.found is True
    assert result.poly_ref != 0
    assert result.distance >= 0


@requires_real_mmaps
def test_find_nearest_poly_with_extents(real):
    tight = real.query.find_nearest_poly(START, extents=(1.0, 1.0, 1.0))
    loose = real.query.find_nearest_poly(START, extents=(200.0, 200.0, 200.0))
    assert loose.found is True
    # A 1-unit search box may or may not clip the mesh depending on exact placement,
    # but it must never do worse than the looser search.
    if tight.found:
        assert tight.distance <= loose.distance + 1e-3


@requires_real_mmaps
def test_find_polygon_path_and_straight_path(real):
    poly_result = real.query.find_polygon_path(START, END)
    assert poly_result.status in (wn.PathStatus.SUCCESS, wn.PathStatus.PARTIAL)
    assert len(poly_result.polygons) >= 1
    assert poly_result.start_poly != 0
    assert poly_result.end_poly != 0

    straight = real.query.find_straight_path(START, END, poly_result.polygons)
    assert straight.status in (wn.PathStatus.SUCCESS, wn.PathStatus.PARTIAL)
    assert len(straight.points) >= 2
    assert straight.points[0].flags & 0x01  # DT_STRAIGHTPATH_START


@requires_real_mmaps
def test_find_polygon_path_start_not_found_far_away(real):
    far_away = (1_000_000.0, 1_000_000.0, 1_000_000.0)
    result = real.query.find_polygon_path(far_away, END)
    assert result.status == wn.PathStatus.START_NOT_FOUND


@requires_real_mmaps
def test_find_path_returns_path_object(real):
    path = real.query.find_path(START, END)
    assert path.is_valid()
    assert path.status in (wn.PathStatus.SUCCESS, wn.PathStatus.PARTIAL)
    assert len(path.polygon_corridor) >= 1
    assert len(path.straight_path) >= 2
    assert path.start_poly != 0
    assert path.end_poly != 0


@requires_real_mmaps
def test_get_steer_target_progresses_toward_end(real):
    path = real.query.find_path(START, END)
    assert path.is_valid()

    steer = path.get_steer_target(path.start_position)
    assert steer is not None
    assert steer.poly_ref != 0
    assert steer.distance >= 0


@requires_real_mmaps
def test_get_steer_target_full_loop_reaches_destination(real):
    # Regression test: get_steer_target() drives a dtPathCorridor internally, which must
    # be advanced via movePosition()/findCorners() as the agent moves rather than
    # re-running findStraightPath() over the original static corridor each call (that
    # silently assumes the agent is still in the first polygon and stalls a few steps in)
    # -- and a corner list pruned down to zero corners means "arrived", not "no target".
    q = real.query
    path = q.find_path(START, END)
    assert path.is_valid()

    step_size = 5.0
    position = path.start_position
    current_poly = path.start_poly
    reached = False
    for _ in range(500):
        steer = path.get_steer_target(position)
        assert steer is not None, "steering must not give up before reaching the destination"
        if steer.reached:
            reached = True
            break
        dx = steer.position[0] - position[0]
        dy = steer.position[1] - position[1]
        dz = steer.position[2] - position[2]
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5
        scale = min(1.0, step_size / dist) if dist > 1e-6 else 1.0
        desired = (position[0] + dx * scale, position[1] + dy * scale, position[2] + dz * scale)
        moved = q.move_along_surface(position, desired, current_poly)
        assert moved.status == wn.PathStatus.SUCCESS
        position = moved.position
        current_poly = moved.visited[-1]

    assert reached, "steering loop must converge on the destination within 500 small steps"


@requires_real_mmaps
def test_move_along_surface(real):
    nearest = real.query.find_nearest_poly(START)
    assert nearest.found

    result = real.query.move_along_surface(nearest.position, END, nearest.poly_ref)
    assert result.status in (wn.PathStatus.SUCCESS, wn.PathStatus.BUFFER_FULL)
    assert len(result.visited) >= 1


@requires_real_mmaps
def test_raycast(real):
    result = real.query.raycast(START, END)
    assert result.status == wn.PathStatus.SUCCESS
    assert isinstance(result.hit, bool)
    assert result.t >= 0.0


@requires_real_mmaps
def test_closest_point_and_height(real):
    nearest = real.query.find_nearest_poly(START)
    assert nearest.found

    closest = real.query.closest_point_on_poly(nearest.poly_ref, START)
    boundary = real.query.closest_point_on_poly_boundary(nearest.poly_ref, START)
    height = real.query.get_poly_height(nearest.poly_ref, closest)
    assert isinstance(closest, tuple) and len(closest) == 3
    assert isinstance(boundary, tuple) and len(boundary) == 3
    assert isinstance(height, float)


@requires_real_mmaps
def test_find_distance_to_wall(real):
    result = real.query.find_distance_to_wall(START, max_radius=20.0)
    # distance is -1 only if no polygon at all is found near START; otherwise it's in
    # (0, max_radius], with max_radius itself meaning "no wall that close" (see
    # NavigationQuery::find_distance_to_wall's docstring).
    assert result.distance == -1.0 or 0.0 <= result.distance <= 20.0


@requires_real_mmaps
def test_find_path_start_not_found_far_away(real):
    far_away = (1_000_000.0, 1_000_000.0, 1_000_000.0)
    path = real.query.find_path(far_away, END)
    assert path.status == wn.PathStatus.START_NOT_FOUND
    assert path.is_valid() is False


# --- Map lifecycle: reload/free must invalidate outstanding queries/paths -----------
#
# NavigationQuery and Path hold raw dtNavMesh*/dtNavMeshQuery* pointers captured at
# creation time. NavMesh.load_map()/free_map() free and reallocate those underneath any
# previously-issued NavigationQuery/Path, which would otherwise be a use-after-free the
# moment such a stale object is used again. NavMesh tracks a generation counter bumped on
# every load_map()/free_map() call; NavigationQuery/Path capture it at creation and raise
# instead of touching freed memory when it no longer matches. These run on the synthetic
# grid, which loads instantly.


def test_navigation_query_becomes_stale_after_reload(grid):
    q = grid.query
    grid.load_map(0)
    with pytest.raises(RuntimeError):
        q.find_nearest_poly(WEST)


def test_path_becomes_stale_after_reload(grid):
    path = grid.query.find_path(WEST, EAST)
    assert path.is_valid()
    grid.load_map(0)
    with pytest.raises(RuntimeError):
        path.get_steer_target(path.start_position)


def test_navigation_query_becomes_stale_after_free_map(grid):
    q = grid.query
    grid.free_map()
    with pytest.raises(RuntimeError):
        q.find_nearest_poly(WEST)


def test_introspection_becomes_stale_after_reload(grid):
    q = grid.query
    nearest = q.find_nearest_poly(WEST)
    assert nearest.found
    grid.load_map(0)
    with pytest.raises(RuntimeError):
        q.get_poly(nearest.poly_ref)


def test_query_obtained_after_reload_still_works(grid):
    grid.load_map(0)  # double load onto the same NavMesh instance
    assert grid.query.find_nearest_poly(WEST).found is True


# --- World Navigation / NavMesh Introspection (real mmaps) ---------------------------


@requires_real_mmaps
def test_get_poly_returns_structural_data(real):
    nearest = real.query.find_nearest_poly(START)
    assert nearest.found

    poly = real.query.get_poly(nearest.poly_ref)
    assert poly is not None
    assert poly.ref == nearest.poly_ref
    assert poly.type == wn.PolyType.GROUND
    assert len(poly.vertices) >= 3
    assert isinstance(poly.area, int)
    assert isinstance(poly.flags, int)
    assert isinstance(poly.center, tuple) and len(poly.center) == 3


@requires_real_mmaps
def test_get_poly_invalid_ref_returns_none_not_raise(real):
    q = real.query
    assert q.get_poly(0) is None
    assert q.get_poly(0xFFFFFFFF) is None
    assert q.get_tile_info(0) is None


@requires_real_mmaps
def test_get_poly_neighbors_are_reciprocal(real):
    # If B is a neighbor of A, A must be reachable from B too -- the corridor graph is
    # symmetric for internal links.
    nearest = real.query.find_nearest_poly(START)
    assert nearest.found

    neighbors = real.query.get_poly(nearest.poly_ref).neighbors
    assert len(neighbors) >= 1
    back = real.query.get_poly(neighbors[0]).neighbors
    assert nearest.poly_ref in back


@requires_real_mmaps
def test_get_loaded_tiles_and_get_tile_info_agree(real):
    tiles = real.get_loaded_tiles()
    assert len(tiles) >= 1
    for tile in tiles:
        assert tile.poly_count >= 0
        assert isinstance(tile.uses_liquids, bool)

    nearest = real.query.find_nearest_poly(START)
    assert nearest.found
    tile_info = real.query.get_tile_info(nearest.poly_ref)
    assert tile_info is not None
    match = next(t for t in tiles if (t.x, t.y, t.layer) == (tile_info.x, tile_info.y, tile_info.layer))
    assert match.poly_count == tile_info.poly_count


@requires_real_mmaps
def test_get_tile_polys_matches_tile_poly_count(real):
    tiles = real.get_loaded_tiles()
    assert tiles

    tile = tiles[0]
    polys = real.query.get_tile_polys(tile.x, tile.y, tile.layer)
    assert len(polys) == tile.poly_count
    for poly in polys:
        assert poly.tile_x == tile.x
        assert poly.tile_y == tile.y


@requires_real_mmaps
def test_get_tile_polys_missing_tile_returns_empty(real):
    assert real.query.get_tile_polys(-1, -1, 0) == []


@requires_real_mmaps
def test_get_offmesh_connections_all_and_per_tile_agree(real):
    all_connections = real.query.get_offmesh_connections()

    all_refs = {c.ref for c in all_connections}
    for tile in real.get_loaded_tiles():
        per_tile = real.query.get_offmesh_connections(tile.x, tile.y, tile.layer)
        for conn in per_tile:
            assert conn.ref in all_refs
        for conn in per_tile:
            assert isinstance(conn.bidirectional, bool)
            assert conn.radius >= 0


@requires_real_mmaps
def test_get_offmesh_connections_half_specified_tile_raises(real):
    with pytest.raises(ValueError):
        real.query.get_offmesh_connections(0)


@requires_real_mmaps
def test_sample_polys_raises_instead_of_truncating(real):
    with pytest.raises(RuntimeError):
        real.query.sample_polys(START, 300.0, max_polys=1)


@requires_real_mmaps
def test_sample_polys_finds_polys_near_start(real):
    nearest = real.query.find_nearest_poly(START)
    assert nearest.found

    polys = real.query.sample_polys(START, 30.0)
    assert len(polys) >= 1
    assert any(p.ref == nearest.poly_ref for p in polys)


@requires_real_mmaps
def test_sample_polys_rejects_non_positive_radius(real):
    with pytest.raises(ValueError):
        real.query.sample_polys(START, 0.0)
