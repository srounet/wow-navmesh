"""Tests for wow_navmesh.NavMesh.

Most tests here need no real mmap data and run in CI as-is. A second tier exercises
find_path() against a real TrinityCore/AzerothCore mmaps directory; point the
WOW_NAVMESH_TEST_MMAPS environment variable at one (containing e.g. 000.mmap +
0002239.mmtile for Eastern Kingdoms) to enable it.
"""

import os

import pytest

import wow_navmesh as wn


def test_repr_and_initial_state(tmp_path):
    nm = wn.NavMesh(str(tmp_path))
    assert nm.is_loaded is False
    assert nm.map_id is None
    assert nm.mmaps_path == str(tmp_path)
    assert "NavMesh(" in repr(nm)
    assert "map_id=None" in repr(nm)


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
    # max_points still surfaces as the "no map loaded" RuntimeError here. The ValueError
    # path (test_find_path_rejects_non_positive_max_points below) needs a loaded map to
    # observe, so it's covered under the real-mmaps tier instead.
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

    assert f.include_flags == 0x0001
    assert f.exclude_flags == 0x0002
    assert f.area_cost(3) == 2.5


def test_path_status_enum_values():
    assert {s.name for s in wn.PathStatus} == {
        "SUCCESS",
        "PARTIAL",
        "NO_PATH",
        "START_NOT_FOUND",
        "END_NOT_FOUND",
        "INVALID_START",
        "INVALID_END",
        "BUFFER_FULL",
        "QUERY_FAILED",
    }


# --- Tests against a real mmaps directory -----------------------------------------

MMAPS_PATH = os.environ.get("WOW_NAVMESH_TEST_MMAPS")

requires_real_mmaps = pytest.mark.skipif(
    not MMAPS_PATH, reason="set WOW_NAVMESH_TEST_MMAPS to a real mmaps directory to run"
)


@requires_real_mmaps
def test_load_real_map():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    assert nm.is_loaded is True
    assert nm.map_id == 0


@requires_real_mmaps
def test_find_path_rejects_non_positive_max_points():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    with pytest.raises(ValueError):
        nm.find_path((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), max_points=0)
    with pytest.raises(ValueError):
        nm.find_path((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), max_points=-1)


@requires_real_mmaps
def test_find_path_returns_polyline_near_endpoints():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)

    start = (4900.0, -4200.0, -500.0)
    end = (5025.0, -3825.0, -500.0)
    result = nm.find_path(start, end, max_points=64)
    path = result.points

    assert len(path) >= 2
    assert result.path_type in (wn.PathType.NORMAL, wn.PathType.PARTIAL)


@requires_real_mmaps
def test_find_path_centered_returns_polyline_near_endpoints():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)

    start = (4900.0, -4200.0, -500.0)
    end = (5025.0, -3825.0, -500.0)
    result = nm.find_path(start, end, centered=True)
    path = result.points

    assert len(path) >= 2
    assert result.path_type in (wn.PathType.NORMAL, wn.PathType.PARTIAL)
    assert result.actual_end == path[-1]
    # The straight path may snap to the walkable surface, so allow the extents
    # (50 units) used internally for the nearest-poly search.
    assert abs(path[0][0] - start[0]) < 50
    assert abs(path[0][1] - start[1]) < 50
    assert abs(path[-1][0] - end[0]) < 50
    assert abs(path[-1][1] - end[1]) < 50


# --- NavigationQuery / Path tests (real mmaps) --------------------------------------

START = (4900.0, -4200.0, -500.0)
END = (5025.0, -3825.0, -500.0)


@requires_real_mmaps
def test_find_nearest_poly():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    result = nm.query.find_nearest_poly(START)
    assert result.found is True
    assert result.poly_ref != 0
    assert result.distance >= 0


@requires_real_mmaps
def test_find_nearest_poly_with_extents():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    tight = nm.query.find_nearest_poly(START, extents=(1.0, 1.0, 1.0))
    loose = nm.query.find_nearest_poly(START, extents=(200.0, 200.0, 200.0))
    assert loose.found is True
    # A 1-unit search box may or may not clip the mesh depending on exact placement,
    # but it must never do worse than the looser search.
    if tight.found:
        assert tight.distance <= loose.distance + 1e-3


@requires_real_mmaps
def test_find_polygon_path_and_straight_path():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    poly_result = nm.query.find_polygon_path(START, END)
    assert poly_result.status in (wn.PathStatus.SUCCESS, wn.PathStatus.PARTIAL)
    assert len(poly_result.polygons) >= 1
    assert poly_result.start_poly != 0
    assert poly_result.end_poly != 0

    straight = nm.query.find_straight_path(START, END, poly_result.polygons)
    assert straight.status in (wn.PathStatus.SUCCESS, wn.PathStatus.PARTIAL)
    assert len(straight.points) >= 2
    assert straight.points[0].flags & 0x01  # DT_STRAIGHTPATH_START


@requires_real_mmaps
def test_find_polygon_path_start_not_found_far_away():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    far_away = (1_000_000.0, 1_000_000.0, 1_000_000.0)
    result = nm.query.find_polygon_path(far_away, END)
    assert result.status == wn.PathStatus.START_NOT_FOUND


@requires_real_mmaps
def test_find_path_returns_path_object():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    path = nm.query.find_path(START, END)
    assert path.is_valid()
    assert path.status in (wn.PathStatus.SUCCESS, wn.PathStatus.PARTIAL)
    assert len(path.polygon_corridor) >= 1
    assert len(path.straight_path) >= 2
    assert path.start_poly != 0
    assert path.end_poly != 0


@requires_real_mmaps
def test_get_steer_target_progresses_toward_end():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    path = nm.query.find_path(START, END)
    assert path.is_valid()

    steer = path.get_steer_target(path.start_position)
    assert steer is not None
    assert steer.poly_ref != 0
    assert steer.distance >= 0


@requires_real_mmaps
def test_get_steer_target_full_loop_reaches_destination():
    # Regression test: get_steer_target() drives a dtPathCorridor internally, which must
    # be advanced via movePosition()/findCorners() as the agent moves rather than
    # re-running findStraightPath() over the original static corridor each call (that
    # silently assumes the agent is still in the first polygon and stalls a few steps in)
    # -- and a corner list pruned down to zero corners means "arrived", not "no target".
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    q = nm.query
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
        if moved.visited:
            current_poly = moved.visited[-1]

    assert reached, "steering loop must converge on the destination within 500 small steps"


@requires_real_mmaps
def test_move_along_surface():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    nearest = nm.query.find_nearest_poly(START)
    assert nearest.found

    result = nm.query.move_along_surface(nearest.position, END, nearest.poly_ref)
    assert result.status == wn.PathStatus.SUCCESS
    assert len(result.visited) >= 1


@requires_real_mmaps
def test_raycast():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    result = nm.query.raycast(START, END)
    assert isinstance(result.hit, bool)
    assert 0.0 <= result.t or result.t == pytest.approx(0.0)


@requires_real_mmaps
def test_closest_point_and_height():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    nearest = nm.query.find_nearest_poly(START)
    assert nearest.found

    closest = nm.query.closest_point_on_poly(nearest.poly_ref, START)
    boundary = nm.query.closest_point_on_poly_boundary(nearest.poly_ref, START)
    height = nm.query.get_poly_height(nearest.poly_ref, closest)
    assert isinstance(closest, tuple) and len(closest) == 3
    assert isinstance(boundary, tuple) and len(boundary) == 3
    assert isinstance(height, float)


@requires_real_mmaps
def test_find_distance_to_wall():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    result = nm.query.find_distance_to_wall(START, max_radius=20.0)
    # distance is -1 only if no polygon at all is found near START; otherwise it's in
    # (0, max_radius], with max_radius itself meaning "no wall that close" (see
    # NavigationQuery::find_distance_to_wall's docstring).
    assert result.distance == -1.0 or 0.0 <= result.distance <= 20.0


@requires_real_mmaps
def test_find_path_start_not_found_far_away():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    far_away = (1_000_000.0, 1_000_000.0, 1_000_000.0)
    path = nm.query.find_path(far_away, END)
    assert path.status == wn.PathStatus.START_NOT_FOUND
    assert path.is_valid() is False


# --- Map lifecycle: reload/free must invalidate outstanding queries/paths -----------
#
# NavigationQuery and Path hold raw dtNavMesh*/dtNavMeshQuery* pointers captured at
# creation time. NavMesh.load_map()/free_map() free and reallocate those underneath any
# previously-issued NavigationQuery/Path, which would otherwise be a use-after-free the
# moment such a stale object is used again. NavMesh tracks a generation counter bumped on
# every load_map()/free_map() call; NavigationQuery/Path capture it at creation and raise
# instead of touching freed memory when it no longer matches.


@requires_real_mmaps
def test_navigation_query_becomes_stale_after_reload():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    q = nm.query

    nm.load_map(0)

    with pytest.raises(RuntimeError):
        q.find_nearest_poly(START)


@requires_real_mmaps
def test_path_becomes_stale_after_reload():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    path = nm.query.find_path(START, END)
    assert path.is_valid()

    nm.load_map(0)

    with pytest.raises(RuntimeError):
        path.get_steer_target(path.start_position)


@requires_real_mmaps
def test_navigation_query_becomes_stale_after_free_map():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    q = nm.query

    nm.free_map()

    with pytest.raises(RuntimeError):
        q.find_nearest_poly(START)


@requires_real_mmaps
def test_query_obtained_after_reload_still_works():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    nm.load_map(0)  # double load onto the same NavMesh instance

    result = nm.query.find_nearest_poly(START)
    assert result.found is True


# --- World Navigation / NavMesh Introspection ---------------------------------------


@requires_real_mmaps
def test_get_poly_returns_structural_data():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    nearest = nm.query.find_nearest_poly(START)
    assert nearest.found

    poly = nm.query.get_poly(nearest.poly_ref)
    assert poly is not None
    assert poly.ref == nearest.poly_ref
    assert poly.type == wn.PolyType.GROUND
    assert len(poly.vertices) >= 3
    assert isinstance(poly.area, int)
    assert isinstance(poly.flags, int)
    assert isinstance(poly.center, tuple) and len(poly.center) == 3


@requires_real_mmaps
def test_get_poly_individual_accessors_match_get_poly():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    nearest = nm.query.find_nearest_poly(START)
    assert nearest.found
    ref = nearest.poly_ref
    q = nm.query

    poly = q.get_poly(ref)
    assert q.get_poly_type(ref) == poly.type
    assert q.get_poly_area(ref) == poly.area
    assert q.get_poly_flags(ref) == poly.flags
    assert q.get_poly_center(ref) == poly.center
    assert q.get_poly_vertices(ref) == poly.vertices
    assert q.get_poly_neighbors(ref) == poly.neighbors


@requires_real_mmaps
def test_get_poly_invalid_ref_returns_none_not_raise():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    q = nm.query

    assert q.get_poly(0) is None
    assert q.get_poly(0xFFFFFFFF) is None
    assert q.get_poly_type(0) is None
    assert q.get_poly_area(0) is None
    assert q.get_poly_flags(0) is None
    assert q.get_poly_center(0) is None
    assert q.get_poly_vertices(0) == []
    assert q.get_poly_neighbors(0) == []
    assert q.get_tile_info(0) is None


@requires_real_mmaps
def test_get_poly_neighbors_are_reciprocal():
    # If B is a neighbor of A, A must be reachable from B too -- the corridor graph is
    # symmetric for internal links.
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    nearest = nm.query.find_nearest_poly(START)
    assert nearest.found

    neighbors = nm.query.get_poly_neighbors(nearest.poly_ref)
    assert len(neighbors) >= 1
    back = nm.query.get_poly_neighbors(neighbors[0])
    assert nearest.poly_ref in back


@requires_real_mmaps
def test_get_loaded_tiles_and_get_tile_info_agree():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)

    tiles = nm.get_loaded_tiles()
    assert len(tiles) >= 1
    for tile in tiles:
        assert tile.poly_count >= 0
        assert isinstance(tile.uses_liquids, bool)

    nearest = nm.query.find_nearest_poly(START)
    assert nearest.found
    tile_info = nm.query.get_tile_info(nearest.poly_ref)
    assert tile_info is not None
    match = next(t for t in tiles if (t.x, t.y, t.layer) == (tile_info.x, tile_info.y, tile_info.layer))
    assert match.poly_count == tile_info.poly_count


def test_get_loaded_tiles_without_map_raises(tmp_path):
    nm = wn.NavMesh(str(tmp_path))
    with pytest.raises(RuntimeError):
        nm.get_loaded_tiles()


@requires_real_mmaps
def test_get_tile_polys_matches_tile_poly_count():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    tiles = nm.get_loaded_tiles()
    assert tiles

    tile = tiles[0]
    polys = nm.query.get_tile_polys(tile.x, tile.y, tile.layer)
    assert len(polys) == tile.poly_count
    for poly in polys:
        assert poly.tile_x == tile.x
        assert poly.tile_y == tile.y


@requires_real_mmaps
def test_get_tile_polys_missing_tile_returns_empty():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    assert nm.query.get_tile_polys(-1, -1, 0) == []


@requires_real_mmaps
def test_get_offmesh_connections_all_and_per_tile_agree():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    all_connections = nm.query.get_offmesh_connections()

    all_refs = {c.ref for c in all_connections}
    for tile in nm.get_loaded_tiles():
        per_tile = nm.query.get_offmesh_connections(tile.x, tile.y, tile.layer)
        for conn in per_tile:
            assert conn.ref in all_refs
        for conn in per_tile:
            assert isinstance(conn.bidirectional, bool)
            assert conn.radius >= 0


@requires_real_mmaps
def test_get_offmesh_connections_half_specified_tile_raises():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    with pytest.raises(ValueError):
        nm.query.get_offmesh_connections(0)


@requires_real_mmaps
def test_sample_polys_raises_instead_of_truncating():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    with pytest.raises(RuntimeError):
        nm.query.sample_polys(START, 300.0, max_polys=1)


@requires_real_mmaps
def test_sample_polys_finds_polys_near_start():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    nearest = nm.query.find_nearest_poly(START)
    assert nearest.found

    polys = nm.query.sample_polys(START, 30.0)
    assert len(polys) >= 1
    assert any(p.ref == nearest.poly_ref for p in polys)


@requires_real_mmaps
def test_sample_polys_rejects_non_positive_radius():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    with pytest.raises(ValueError):
        nm.query.sample_polys(START, 0.0)


@requires_real_mmaps
def test_introspection_becomes_stale_after_reload():
    nm = wn.NavMesh(MMAPS_PATH)
    nm.load_map(0)
    q = nm.query
    nearest = q.find_nearest_poly(START)
    assert nearest.found

    nm.load_map(0)

    with pytest.raises(RuntimeError):
        q.get_poly(nearest.poly_ref)


def _write_fake_mmaps(dir_path, header_extra_bytes):
    """A 000.mmap plus one 0000000.mmtile whose MmapTileHeader has header_extra_bytes of
    trailing fields (0 = the original version-4 shape, 36 = AzerothCore's version 20).

    The Detour payload is deliberately just a dtMeshHeader magic/version stub: enough to
    prove load_map() found the payload at the right offset, not enough to be a real tile.
    """
    import struct

    # dtNavMeshParams: float orig[3], tileWidth, tileHeight, int maxTiles, maxPolys.
    (dir_path / "000.mmap").write_bytes(
        struct.pack("<5f2i", 0.0, 0.0, 0.0, 533.33333, 533.33333, 4096, 1 << 14)
    )

    payload = struct.pack("<II", 0x444E4156, 7) + b"\0" * 92  # 'DNAV', DT_NAVMESH_VERSION
    header = struct.pack("<4IB3x", 0x4D4D4150, 7, 20, len(payload), 1)
    (dir_path / "0000000.mmtile").write_bytes(header + b"\0" * header_extra_bytes + payload)


@pytest.mark.parametrize("header_extra_bytes", [0, 36])
def test_mmtile_payload_offset_is_derived_from_file_size(tmp_path, header_extra_bytes):
    # The mmap generator has grown MmapTileHeader across versions (20 bytes at version 4,
    # 56 at version 20). Both must locate the Detour payload -- reading it at a hardcoded
    # sizeof(MmapTileHeader) starts 36 bytes early on the newer shape and reports the
    # resulting garbage as a bad navmesh magic.
    _write_fake_mmaps(tmp_path, header_extra_bytes)
    nm = wn.NavMesh(str(tmp_path))
    nm.load_map(0)

    tiles = nm.get_loaded_tiles()
    assert len(tiles) == 1
    assert (tiles[0].x, tiles[0].y, tiles[0].layer) == (0, 0, 0)
    # Also pins the uses_liquids lookup to the tile's own dtMeshHeader coords.
    assert tiles[0].uses_liquids is True
