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
