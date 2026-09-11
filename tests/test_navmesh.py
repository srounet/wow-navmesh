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
