# Changelog

All notable changes to this project are documented in this file.

## [Unreleased]

- `NavMesh.query -> NavigationQuery`: a thin wrapper over `dtNavMeshQuery` exposing
  `find_nearest_poly`, `find_polygon_path` (raw `dtPolyRef` corridor), `find_straight_path`,
  `move_along_surface`, `raycast`, `closest_point_on_poly(_boundary)`, `get_poly_height`,
  `find_distance_to_wall`, and `find_path` (returns a `Path`).
- `Path`: bundles the polygon corridor, straight path, and status from one `find_path()`
  call, plus `get_steer_target()` for step-by-step steering (backed by Detour's
  `dtPathCorridor` utility, so the corridor is correctly trimmed as the agent advances
  instead of re-deriving that from the original static corridor each call).
- `PathStatus`, `QueryFilter`, `NavMeshQueryConfig`, and typed result objects
  (`NearestPolyResult`, `PolygonPathResult`, `StraightPathResult`/`StraightPathPoint`,
  `MoveAlongSurfaceResult`, `RaycastResult`, `WallDistanceResult`, `SteerTarget`) replace
  the previous "throw everything away but a point list" behavior for this new API.
  `NavMesh.find_path()` is unchanged and still returns `PathResult` as before.
- `dtPolyRef` values are surfaced as plain Python `int`s throughout (64-bit under this
  project's default `DT_POLYREF64` build, 32-bit with `WOW_NAVMESH_POLYREF64=OFF`).
- Map lifecycle safety: any `NavigationQuery`/`Path` obtained before a `load_map()` or
  `free_map()` call now raises `RuntimeError` if used afterwards, instead of touching the
  freed/reallocated navmesh data underneath it.
- `scripts/try_navigation.py`: a runnable smoke-test script demonstrating the full
  nearest-poly / corridor / straight-path / steering-loop / raycast / wall-distance
  pipeline against a real mmaps directory.

## [0.1.0] - Unreleased

Initial public release.

- `NavMesh` binding (nanobind) over Detour's `dtNavMesh`/`dtNavMeshQuery`, for loading
  TrinityCore/AzerothCore `.mmap`/`.mmtile` files and running `find_path()`.
- Detour sources vendored as a git submodule against upstream recastnavigation `v1.6.0`,
  built with `DT_POLYREF64` to match TrinityCore/AzerothCore's mmap layout (configurable
  via the `WOW_NAVMESH_POLYREF64` CMake option / environment variable).
- Tile layout validation: `load_map()` raises a clear `RuntimeError` when a tile's
  on-disk size doesn't match the dtPolyRef width this build targets, instead of silently
  misreading it.
- Type stubs (`__init__.pyi`, `py.typed`) and a pytest suite (real-mmaps tests gated by
  `WOW_NAVMESH_TEST_MMAPS`).
