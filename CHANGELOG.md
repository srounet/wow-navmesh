# Changelog

All notable changes to this project are documented in this file.

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
