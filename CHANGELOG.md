# Changelog

All notable changes to this project are documented in this file.

## [1.0.0] - 2026-10-24

First public release of wow-navmesh: Python bindings for Detour, the navmesh query
engine from recastnavigation. It reads the `.mmap`/`.mmtile` files that
TrinityCore/AzerothCore generate for World of Warcraft 3.3.5a and runs pathfinding and
navmesh queries over them from Python.

- `NavMesh`: loads a map from an mmaps directory (any `str` or `os.PathLike`, including
  non-ASCII paths) and validates every tile, including the `DT_POLYREF64` tile layout
  these mmaps use. `find_path()` returns a `PathResult` and accepts `waypoint_distance`,
  `search_extent` and `centered`. `max_nodes` sizes the search node pool for long
  routes.
- `NavigationQuery` (`NavMesh.query`): nearest polygon, polygon corridor, straight path,
  move along surface (snapped onto the surface), raycast, closest point, polygon height
  and distance to wall. Every coordinate triple, search extents included, is in WoW
  `(x, y, z)` order.
- `Path`: a corridor and straight path from one query, with `get_steer_target()` for
  step-by-step steering.
- Navmesh introspection: polygons, tiles, off-mesh connections and area sampling.
- `QueryFilter` and `NavMeshQueryConfig` for include/exclude flags, area costs and
  buffer sizes. Every failure has its own `PathStatus`, and a result cut off by a full
  buffer reports `BUFFER_FULL`.
- Prebuilt wheels for 64-bit CPython 3.10–3.14 on Windows, with type stubs and
  Google-style docstrings throughout.
- MIT licensed. The licenses of the bundled third-party software are in
  `THIRD_PARTY_NOTICES.md`.
