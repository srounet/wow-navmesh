# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Python bindings (via nanobind) around Detour's `dtNavMesh`/`dtNavMeshQuery` from
recastnavigation, scoped specifically to reading and pathfinding over the
`.mmap`/`.mmtile` navmesh files produced by TrinityCore/AzerothCore for WoW 3.3.5a. It
only wraps Detour's *query* engine — it does not build navmeshes (that's Recast/the
core's own mmap generator).

Windows x64 only. Non-Windows and 32-bit builds are rejected at CMake configure time
(`CMakeLists.txt`).

## Build system

- `scikit-build-core` + `nanobind`, driven from `pyproject.toml`/`CMakeLists.txt`.
- Detour's sources are vendored as a git submodule at `extern/recastnavigation/`
  (pinned to `v1.6.0`) — must be initialized (`git submodule update --init --recursive`,
  or clone with `--recursive`) before any build works.
- Requires MSVC ("Desktop development with C++" workload). CMake/Ninja come from pip.

Common commands (PowerShell):

```powershell
pip install .                        # build + install the extension
pip install -e . --no-build-isolation  # editable install for iterative dev
pip install pytest
pytest                                # run the test suite
pytest tests/test_navmesh.py::test_repr_and_initial_state  # single test
```

Building against a plain (non-TrinityCore/AzerothCore) Detour navmesh: set
`WOW_NAVMESH_POLYREF64=OFF` before installing (`$env:WOW_NAVMESH_POLYREF64 = "OFF"`).
This is a CMake option that toggles a compile define (`DT_POLYREF64`) — it changes the
binary's expected on-disk tile layout, not just a runtime flag, so it requires a rebuild.

## Architecture

Everything lives in one file: `src/bindings.cpp`. It defines a single C++ `NavMesh`
class exposed to Python as `_wow_navmesh.NavMesh`, then re-exported as
`wow_navmesh.NavMesh` in `src/wow_navmesh/__init__.py`.

Key mechanics inside `NavMesh`:

- **`load_map(map_id)`**: reads `{map_id:03}.mmap` for the `dtNavMeshParams` header,
  then scans a sparse 64x64 grid of `{map_id:03}{x:02}{y:02}.mmtile` files, validating
  each one's own `MmapTileHeader` (magic/version) before handing its payload to
  `dtNavMesh::addTile()`.
- **The `DT_POLYREF64` layout check (`checkTileLayout`)**: TrinityCore/AzerothCore build
  their mmap generator with `DT_POLYREF64`, widening `dtPolyRef` from 32 to 64 bits.
  That changes the size of `dtLink` (which embeds a `dtPolyRef`), and thus the byte
  offset of every tile section after it — but doesn't change the magic/version Detour
  itself checks, so a mismatched build would otherwise silently misread tiles instead of
  failing loudly. `expectedTileSize()` recomputes a tile's expected serialized size from
  its `dtMeshHeader` counts (mirroring `DetourNavMeshBuilder`'s own section-size math)
  under both possible `dtLink` sizes, and `checkTileLayout()` compares that against the
  file's actual size to detect and name a layout mismatch before it corrupts anything.
  If you touch this logic, keep it in sync with upstream `DetourNavMeshBuilder.cpp`.
- **Coordinate swap**: `.mmap`/`.mmtile` files store coordinates as `(y, z, x)`; the
  public API always uses WoW's `(x, y, z)` order. The swap happens only at the
  `find_path()` boundary (both on the way in for start/end points and on the way out for
  result points) — nowhere else in the codebase needs to think about it.
- **RAII ownership**: `DtNavMeshPtr`/`DtBuffer` (custom deleters over `dtFreeNavMesh`/
  `dtFree`) ensure a throw partway through `load_map()` can't leak the in-progress mesh
  or a tile buffer that `addTile()` didn't take ownership of. `load_map()` calls
  `free_map()` first, so a failed load leaves the previous map's state, not a partial one.
- Nearest-poly search extents (50 units/axis) and the internal path-poly buffer (512
  polys) inside `find_path()` are fixed constants, not exposed as parameters.

Python-side surface is thin: `src/wow_navmesh/__init__.py` just re-exports `NavMesh` and
resolves `__version__` via `importlib.metadata`. Type stubs live in
`src/wow_navmesh/__init__.pyi` (kept in sync manually with the nanobind bindings) and the
package ships `py.typed`.

## Tests

`tests/test_navmesh.py` has two tiers:

- No-mmap-data tests (construction, error paths, context manager) run in CI as-is.
- A second tier exercises real `find_path()` behavior against an actual
  TrinityCore/AzerothCore mmaps directory, gated behind the `WOW_NAVMESH_TEST_MMAPS`
  env var (skipped otherwise). Point it at a directory containing e.g. `000.mmap` +
  `0002239.mmtile` for Eastern Kingdoms to enable that tier locally.

## CI/release

- `.github/workflows/ci.yml`: runs the test suite on `windows-latest` across Python
  3.10–3.13 on every push/PR to `main`.
- `.github/workflows/release.yml`: on `v*` tags, builds `win_amd64` wheels via
  `cibuildwheel` and an sdist, then publishes to PyPI via Trusted Publishing (OIDC).
