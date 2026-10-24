# Copyright (c) 2026 srounet
# SPDX-License-Identifier: MIT
"""Python bindings for Detour navmesh pathfinding over TrinityCore/AzerothCore mmaps.

Start with NavMesh: point it at an mmaps directory, call load_map(), then use
find_path() or the lower-level NavigationQuery returned by NavMesh.query.
"""

from importlib.metadata import PackageNotFoundError, version

from ._wow_navmesh import (
    MoveAlongSurfaceResult,
    NavigationQuery,
    NavMesh,
    NavMeshQueryConfig,
    NearestPolyResult,
    OffMeshConnection,
    Path,
    PathResult,
    PathStatus,
    PathType,
    PolygonPathResult,
    PolyInfo,
    PolyType,
    QueryFilter,
    RaycastResult,
    SteerTarget,
    StraightPathPoint,
    StraightPathResult,
    TileInfo,
    WallDistanceResult,
)

try:
    __version__ = version("wow-navmesh")
except PackageNotFoundError:  # pragma: no cover - package not installed (e.g. running from source)
    __version__ = "0.0.0"

__all__ = [
    "MoveAlongSurfaceResult",
    "NavMesh",
    "NavMeshQueryConfig",
    "NavigationQuery",
    "NearestPolyResult",
    "OffMeshConnection",
    "Path",
    "PathResult",
    "PathStatus",
    "PathType",
    "PolygonPathResult",
    "PolyInfo",
    "PolyType",
    "QueryFilter",
    "RaycastResult",
    "SteerTarget",
    "StraightPathPoint",
    "StraightPathResult",
    "TileInfo",
    "WallDistanceResult",
    "__version__",
]
