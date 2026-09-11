from importlib.metadata import PackageNotFoundError, version

from ._wow_navmesh import (
    MoveAlongSurfaceResult,
    NavigationQuery,
    NavMesh,
    NavMeshQueryConfig,
    NearestPolyResult,
    Path,
    PathResult,
    PathStatus,
    PathType,
    PolygonPathResult,
    QueryFilter,
    RaycastResult,
    SteerTarget,
    StraightPathPoint,
    StraightPathResult,
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
    "Path",
    "PathResult",
    "PathStatus",
    "PathType",
    "PolygonPathResult",
    "QueryFilter",
    "RaycastResult",
    "SteerTarget",
    "StraightPathPoint",
    "StraightPathResult",
    "WallDistanceResult",
    "__version__",
]
