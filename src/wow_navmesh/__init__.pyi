from enum import Enum
from types import TracebackType

Point3 = tuple[float, float, float]

__version__: str

class PathType(Enum):
    NOT_FOUND = 0
    """No path at all between start and end."""
    NORMAL = 1
    """Path reaches the requested end point."""
    PARTIAL = 2
    """Path only reaches as close to the end as the navmesh allows."""

class PathResult:
    points: list[Point3]
    path_type: PathType
    actual_end: Point3
    """The point the path actually reaches; differs from the requested end when
    path_type is PARTIAL or NOT_FOUND."""

PolyRef = int
"""A dtPolyRef handle. 64-bit under the default DT_POLYREF64 build (matching
TrinityCore/AzerothCore mmaps), 32-bit if built with WOW_NAVMESH_POLYREF64=OFF."""

class PathStatus(Enum):
    SUCCESS = 0
    """The path reaches the requested end point."""
    PARTIAL = 1
    """The path only reaches as close to the end as the navmesh allows."""
    NO_PATH = 2
    """No route exists between start and end."""
    START_NOT_FOUND = 3
    """No polygon found near the start point."""
    END_NOT_FOUND = 4
    """No polygon found near the end point."""
    INVALID_START = 5
    """The start polygon reference is invalid."""
    INVALID_END = 6
    """The end polygon reference is invalid."""
    BUFFER_FULL = 7
    """The result buffer was too small to hold the full result."""
    QUERY_FAILED = 8
    """The underlying Detour query failed."""

class QueryFilter:
    """Python wrapper over dtQueryFilter."""

    def __init__(self) -> None: ...
    include_flags: int
    exclude_flags: int
    def area_cost(self, area: int) -> float: ...
    def set_area_cost(self, area: int, cost: float) -> None: ...

class NavMeshQueryConfig:
    """Defaults used by NavigationQuery methods that aren't given an explicit override."""

    def __init__(self) -> None: ...
    nearest_poly_extents: Point3
    max_path_polys: int
    max_straight_path_points: int

class NearestPolyResult:
    found: bool
    poly_ref: PolyRef
    position: Point3
    distance: float

class PolygonPathResult:
    status: PathStatus
    start_poly: PolyRef
    end_poly: PolyRef
    polygons: list[PolyRef]
    reached_end: bool

class StraightPathPoint:
    position: Point3
    flags: int
    poly_ref: PolyRef

class StraightPathResult:
    status: PathStatus
    points: list[StraightPathPoint]

class MoveAlongSurfaceResult:
    status: PathStatus
    position: Point3
    visited: list[PolyRef]

class RaycastResult:
    hit: bool
    position: Point3
    normal: Point3
    t: float
    path: list[PolyRef]

class WallDistanceResult:
    distance: float
    position: Point3
    normal: Point3

class SteerTarget:
    position: Point3
    poly_ref: PolyRef
    distance: float
    reached: bool
    off_mesh: bool

class Path:
    """A polygon corridor + straight path bundled with enough state to steer along it."""

    def is_valid(self) -> bool: ...
    def is_complete(self) -> bool: ...
    @property
    def status(self) -> PathStatus: ...
    @property
    def start_position(self) -> Point3: ...
    @property
    def requested_end_position(self) -> Point3: ...
    @property
    def actual_end_position(self) -> Point3: ...
    @property
    def start_poly(self) -> PolyRef: ...
    @property
    def end_poly(self) -> PolyRef: ...
    @property
    def polygon_corridor(self) -> list[PolyRef]: ...
    @property
    def straight_path(self) -> list[StraightPathPoint]: ...
    def get_steer_target(
        self,
        current: Point3,
        min_target_distance: float = 0.5,
        max_target_distance: float = 6.0,
    ) -> SteerTarget | None: ...

class NavigationQuery:
    """Thin wrapper over dtNavMeshQuery. Obtained from NavMesh.query."""

    def find_nearest_poly(
        self,
        position: Point3,
        extents: Point3 | None = None,
        filter: QueryFilter | None = None,
    ) -> NearestPolyResult: ...
    def find_polygon_path(
        self,
        start: Point3,
        end: Point3,
        filter: QueryFilter | None = None,
        max_polys: int | None = None,
    ) -> PolygonPathResult: ...
    def find_straight_path(
        self,
        start: Point3,
        end: Point3,
        polygons: list[PolyRef],
        filter: QueryFilter | None = None,
        max_points: int | None = None,
    ) -> StraightPathResult: ...
    def move_along_surface(
        self,
        current: Point3,
        desired: Point3,
        start_poly: PolyRef,
        filter: QueryFilter | None = None,
        max_visited: int | None = None,
    ) -> MoveAlongSurfaceResult: ...
    def raycast(
        self,
        start: Point3,
        end: Point3,
        start_poly: PolyRef | None = None,
        filter: QueryFilter | None = None,
        max_path: int | None = None,
    ) -> RaycastResult: ...
    def closest_point_on_poly(self, poly_ref: PolyRef, position: Point3) -> Point3: ...
    def closest_point_on_poly_boundary(self, poly_ref: PolyRef, position: Point3) -> Point3: ...
    def get_poly_height(self, poly_ref: PolyRef, position: Point3) -> float: ...
    def find_distance_to_wall(
        self,
        position: Point3,
        start_poly: PolyRef | None = None,
        max_radius: float = 10.0,
        filter: QueryFilter | None = None,
    ) -> WallDistanceResult: ...
    def find_path(
        self,
        start: Point3,
        end: Point3,
        filter: QueryFilter | None = None,
        max_polys: int | None = None,
    ) -> Path: ...
    config: NavMeshQueryConfig

class NavMesh:
    """A Detour navmesh bound to a directory of .mmap/.mmtile files.

    mmap tile files store coordinates as (y, z, x) instead of WoW's (x, y, z); NavMesh
    handles that swap internally, so callers only ever see WoW-order (x, y, z) coordinates.
    """

    def __init__(self, mmaps_path: str) -> None: ...
    def load_map(self, map_id: int) -> None:
        """Load the navmesh for the given map id, replacing any map currently loaded.

        Raises RuntimeError if the mmap files are missing, truncated, or built for a
        different dtPolyRef layout than this extension was compiled for.
        """
        ...
    def free_map(self) -> None:
        """Release the currently loaded map, if any."""
        ...
    def find_path(
        self,
        start: Point3,
        end: Point3,
        max_points: int = 256,
        waypoint_distance: float | None = None,
        search_extent: float = 50.0,
        centered: bool = False,
    ) -> PathResult:
        """Find a path between two (x, y, z) points in world coordinates.

        Args:
            start: Starting point (x, y, z) in world coordinates.
            end: Ending point (x, y, z) in world coordinates.
            max_points: Maximum number of waypoints to return before subdivision (default 256).
                Ignored when centered=True.
            waypoint_distance: If set, subdivides segments to maintain max distance between
                waypoints. Useful for bot navigation to avoid gaps between waypoints.
            search_extent: Search radius for finding the nearest polygon on the navmesh
                (default 50.0). Increase for looser matching, decrease for tighter precision.
            centered: If True, route through polygon portal midpoints instead of the
                shortest-path funnel algorithm. Produces a longer, less direct path that
                stays away from walls, instead of hugging corners like the default
                straight path does.

        Returns: A PathResult. `points` is empty and `path_type` is NOT_FOUND if no path
            was found. Raises RuntimeError if no map is loaded, or ValueError if arguments
            are invalid.
        """
        ...
    def __enter__(self) -> "NavMesh": ...
    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None: ...
    def __repr__(self) -> str: ...
    @property
    def is_loaded(self) -> bool: ...
    @property
    def map_id(self) -> int | None: ...
    @property
    def mmaps_path(self) -> str: ...
    @property
    def query(self) -> NavigationQuery:
        """A NavigationQuery bound to the currently loaded map.

        Raises RuntimeError if no map is loaded.
        """
        ...
    query_config: NavMeshQueryConfig
    """Defaults (nearest-poly extents, buffer sizes) used by NavigationQuery methods
    that aren't given an explicit override."""
