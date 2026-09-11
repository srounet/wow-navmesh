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
