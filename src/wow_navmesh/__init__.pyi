from types import TracebackType

Point3 = tuple[float, float, float]

__version__: str

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
    ) -> list[Point3]:
        """Find a path between two (x, y, z) points in world coordinates.

        Args:
            start: Starting point (x, y, z) in world coordinates.
            end: Ending point (x, y, z) in world coordinates.
            max_points: Maximum number of waypoints to return before subdivision (default 256).
            waypoint_distance: If set, subdivides segments to maintain max distance between
                waypoints. Useful for bot navigation to avoid gaps between waypoints.
            search_extent: Search radius for finding the nearest polygon on the navmesh
                (default 50.0). Increase for looser matching, decrease for tighter precision.

        Returns: List of (x, y, z) tuples describing the path, or an empty list if no path
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
