# Copyright (c) 2026 srounet
# SPDX-License-Identifier: MIT
"""Type stubs for wow_navmesh.

Kept in sync by hand with the nanobind bindings in src/bindings.cpp. All coordinates
are WoW world coordinates in (x, y, z) order.
"""

import os
from enum import Enum
from types import TracebackType

Point3 = tuple[float, float, float]
"""A point in WoW world coordinates, (x, y, z)."""

PolyRef = int
"""An opaque dtPolyRef handle; 0 means "no polygon".

64-bit under the default DT_POLYREF64 build (matching TrinityCore/AzerothCore mmaps),
32-bit if built with WOW_NAVMESH_POLYREF64=OFF.
"""

__version__: str

class PathType(Enum):
    """Outcome of NavMesh.find_path()."""

    NOT_FOUND = 0
    """No path at all between start and end."""
    NORMAL = 1
    """Path reaches the requested end point."""
    PARTIAL = 2
    """Path stops short of the requested end: the end is unreachable, or the path outgrew
    max_points, the 512-polygon corridor or the search's node pool."""

class PathResult:
    """Result of NavMesh.find_path().

    Attributes:
        points: Waypoints from start to end. Empty when path_type is NOT_FOUND.
        path_type: Whether the path reaches the requested end.
        actual_end: The last waypoint: the requested end snapped onto the navmesh for a
            NORMAL path, where the path stops for a PARTIAL one, and start for
            NOT_FOUND.
    """

    points: list[Point3]
    path_type: PathType
    actual_end: Point3

class PathStatus(Enum):
    """Outcome of a NavigationQuery or Path operation."""

    SUCCESS = 0
    """The path reaches the requested end point."""
    PARTIAL = 1
    """The path stops short of the end: the end is unreachable, or the search ran out of
    nodes (see NavMesh's max_nodes)."""
    NO_PATH = 2
    """No route exists between start and end."""
    START_NOT_FOUND = 3
    """No polygon found near the start point."""
    END_NOT_FOUND = 4
    """No polygon found near the end point."""
    BUFFER_FULL = 5
    """The result buffer was too small; the result holds as much as fit, from the start."""
    QUERY_FAILED = 6
    """The underlying Detour query failed."""

class QueryFilter:
    """Polygon filter and per-area traversal costs (wraps Detour's dtQueryFilter).

    A polygon is traversable if its flags share at least one bit with include_flags and
    none with exclude_flags.

    Attributes:
        include_flags: Poly flags a polygon must have at least one of. Defaults to
            0xFFFF.
        exclude_flags: Poly flags that make a polygon non-traversable. Defaults to 0.
    """

    include_flags: int
    exclude_flags: int

    def __init__(self) -> None:
        """Creates a filter that accepts every polygon, with all area costs at 1.0."""

    def area_cost(self, area: int) -> float:
        """Returns the traversal cost multiplier for an area.

        Args:
            area: Area id, 0 to 63.

        Returns:
            The cost multiplier.

        Raises:
            ValueError: If area is not in 0 to 63.
        """

    def set_area_cost(self, area: int, cost: float) -> None:
        """Sets the traversal cost multiplier for an area.

        Args:
            area: Area id, 0 to 63.
            cost: Cost multiplier; higher makes the area less attractive to path through.

        Raises:
            ValueError: If area is not in 0 to 63.
        """

class NavMeshQueryConfig:
    """Defaults used by NavigationQuery methods that aren't given an explicit override.

    Attributes:
        nearest_poly_extents: Half-extents (x, y, z) of the nearest-poly search box.
            Defaults to (50.0, 50.0, 50.0).
        max_path_polys: Polygon corridor buffer size. Defaults to 512.
        max_straight_path_points: Straight-path buffer size. Defaults to 256.
    """

    nearest_poly_extents: Point3
    max_path_polys: int
    max_straight_path_points: int

    def __init__(self) -> None:
        """Creates a config holding the default values."""

class NearestPolyResult:
    """Result of NavigationQuery.find_nearest_poly().

    Attributes:
        found: Whether a polygon was found within the search box.
        poly_ref: The nearest polygon, or 0 if not found.
        position: The closest point on that polygon. Only meaningful if found.
        distance: Distance from the query point to position, or -1.0 if not found.
    """

    found: bool
    poly_ref: PolyRef
    position: Point3
    distance: float

class PolygonPathResult:
    """Result of NavigationQuery.find_polygon_path().

    Attributes:
        status: SUCCESS, PARTIAL, START_NOT_FOUND, END_NOT_FOUND, NO_PATH, BUFFER_FULL
            or QUERY_FAILED.
        start_poly: Polygon nearest the start, or 0 if not found.
        end_poly: Polygon nearest the end, or 0 if not found.
        polygons: The polygon corridor from start_poly toward end_poly. Only its first
            max_polys polygons if status is BUFFER_FULL.
        reached_end: Whether the corridor ends in end_poly.
    """

    status: PathStatus
    start_poly: PolyRef
    end_poly: PolyRef
    polygons: list[PolyRef]
    reached_end: bool

class StraightPathPoint:
    """One point of a straight path.

    Attributes:
        position: The point.
        flags: DT_STRAIGHTPATH_* bits: 0x01 start, 0x02 end, 0x04 off-mesh connection.
        poly_ref: The polygon the path enters at this point.
    """

    position: Point3
    flags: int
    poly_ref: PolyRef

class StraightPathResult:
    """Result of NavigationQuery.find_straight_path().

    Attributes:
        status: SUCCESS, PARTIAL, BUFFER_FULL or QUERY_FAILED.
        points: The straight path, from start to end. Only its first max_points points
            if status is BUFFER_FULL.
    """

    status: PathStatus
    points: list[StraightPathPoint]

class MoveAlongSurfaceResult:
    """Result of NavigationQuery.move_along_surface().

    Attributes:
        status: SUCCESS, BUFFER_FULL (more than max_visited polygons were crossed;
            position is still right) or QUERY_FAILED.
        position: Where the move ended, with its height snapped onto the navmesh
            surface.
        visited: Polygons crossed during the move. Unless status is BUFFER_FULL, the
            last one contains position.
    """

    status: PathStatus
    position: Point3
    visited: list[PolyRef]

class RaycastResult:
    """Result of NavigationQuery.raycast().

    The hit fields are only meaningful if status is SUCCESS or BUFFER_FULL.

    Attributes:
        status: SUCCESS, BUFFER_FULL (more than max_path polygons were crossed; the hit
            fields are still right), START_NOT_FOUND or QUERY_FAILED.
        hit: Whether the ray hit a navmesh wall before reaching end.
        position: The hit point, or end if nothing was hit.
        normal: The wall normal at the hit point, (0, 0, 0) if nothing was hit.
        t: Fraction of the way from start to end where the hit occurred. A very large
            value if nothing was hit.
        path: Polygons crossed by the ray. Only the first max_path if status is
            BUFFER_FULL.
    """

    status: PathStatus
    hit: bool
    position: Point3
    normal: Point3
    t: float
    path: list[PolyRef]

class WallDistanceResult:
    """Result of NavigationQuery.find_distance_to_wall().

    Attributes:
        distance: Distance to the nearest wall; equal to max_radius if no wall is that
            close, -1.0 if no polygon was found near the position.
        position: The nearest wall point. Only meaningful if distance < max_radius.
        normal: The wall normal at position. Only meaningful if distance < max_radius.
    """

    distance: float
    position: Point3
    normal: Point3

class SteerTarget:
    """Result of Path.get_steer_target().

    Attributes:
        position: The point to steer toward.
        poly_ref: The polygon at that point.
        distance: Distance from the agent to position.
        reached: Whether the agent has arrived at the end of the path.
        off_mesh: Whether position is the start of an off-mesh connection.
    """

    position: Point3
    poly_ref: PolyRef
    distance: float
    reached: bool
    off_mesh: bool

class PolyType(Enum):
    """Kind of a navmesh polygon."""

    GROUND = 0
    """A standard convex walkable polygon that is part of the mesh surface."""
    OFFMESH_CONNECTION = 1
    """A 2-vertex off-mesh connection (jump/teleport/etc.)."""

class PolyInfo:
    """Raw structural data for one navmesh polygon.

    area, flags and geometry are raw navmesh data. They don't by themselves mean
    "road", "water", "building", etc.; that classification is left to a layer built on
    top of this API.

    Attributes:
        ref: The polygon's reference.
        type: Ground polygon or off-mesh connection.
        flags: Poly flags, as set by the mmap generator.
        area: Area id, as set by the mmap generator.
        center: Vertex centroid (Detour has no dedicated per-poly center function).
        vertices: The polygon's vertices.
        neighbors: Polygons reachable across this polygon's edges. Border edges are
            never included. A ref can repeat: a neighbour touching several edge segments
            of a tile border has one link per segment.
        tile_x: X grid coordinate of the containing tile.
        tile_y: Y grid coordinate of the containing tile.
        tile_layer: Layer of the containing tile.
    """

    ref: PolyRef
    type: PolyType
    flags: int
    area: int
    center: Point3
    vertices: list[Point3]
    neighbors: list[PolyRef]
    tile_x: int
    tile_y: int
    tile_layer: int

class TileInfo:
    """Structural data for one loaded navmesh tile.

    Attributes:
        x: Tile grid x coordinate (Detour's, not the .mmtile file name's).
        y: Tile grid y coordinate (Detour's, not the .mmtile file name's).
        layer: Tile layer.
        bounds_min: Minimum corner of the tile's bounding box.
        bounds_max: Maximum corner of the tile's bounding box.
        poly_count: Number of polygons in the tile.
        uses_liquids: The tile's mmap header flag for liquid handling. A tile-level
            flag, not a per-polygon "is water" classification.
    """

    x: int
    y: int
    layer: int
    bounds_min: Point3
    bounds_max: Point3
    poly_count: int
    uses_liquids: bool

class OffMeshConnection:
    """Raw off-mesh connection data (jump/ferry/teleporter/door/etc.).

    No WoW-specific semantics attached.

    Attributes:
        ref: The connection's polygon reference.
        start: Start point.
        end: End point.
        radius: Endpoint radius.
        flags: Poly flags of the connection.
        area: Area id of the connection.
        bidirectional: Whether the connection can be traversed end to start too.
    """

    ref: PolyRef
    start: Point3
    end: Point3
    radius: float
    flags: int
    area: int
    bidirectional: bool

class Path:
    """A polygon corridor and straight path, plus state to steer along them.

    Obtained from NavigationQuery.find_path(). Becomes stale, and raises RuntimeError
    when used, once its NavMesh loads or frees a map.
    """

    def is_valid(self) -> bool:
        """Returns whether the path is usable, i.e. status is SUCCESS, PARTIAL or BUFFER_FULL."""

    def is_complete(self) -> bool:
        """Returns whether the path reaches the requested end, i.e. status is SUCCESS."""

    @property
    def status(self) -> PathStatus:
        """Outcome of the query that built this path."""

    @property
    def start_position(self) -> Point3:
        """Start position, clamped onto the start polygon if a corridor was found."""

    @property
    def requested_end_position(self) -> Point3:
        """End position as originally requested."""

    @property
    def actual_end_position(self) -> Point3:
        """Where the path actually ends: the requested end snapped onto the navmesh, or the
        closest point the corridor reaches if it stops short."""

    @property
    def start_poly(self) -> PolyRef:
        """Polygon the path starts in, or 0 if none was found."""

    @property
    def end_poly(self) -> PolyRef:
        """Polygon nearest the requested end, or 0 if none was found."""

    @property
    def polygon_corridor(self) -> list[PolyRef]:
        """The polygon corridor as originally found (not trimmed by steering)."""

    @property
    def straight_path(self) -> list[StraightPathPoint]:
        """The straight (funnel) path as originally found."""

    def get_steer_target(
        self,
        current: Point3,
        min_target_distance: float = 0.5,
        max_target_distance: float = 6.0,
    ) -> SteerTarget | None:
        """Returns the next point to steer toward along this path.

        Each call advances the path's internal corridor to current, so call it with the
        agent's actual position as it moves. This only computes where to go; moving the
        agent is up to the caller (e.g. with NavigationQuery.move_along_surface()).

        Args:
            current: The agent's current position.
            min_target_distance: Horizontal radius within which a corner counts as
                reached and is skipped in favor of the next one.
            max_target_distance: Height (z) tolerance for that same check.

        Returns:
            The steering target, or None if the path has no corridor (e.g. NO_PATH).
            The target's reached attribute is True once the agent has arrived.

        Raises:
            ValueError: If either distance is not positive.
            RuntimeError: If the path is stale.
        """

class NavigationQuery:
    """Navmesh queries over the map loaded in a NavMesh (wraps dtNavMeshQuery).

    Obtained from NavMesh.query. Becomes stale, and raises RuntimeError when used, once
    its NavMesh loads or frees a map. Not thread-safe.

    Attributes:
        config: Defaults used by methods that aren't given an explicit override. A copy
            of NavMesh.query_config taken when this query was created.
    """

    config: NavMeshQueryConfig

    def find_nearest_poly(
        self,
        position: Point3,
        extents: Point3 | None = None,
        filter: QueryFilter | None = None,
    ) -> NearestPolyResult:
        """Finds the polygon nearest to a position.

        Args:
            position: The point to search around.
            extents: Half-extents (x, y, z) of the search box. Defaults to
                config.nearest_poly_extents.
            filter: Polygons to consider. Defaults to all.

        Returns:
            The nearest polygon; found is False if none is within the search box.
        """

    def find_polygon_path(
        self,
        start: Point3,
        end: Point3,
        filter: QueryFilter | None = None,
        max_polys: int | None = None,
    ) -> PolygonPathResult:
        """Finds the raw polygon corridor between two points, without a point path.

        Args:
            start: Start point.
            end: End point.
            filter: Polygons the corridor may cross. Defaults to all.
            max_polys: Maximum corridor length. Defaults to config.max_path_polys.

        Returns:
            The corridor; check status for failures.

        Raises:
            ValueError: If max_polys is not positive.
        """

    def find_straight_path(
        self,
        start: Point3,
        end: Point3,
        polygons: list[PolyRef],
        max_points: int | None = None,
    ) -> StraightPathResult:
        """Runs Detour's funnel algorithm over an already-computed polygon corridor.

        Args:
            start: Start point, inside polygons[0].
            end: End point, inside polygons[-1].
            polygons: The corridor, e.g. from find_polygon_path().
            max_points: Maximum number of points. Defaults to
                config.max_straight_path_points.

        Returns:
            The straight path; check status for failures.

        Raises:
            ValueError: If polygons is empty or max_points is not positive.
        """

    def move_along_surface(
        self,
        current: Point3,
        desired: Point3,
        start_poly: PolyRef,
        filter: QueryFilter | None = None,
        max_visited: int | None = None,
    ) -> MoveAlongSurfaceResult:
        """Slides from current toward desired across the walkable surface.

        The move never leaves the navmesh: it stops or slides along walls in the way.
        The resulting position's height is snapped onto the navmesh surface.

        Args:
            current: Start position.
            desired: Position to move toward.
            start_poly: Polygon containing current.
            filter: Polygons the move may cross. Defaults to all.
            max_visited: Maximum number of visited polygons to record. Defaults to 32.

        Returns:
            Where the move ended and the polygons it crossed.

        Raises:
            ValueError: If start_poly is 0 or max_visited is not positive.
        """

    def raycast(
        self,
        start: Point3,
        end: Point3,
        start_poly: PolyRef | None = None,
        filter: QueryFilter | None = None,
        max_path: int | None = None,
    ) -> RaycastResult:
        """Casts a ray along the navmesh surface from start toward end.

        This is Detour's navmesh raycast, not a geometric one: it reports where the
        surface ends between the two points, ignoring height.

        Args:
            start: Ray origin.
            end: Ray target.
            start_poly: Polygon containing start. Found with find_nearest_poly() if
                omitted.
            filter: Polygons the ray may cross. Defaults to all.
            max_path: Maximum number of crossed polygons to record. Defaults to
                config.max_path_polys.

        Returns:
            The hit, if any; check status for failures (no start polygon found, or an
            invalid start_poly).

        Raises:
            ValueError: If max_path is not positive.
        """

    def closest_point_on_poly(self, poly_ref: PolyRef, position: Point3) -> Point3:
        """Returns the point on a polygon closest to a position.

        Args:
            poly_ref: The polygon.
            position: The point to project.

        Returns:
            The closest point, on the polygon's detail surface.

        Raises:
            RuntimeError: If poly_ref is invalid.
        """

    def closest_point_on_poly_boundary(self, poly_ref: PolyRef, position: Point3) -> Point3:
        """Returns the closest point on a polygon's boundary.

        Args:
            poly_ref: The polygon.
            position: The point to project.

        Returns:
            position itself if it lies inside the polygon's horizontal (x, y) bounds,
            otherwise the closest point on the polygon's boundary.

        Raises:
            RuntimeError: If poly_ref is invalid.
        """

    def get_poly_height(self, poly_ref: PolyRef, position: Point3) -> float:
        """Returns the surface height of a polygon at a position.

        Args:
            poly_ref: The polygon.
            position: The point; only its horizontal coordinates are used.

        Returns:
            The height, from the polygon's detail mesh.

        Raises:
            RuntimeError: If poly_ref is invalid or position lies outside the polygon.
        """

    def find_distance_to_wall(
        self,
        position: Point3,
        start_poly: PolyRef | None = None,
        max_radius: float = 10.0,
        filter: QueryFilter | None = None,
    ) -> WallDistanceResult:
        """Measures the distance from a position to the nearest navmesh wall.

        Args:
            position: The point to measure from.
            start_poly: Polygon containing position. Found with find_nearest_poly() if
                omitted.
            max_radius: Search radius.
            filter: Polygons the search may cross. Defaults to all.

        Returns:
            The nearest wall. distance equals max_radius if no wall is that close, and
            is -1.0 if no polygon was found near position.

        Raises:
            ValueError: If max_radius is not positive.
        """

    def find_path(
        self,
        start: Point3,
        end: Point3,
        filter: QueryFilter | None = None,
        max_polys: int | None = None,
    ) -> Path:
        """Finds a path between two points, ready for steering.

        Runs find_polygon_path(), then find_straight_path() over its corridor.

        Args:
            start: Start point.
            end: End point.
            filter: Polygons the path may cross. Defaults to all.
            max_polys: Maximum corridor length. Defaults to config.max_path_polys.

        Returns:
            The path; check status or is_valid() for failures.

        Raises:
            ValueError: If max_polys is not positive.
        """

    def get_poly(self, poly_ref: PolyRef) -> PolyInfo | None:
        """Returns all structural data for a polygon.

        Args:
            poly_ref: The polygon.

        Returns:
            The polygon's data, or None if poly_ref is not a valid polygon.
        """

    def get_tile_info(self, poly_ref: PolyRef) -> TileInfo | None:
        """Returns the tile containing a polygon, or None if poly_ref is invalid.

        Args:
            poly_ref: The polygon.
        """

    def get_tile_polys(self, tile_x: int, tile_y: int, tile_layer: int = 0) -> list[PolyInfo]:
        """Returns every polygon of a tile.

        Args:
            tile_x: Tile grid x coordinate (as in TileInfo.x).
            tile_y: Tile grid y coordinate (as in TileInfo.y).
            tile_layer: Tile layer.

        Returns:
            The tile's polygons, or an empty list if no such tile is loaded.
        """

    def get_offmesh_connections(
        self,
        tile_x: int | None = None,
        tile_y: int | None = None,
        tile_layer: int = 0,
    ) -> list[OffMeshConnection]:
        """Returns the off-mesh connections of one tile, or of every loaded tile.

        Args:
            tile_x: Tile grid x coordinate. Omit together with tile_y for all tiles.
            tile_y: Tile grid y coordinate. Omit together with tile_x for all tiles.
            tile_layer: Tile layer. Ignored when listing all tiles.

        Returns:
            The off-mesh connections.

        Raises:
            ValueError: If only one of tile_x/tile_y is given.
        """

    def sample_polys(
        self,
        center: Point3,
        radius: float,
        filter: QueryFilter | None = None,
        max_polys: int | None = None,
    ) -> list[PolyInfo]:
        """Returns every polygon within radius of center.

        This is an axis-aligned box query, not an exact circle: polygons slightly beyond
        radius at the corners of the box can be included.

        Args:
            center: Center of the search box.
            radius: Half-extent of the search box on every axis.
            filter: Polygons to include. Defaults to all.
            max_polys: Maximum number of polygons. Defaults to config.max_path_polys.

        Returns:
            The polygons in range.

        Raises:
            ValueError: If radius or max_polys is not positive.
            RuntimeError: If more than max_polys polygons are in range.
        """

class NavMesh:
    """A Detour navmesh bound to a directory of .mmap/.mmtile files.

    Holds at most one loaded map at a time. Not thread-safe; use one NavMesh per thread.
    Can be used as a context manager, which frees the loaded map on exit.

    Attributes:
        query_config: Defaults copied into each NavigationQuery obtained from query.
    """

    query_config: NavMeshQueryConfig

    def __init__(self, mmaps_path: str | os.PathLike[str], max_nodes: int = 2048) -> None:
        """Binds to an mmaps directory. Nothing is loaded until load_map().

        Args:
            mmaps_path: Directory containing the .mmap/.mmtile files.
            max_nodes: Node pool size for path searches, up to 65535. A search that runs
                out of nodes returns a PARTIAL path; raise this for long cross-continent
                routes.

        Raises:
            ValueError: If max_nodes is not in 1 to 65535.
        """

    def load_map(self, map_id: int) -> None:
        """Loads the navmesh for a map, replacing any map currently loaded.

        The current map is freed first, so on failure no map is loaded. Any
        NavigationQuery or Path obtained before this call becomes stale.

        Args:
            map_id: WoW map id, e.g. 0 for Eastern Kingdoms.

        Raises:
            RuntimeError: If the mmap files are missing, truncated, or built for a
                different dtPolyRef layout than this extension was compiled for.
        """

    def free_map(self) -> None:
        """Releases the currently loaded map, if any.

        Any NavigationQuery or Path obtained before this call becomes stale.
        """

    def get_loaded_tiles(self) -> list[TileInfo]:
        """Returns every currently loaded tile.

        Raises:
            RuntimeError: If no map is loaded.
        """

    def find_path(
        self,
        start: Point3,
        end: Point3,
        max_points: int = 256,
        waypoint_distance: float | None = None,
        search_extent: float = 50.0,
        centered: bool = False,
    ) -> PathResult:
        """Finds a path between two points.

        Args:
            start: Start point.
            end: End point.
            max_points: Maximum number of waypoints before subdivision. Ignored when
                centered is True.
            waypoint_distance: If set, subdivides segments so consecutive waypoints are
                at most this far apart.
            search_extent: Half-extent of the box searched for the nearest polygon to
                start and end. Increase for looser matching, decrease for tighter.
            centered: If True, route through polygon portal midpoints instead of the
                shortest-path funnel algorithm. Produces a longer, less direct path that
                stays away from walls instead of hugging corners.

        Returns:
            The path. points is empty and path_type is NOT_FOUND if no path was found;
            path_type is PARTIAL if the points stop short of end.

        Raises:
            RuntimeError: If no map is loaded.
            ValueError: If max_points, waypoint_distance or search_extent is not
                positive.
        """

    def __enter__(self) -> "NavMesh":
        """Returns this NavMesh."""

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Frees the loaded map. Does not suppress exceptions."""

    def __repr__(self) -> str: ...
    @property
    def is_loaded(self) -> bool:
        """Whether a map is loaded."""

    @property
    def map_id(self) -> int | None:
        """The loaded map's id, or None if no map is loaded."""

    @property
    def mmaps_path(self) -> str:
        """The mmaps directory this NavMesh reads from."""

    @property
    def query(self) -> NavigationQuery:
        """A new NavigationQuery bound to the currently loaded map.

        Raises:
            RuntimeError: If no map is loaded.
        """
