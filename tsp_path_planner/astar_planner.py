# Copyright (c) 2025 Yixun Hu
"""
A* path planning algorithm for 2D grid-based navigation.
"""
import logging
import numpy as np
import heapq
from typing import Tuple, Optional, List, Set, Dict

from . import astar_utils

logger = logging.getLogger(__name__)

class Vertex:
    """
    A class for vertices in A* search.

    Coordinate convention: position is stored as (col, row) tuple
    - col: horizontal index (corresponds to x in world, map column)
    - row: vertical index (corresponds to y in world, map row)
    - Map access: navigable_map[position[1], position[0]] = navigable_map[row, col] (standard numpy)

    This convention matches xy_to_px output format which returns [col, row].
    """
    # __slots__ reduces memory usage significantly for large number of objects
    __slots__ = ['parent', 'position', 'C', 'H', 'F']

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position  # Tuple (col, row) matching xy_to_px output
        self.C = float('inf')  # Cost-to-come (G value)
        self.H = 0  # Heuristic
        self.F = float('inf')  # Total Cost
 
    def __eq__(self, other):
        if other is None:
            return False
        return self.position == other.position
    
    def __hash__(self):
        return hash(self.position)

    def __lt__(self, other):
        """Required for heapq to compare vertices based on F value"""
        return self.F < other.F


class AStarPlanner:
    """
    A* path planner for 2D grid-based navigation.

    Coordinate Convention:
    ----------------------
    This class uses two coordinate systems:

    1. World coordinates (meters): [x, y]
       - Used for input/output positions
       - x: horizontal axis (left-right)
       - y: vertical axis (forward-backward)

    2. Pixel/grid coordinates: stored as numpy arrays [col, row] or tuples (row, col)
       - Maps are stored as numpy arrays with shape (rows, cols)
       - To access map: navigable_map[row, col] = navigable_map[y_idx, x_idx]
       - Internal representations use (row, col) tuples for consistency
       - Note: _xy_to_px() returns [col, row] as numpy array for compatibility

    Key conversions:
    - _xy_to_px([x, y]) -> [col, row] (numpy array)
    - _px_to_xy([col, row]) -> [x, y] (numpy array)
    - Map access: map[pixel[1], pixel[0]] where pixel = [col, row]
    - Vertex position: (row, col) tuple for A* algorithm
    """

    def __init__(
        self,
        waypoint_threshold: float = 0.3,
        max_speed: float = 0.25,
        max_angular_speed: float = 0.15,
        dilation_radius: float = 0.0,
        dilation_retry_step: float = 0.05,
        max_nearest_navigable_search_radius: int = 50,
    ):
        """
        Initialize A* planner with configurable parameters.

        Args:
            waypoint_threshold: Distance threshold to consider waypoint reached (meters)
            max_speed: Maximum linear speed (m/s)
            max_angular_speed: Maximum angular speed (rad/s)
            dilation_radius: Obstacle dilation radius for safety margin (meters)
            dilation_retry_step: Step size for reducing dilation on retry (meters)
            max_nearest_navigable_search_radius: Max BFS search radius for finding navigable pixels (pixels)
        """
        self.waypoint_threshold = waypoint_threshold
        self.max_speed = max_speed
        self.max_angular_speed = max_angular_speed
        self._dilation_radius: float = max(0.0, dilation_radius)
        self._dilation_retry_step: float = max(0.0, dilation_retry_step)
        self._max_nearest_navigable_search_radius: int = max(1, max_nearest_navigable_search_radius)

        # Log all A* parameters
        logger.info("=" * 60)
        logger.info("AStarPlanner Configuration:")
        logger.info(f"  waypoint_threshold: {self.waypoint_threshold}")
        logger.info(f"  max_speed: {self.max_speed}")
        logger.info(f"  max_angular_speed: {self.max_angular_speed}")
        logger.info(f"  dilation_radius: {self._dilation_radius}")
        logger.info(f"  dilation_retry_step: {self._dilation_retry_step}")
        logger.info(f"  max_nearest_navigable_search_radius: {self._max_nearest_navigable_search_radius}")
        logger.info("=" * 60)

        # Pre-compute neighbor directions to save time in loops
        # (dcol, drow, cost) matching the transposed (col, row) position convention
        self.motions = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),  # Straight
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414) # Diagonal
        ]


    def _plan_single_attempt(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        navigable_map: np.ndarray,
        pixels_per_meter: int,
        verbose: bool = True,
    ) -> Optional[np.ndarray]:
        """
        Attempt to plan a path with the given navigable map.

        Args:
            start: Start position [x, y] in meters
            goal: Goal position [x, y] in meters
            navigable_map: Navigable map (2D boolean array)
            pixels_per_meter: Resolution of the map
            verbose: Whether to log detailed info

        Returns:
            Path in pixel coordinates or None if no path found
        """
        # Set up map context (still needed for _astar_search)
        self.pixels_per_meter = pixels_per_meter
        self._map = navigable_map
        size = navigable_map.shape[0]
        episode_pixel_origin = np.array([size // 2, size // 2])
        self._episode_pixel_origin = episode_pixel_origin

        if verbose:
            logger.debug(f"  Episode pixel origin: {episode_pixel_origin}")

        # Convert world coordinates to pixel coordinates
        start_px = astar_utils.xy_to_px(start.reshape(1, 2), navigable_map, pixels_per_meter, episode_pixel_origin)[0]
        goal_px = astar_utils.xy_to_px(goal.reshape(1, 2), navigable_map, pixels_per_meter, episode_pixel_origin)[0]

        if verbose:
            logger.debug(f"  Pixel coords: start_px={start_px}, goal_px={goal_px}")

        # Clip to bounds
        start_px_orig, goal_px_orig = start_px.copy(), goal_px.copy()
        start_px = np.clip(start_px, [0, 0], [size-1, size-1])
        goal_px = np.clip(goal_px, [0, 0], [size-1, size-1])

        if verbose:
            if not np.array_equal(start_px, start_px_orig):
                logger.warning(f"Start position {start_px_orig} out of bounds, clipped to {start_px}")
            if not np.array_equal(goal_px, goal_px_orig):
                logger.warning(f"Goal position {goal_px_orig} out of bounds, clipped to {goal_px}")

        # Check navigability
        start_navigable = navigable_map[start_px[1], start_px[0]]
        goal_navigable = navigable_map[goal_px[1], goal_px[0]]

        if verbose:
            logger.debug("Checking navigability:")
            logger.debug(f"  Start pixel [row={start_px[1]}, col={start_px[0]}]: navigable={start_navigable}")
            logger.debug(f"  Goal pixel [row={goal_px[1]}, col={goal_px[0]}]: navigable={goal_navigable}")
            logger.debug(f"  Navigable map has {np.sum(navigable_map)} navigable pixels out of {navigable_map.size} total")

        # Find nearest navigable pixels if needed
        if not start_navigable:
            if verbose:
                logger.warning(f"Start [row={start_px[1]}, col={start_px[0]}] is non-navigable, finding nearest navigable pixel...")
            nearest = astar_utils.find_nearest_navigable(tuple(start_px), navigable_map, self._max_nearest_navigable_search_radius)
            if nearest is None:
                if verbose:
                    logger.warning("Could not find navigable start position.")
                return None
            start_px = np.array(nearest)
            if verbose:
                logger.info(f"Using nearest navigable start: {start_px}")

        if not goal_navigable:
            if verbose:
                logger.warning(f"Goal [row={goal_px[1]}, col={goal_px[0]}] is non-navigable, finding nearest navigable pixel...")
            nearest = astar_utils.find_nearest_navigable(tuple(goal_px), navigable_map, self._max_nearest_navigable_search_radius)
            if nearest is None:
                if verbose:
                    logger.warning("Could not find navigable goal position.")
                return None
            goal_px = np.array(nearest)
            if verbose:
                logger.info(f"Using nearest navigable goal: {goal_px}")

        # Run A* search
        path_px = self._astar_search(tuple(start_px), tuple(goal_px), navigable_map)
        return path_px

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        navigable_map: np.ndarray,
        pixels_per_meter: int,
        input_interval: float = 0.5,
    ) -> np.ndarray:
        """Plan a path from start to goal using A* with dilation fallback."""
        logger.info("plan() called:")
        logger.info(f"  Start: {start}")
        logger.info(f"  Goal: {goal}")
        logger.info(f"  Map size: {navigable_map.shape}")
        logger.info(f"  Pixels per meter: {pixels_per_meter}")
        logger.info(f"  Dilation Radius meter: {self._dilation_radius}")

        # Save original navigable map for potential retry with smaller dilation
        original_navigable_map = navigable_map.copy()

        # Try planning with progressively smaller dilation radius
        current_dilation = self._dilation_radius

        while current_dilation >= 0:
            # Apply dilation for this attempt
            dilation_radius_px = int(current_dilation * pixels_per_meter)
            if dilation_radius_px > 0:
                navigable_map = astar_utils.dilate_navigable_map(original_navigable_map, dilation_radius_px)
                logger.info(f"Applied dilation with radius {current_dilation:.3f}m ({dilation_radius_px} pixels)")
            else:
                navigable_map = original_navigable_map.copy()
                logger.info("Using original map (no dilation)")

            # Attempt to plan with this dilation level
            verbose = (current_dilation == self._dilation_radius)  # Verbose only on first attempt
            path_px = self._plan_single_attempt(start, goal, navigable_map, pixels_per_meter, verbose=verbose)

            if path_px is not None and len(path_px) > 0:
                # Path found!
                if current_dilation < self._dilation_radius:
                    logger.info(f"Path found with reduced dilation {current_dilation:.3f}m!")
                logger.info(f"  Path found: {len(path_px)} pixels")

                # Convert to world coordinates
                size = navigable_map.shape[0]
                episode_pixel_origin = np.array([size // 2, size // 2])
                path_world = astar_utils.px_to_xy(np.array(path_px), navigable_map, pixels_per_meter, episode_pixel_origin)
                logger.debug(f"  World path length: {len(path_world)} waypoints")

                # Path smoothing
                simplified_path = astar_utils.simplify_path(path_world, navigable_map, pixels_per_meter)
                logger.info(f"  Simplified: {len(path_world)} → {len(simplified_path)} waypoints")

                # Add interpolation to make waypoints denser
                dense_path = astar_utils.interpolate_path(simplified_path, interval=input_interval)
                logger.info(f"  After interpolation: {len(dense_path)} waypoints")
                logger.info("plan() complete")

                return dense_path

            # No path found with this dilation, try smaller
            if current_dilation == self._dilation_radius:
                logger.info(f"No path found with dilation {current_dilation:.3f}m")

            current_dilation -= self._dilation_retry_step
            if current_dilation >= 0:
                logger.info(f"Retrying with smaller dilation: {current_dilation:.3f}m")

        # All dilation values tried, no path found
        logger.warning("No path found even with dilation=0! Area is truly unreachable.")
        return np.array([start])

    def _astar_search(
        self,
        start_px: Tuple[int, int],
        goal_px: Tuple[int, int],
        navigable_map: np.ndarray,
        debug: bool = True,
    ) -> Optional[List[Tuple[int, int]]]:
        """
        Optimized A* search using heapq and dicts.

        Note: Positions are (col, row) tuples, map uses standard map[row, col] indexing
        """
        rows, cols = navigable_map.shape  # Standard numpy: shape[0]=rows, shape[1]=cols
        
        # Priority Queue: stores vertices to visit
        open_set = []
        
        # Dictionary to store created vertices: position -> Vertex
        # This helps us avoid creating duplicate objects and allows O(1) lookup
        all_nodes: Dict[Tuple[int, int], Vertex] = {}
        
        # Initialize start
        start_node = Vertex(None, start_px)
        start_node.C = 0
        start_node.H = astar_utils.compute_octile_heuristic(start_px, goal_px)
        start_node.F = start_node.C + start_node.H
        
        # Push to heap
        heapq.heappush(open_set, start_node)
        all_nodes[start_px] = start_node
        
        # Closed set (visited positions)
        closed_set: Set[Tuple[int, int]] = set()
        
        goal_node = None
        
        # Debug tracking
        nodes_expanded = 0
        closest_to_goal = start_node
        min_h_to_goal = start_node.H
        
        while open_set:
            # Pop node with lowest F (O(log N))
            current = heapq.heappop(open_set)
            
            # If we already processed this node with a lower cost, skip it
            # (Lazy deletion strategy for heapq)
            if current.position in closed_set:
                continue
            
            nodes_expanded += 1
            
            # Track closest point to goal
            if current.H < min_h_to_goal:
                min_h_to_goal = current.H
                closest_to_goal = current
            
            # Check goal
            if current.position == goal_px:
                goal_node = current
                break
            
            # Add to closed set
            closed_set.add(current.position)
            
            # Explore neighbors
            for dcol, drow, move_cost in self.motions:
                col, row = current.position  # position tuple unpacking: (col, row)
                ncol, nrow = col + dcol, row + drow
                neighbor_pos = (ncol, nrow)

                # Bounds check (standard: 0 <= row < rows, 0 <= col < cols)
                if not (0 <= ncol < cols and 0 <= nrow < rows):
                    continue

                # Check if neighbor cell is navigable (using standard map[row, col] access)
                if not navigable_map[nrow, ncol]:
                    continue

                # Diagonal corner cutting check
                if not astar_utils.is_diagonal_move_valid(navigable_map, col, row, dcol, drow):
                    continue
                
                # Skip if in closed set
                if neighbor_pos in closed_set:
                    continue

                tentative_C = current.C + move_cost
                
                # Retrieve existing neighbor node or create new one
                neighbor = all_nodes.get(neighbor_pos)
                if neighbor is None:
                    neighbor = Vertex(current, neighbor_pos)
                    neighbor.H = astar_utils.compute_octile_heuristic(neighbor_pos, goal_px)
                    all_nodes[neighbor_pos] = neighbor
                elif tentative_C >= neighbor.C:
                    # Not a better path
                    continue
                
                # Found a better path to neighbor
                neighbor.parent = current
                neighbor.C = tentative_C
                neighbor.F = neighbor.C + neighbor.H
                
                # Push to heap (even if it's already there, duplicates are handled by closed_set check)
                heapq.heappush(open_set, neighbor)
                
        # Reconstruct path
        if goal_node:
            path = []
            curr = goal_node
            while curr:
                path.append(curr.position)
                curr = curr.parent
            if debug:
                logger.debug(f"A* SUCCESS: Expanded {nodes_expanded} nodes, path length: {len(path)} pixels")
            return path[::-1] # Reverse

        # Path not found - log debug info
        if debug:
            logger.debug("A* FAILED: No path found!")
            logger.debug(f"  Nodes expanded: {nodes_expanded}")
            logger.debug(f"  Total nodes visited (closed set): {len(closed_set)}")
            logger.debug(f"  Start: [col={start_px[0]}, row={start_px[1]}]")
            logger.debug(f"  Goal:  [col={goal_px[0]}, row={goal_px[1]}]")
            logger.debug(f"  Closest reached: [col={closest_to_goal.position[0]}, row={closest_to_goal.position[1]}]")
            logger.debug(f"  Distance from start to closest: {closest_to_goal.C:.1f} pixels")
            logger.debug(f"  Distance from closest to goal: {min_h_to_goal:.1f} pixels (heuristic)")

            # Calculate actual pixel distance
            dx = goal_px[0] - closest_to_goal.position[0]
            dy = goal_px[1] - closest_to_goal.position[1]
            actual_dist = np.sqrt(dx*dx + dy*dy)
            logger.debug(f"  Actual distance closest->goal: {actual_dist:.1f} pixels")

            # Check if start and goal are in disconnected regions
            if nodes_expanded > 0 and len(closed_set) > 0:
                # The search exhausted all reachable nodes but didn't find goal
                # This means start and goal are in disconnected navigable regions
                logger.debug("  DIAGNOSIS: Start and goal are in DISCONNECTED regions!")
                logger.debug(f"  The navigable area from start contains only {len(closed_set)} pixels")
                logger.debug("  Goal is blocked by obstacles (dilation may have closed a narrow passage)")

        return None
