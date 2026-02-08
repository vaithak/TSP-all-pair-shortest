# Copyright (c) 2025 Yixun Hu
"""
Utility functions for A* path planning.

This module contains helper functions for coordinate conversion, map operations,
path simplification, and other utilities used by the A* planner.
"""
import logging
import math
import numpy as np
from collections import deque
from typing import Tuple, Optional

logger = logging.getLogger(__name__)

try:
    from scipy.ndimage import binary_dilation
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


# ============================================================================
# Coordinate Conversion Functions
# ============================================================================

def xy_to_px(
    points: np.ndarray,
    navigable_map: np.ndarray,
    pixels_per_meter: int,
    episode_pixel_origin: np.ndarray
) -> np.ndarray:
    """
    Convert world coordinates to pixel coordinates.

    Args:
        points: World positions [N, 2] where each row is [x, y] in meters
        navigable_map: The map (used for shape)
        pixels_per_meter: Map resolution
        episode_pixel_origin: Origin in pixel coordinates

    Returns:
        Pixel positions [N, 2] where each row is [col, row]
        Note: To access map, use map[result[i, 1], result[i, 0]]
    """
    # Reverse [x,y] to [y,x], scale, and translate
    px = np.rint(points[:, ::-1] * pixels_per_meter) + episode_pixel_origin
    # Flip row coordinate (map origin is top-left, world origin is center)
    px[:, 0] = navigable_map.shape[0] - px[:, 0]
    return px.astype(int)


def px_to_xy(
    px: np.ndarray,
    navigable_map: np.ndarray,
    pixels_per_meter: int,
    episode_pixel_origin: np.ndarray
) -> np.ndarray:
    """
    Convert pixel coordinates to world coordinates.

    Args:
        px: Pixel positions [N, 2] where each row is [col, row]
        navigable_map: The map (used for shape)
        pixels_per_meter: Map resolution
        episode_pixel_origin: Origin in pixel coordinates

    Returns:
        World positions [N, 2] where each row is [x, y] in meters
    """
    px_copy = px.copy()
    # Flip row coordinate back
    px_copy[:, 0] = navigable_map.shape[0] - px_copy[:, 0]
    # Translate, scale, and reverse [y,x] back to [x,y]
    points = (px_copy - episode_pixel_origin) / pixels_per_meter
    return points[:, ::-1]


# ============================================================================
# Heuristic Functions
# ============================================================================

def compute_octile_heuristic(pos: Tuple[int, int], goal_pos: Tuple[int, int]) -> float:
    """
    Octile Distance Heuristic (optimal for 8-way movement).

    Cost = (sqrt(2) - 1) * min(dx, dy) + max(dx, dy)

    This heuristic is admissible and consistent for grid-based A* with
    diagonal movement where straight moves cost 1 and diagonal moves cost sqrt(2).

    Args:
        pos: Current position (row, col)
        goal_pos: Goal position (row, col)

    Returns:
        Heuristic cost estimate to goal
    """
    dx = abs(pos[0] - goal_pos[0])
    dy = abs(pos[1] - goal_pos[1])
    return (math.sqrt(2) - 1) * min(dx, dy) + max(dx, dy)


# ============================================================================
# Map Operations
# ============================================================================

def dilate_navigable_map(
    navigable_map: np.ndarray,
    dilation_radius_px: int,
) -> np.ndarray:
    """
    Dilate obstacles (non-navigable areas) to create a safety margin.

    Args:
        navigable_map: 2D boolean array where True = navigable
        dilation_radius_px: Dilation radius in pixels

    Returns:
        Dilated navigable map (2D boolean array)
    """
    if dilation_radius_px <= 0:
        return navigable_map

    # Treat everything not navigable as obstacle
    obstacle_map = np.logical_not(navigable_map.astype(bool, copy=False))

    if HAS_SCIPY:
        kernel_size = 2 * dilation_radius_px + 1
        kernel = np.ones((kernel_size, kernel_size), dtype=bool)
        dilated_obstacles = binary_dilation(obstacle_map, structure=kernel, iterations=1)
    else:
        rows, cols = obstacle_map.shape
        dilated_obstacles = obstacle_map.copy()
        obstacle_pixels = np.argwhere(obstacle_map)
        for r, c in obstacle_pixels:
            for dr in range(-dilation_radius_px, dilation_radius_px + 1):
                for dc in range(-dilation_radius_px, dilation_radius_px + 1):
                    if max(abs(dr), abs(dc)) <= dilation_radius_px:
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < rows and 0 <= nc < cols:
                            dilated_obstacles[nr, nc] = True

    # Return navigable map as inverse of dilated obstacles
    return ~dilated_obstacles


def find_nearest_navigable(
    target_px: Tuple[int, int],
    navigable_map: np.ndarray,
    max_search_radius: int = 50,
) -> Optional[Tuple[int, int]]:
    """
    Find the nearest navigable pixel to the target using BFS.

    Args:
        target_px: Target pixel position as (col, row) tuple (from xy_to_px)
        navigable_map: Boolean navigable map accessed as map[row, col]
        max_search_radius: Maximum search distance in pixels

    Returns:
        Nearest navigable pixel as (col, row) tuple, or None if not found
    """
    rows, cols = navigable_map.shape

    # If target is already navigable, return it
    # target_px is (col, row), so access as map[row, col] = map[target_px[1], target_px[0]]
    if navigable_map[target_px[1], target_px[0]]:
        return target_px

    # BFS to find nearest navigable pixel
    queue = deque([target_px])
    visited = {target_px}

    # 8-connected neighbors (dcol, drow) matching (col, row) tuple format
    directions = [
        (-1, 0), (1, 0), (0, -1), (0, 1),  # 4-connected
        (-1, -1), (-1, 1), (1, -1), (1, 1)  # diagonals
    ]

    while queue:
        current = queue.popleft()

        # Check if we've exceeded max search radius
        dist = max(abs(current[0] - target_px[0]), abs(current[1] - target_px[1]))
        if dist > max_search_radius:
            continue

        # Explore neighbors
        for dcol, drow in directions:
            neighbor = (current[0] + dcol, current[1] + drow)

            # Bounds check: neighbor is (col, row), check col < cols and row < rows
            if not (0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows):
                continue

            # Skip if already visited
            if neighbor in visited:
                continue

            visited.add(neighbor)

            # Check if navigable: neighbor is (col, row), access map[row, col]
            if navigable_map[neighbor[1], neighbor[0]]:
                logger.debug(f"  Found nearest navigable pixel: {neighbor} (distance: {dist+1} pixels from {target_px})")
                return neighbor

            queue.append(neighbor)

    logger.debug(f"  No navigable pixel found within {max_search_radius} pixels of {target_px}")
    return None


def is_diagonal_move_valid(
    navigable_map: np.ndarray,
    col: int,
    row: int,
    dcol: int,
    drow: int
) -> bool:
    """
    Check if a diagonal move is valid (doesn't cut through obstacle corners).

    For diagonal moves, both adjacent cardinal directions must be navigable
    to prevent cutting through corners.

    Args:
        navigable_map: Boolean navigable map (accessed as map[row, col] - standard numpy)
        col, row: Current position coordinates (from (col, row) tuple, col = x-axis, row = y-axis)
        dcol, drow: Delta for the move

    Returns:
        True if the diagonal move is valid, False otherwise
    """
    if drow == 0 or dcol == 0:
        # Not a diagonal move, always valid
        return True

    # Check both adjacent cardinal neighbors using standard map[row, col] access
    if not navigable_map[row + drow, col]:
        return False
    if not navigable_map[row, col + dcol]:
        return False
    return True


# ============================================================================
# Path Utilities
# ============================================================================

def is_line_clear(
    start_world: np.ndarray,
    end_world: np.ndarray,
    navigable_map: np.ndarray,
    pixels_per_meter: int,
    episode_pixel_origin: np.ndarray
) -> bool:
    """
    Check if line between two points is collision-free using Bresenham's algorithm.

    Args:
        start_world: Start point in world coordinates
        end_world: End point in world coordinates
        navigable_map: Navigable map for collision checking
        pixels_per_meter: Map resolution
        episode_pixel_origin: Map origin in pixel coordinates

    Returns:
        True if line is clear, False if it intersects an obstacle
    """
    start_px = xy_to_px(start_world.reshape(1, 2), navigable_map, pixels_per_meter, episode_pixel_origin)[0]
    end_px = xy_to_px(end_world.reshape(1, 2), navigable_map, pixels_per_meter, episode_pixel_origin)[0]

    # Bresenham's line algorithm
    # Extract x (horizontal/col) and y (vertical/row) for the algorithm
    x0, y0 = start_px[1], start_px[0]
    x1, y1 = end_px[1], end_px[0]

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1

    # Check navigability along the line using map access pattern: map[x, y] where x=col, y=row
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            if not navigable_map[x, y]:
                return False
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            if not navigable_map[x, y]:
                return False
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    if not navigable_map[x, y]:
        return False
    return True


def simplify_path(
    path: np.ndarray,
    navigable_map: np.ndarray,
    pixels_per_meter: int,
) -> np.ndarray:
    """
    Line-of-sight path simplification.
    Removes unnecessary waypoints where a direct line exists.

    Args:
        path: Path waypoints [N, 2] in world coordinates
        navigable_map: Navigable map for collision checking
        pixels_per_meter: Map resolution

    Returns:
        Simplified path with fewer waypoints
    """
    if len(path) < 3:
        return path

    # Set up map context for coordinate conversions
    size = navigable_map.shape[0]
    episode_pixel_origin = np.array([size // 2, size // 2])

    simplified = [path[0]]
    current_idx = 0

    while current_idx < len(path) - 1:
        # Check from last point backwards to current + 1
        for next_idx in range(len(path) - 1, current_idx, -1):
            if is_line_clear(path[current_idx], path[next_idx],
                             navigable_map, pixels_per_meter, episode_pixel_origin):
                simplified.append(path[next_idx])
                current_idx = next_idx
                break
        else:
            # Should not happen if adjacent points are valid, but safe fallback
            current_idx += 1
            simplified.append(path[current_idx])

    return np.array(simplified)


def interpolate_path(path: np.ndarray, interval: float = 0.5) -> np.ndarray:
    """
    Interpolate path with waypoints spaced at regular intervals.

    Args:
        path: Input path waypoints [N, 2]
        interval: Spacing between waypoints in meters (default 0.5m)

    Returns:
        Interpolated path with denser waypoints
    """
    if len(path) < 2:
        return path

    dense_path = [path[0]]

    for i in range(len(path) - 1):
        start_pt = path[i]
        end_pt = path[i+1]
        dist = np.linalg.norm(end_pt - start_pt)

        if dist > interval:
            num_points = int(np.ceil(dist / interval))
            for j in range(1, num_points + 1):
                t = j / num_points
                interp_pt = start_pt + t * (end_pt - start_pt)
                dense_path.append(interp_pt)
        else:
            dense_path.append(end_pt)

    return np.array(dense_path)
