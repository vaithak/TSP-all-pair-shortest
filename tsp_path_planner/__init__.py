"""
TSP Path Planner - A complete path planning solution combining TSP and A* algorithms.

This module provides tools for:
- Computing shortest paths between waypoints using A*
- Solving the Traveling Salesman Problem (TSP) for optimal waypoint ordering
- Generating complete trajectories for robot navigation
- Saving/loading planning solutions
- Visualizing paths and trajectories
"""

from .path_planner import TSPPathPlanner
from .visualization import visualize_grid, create_animation

__version__ = "1.0.0"
__all__ = ["TSPPathPlanner", "visualize_grid", "create_animation"]
