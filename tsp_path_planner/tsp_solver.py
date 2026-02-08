"""
Python wrapper for TSP solver with fallback implementations.
"""

import logging
import numpy as np
from typing import List, Tuple

logger = logging.getLogger(__name__)

# Try to import C++ module
try:
    from . import _tsp_solver_cpp
    HAS_CPP_SOLVER = True
    logger.info("C++ TSP solver loaded successfully")
except ImportError:
    HAS_CPP_SOLVER = False
    logger.warning("C++ TSP solver not available, using Python fallback")


class TSPSolver:
    """
    TSP solver with C++ implementation and Python fallback.
    
    Uses OR-Tools C++ solver if available, otherwise falls back to
    a simple nearest-neighbor heuristic in Python.
    """
    
    def __init__(self):
        self.use_cpp = HAS_CPP_SOLVER
    
    def solve(
        self,
        distance_matrix: List[List[int]],
        depot: int = 0
    ) -> Tuple[List[int], float]:
        """
        Solve TSP problem.
        
        Args:
            distance_matrix: NxN integer distance matrix
            depot: Starting node index
            
        Returns:
            solution_path: List of node indices in visit order
            total_cost: Total path cost
        """
        if self.use_cpp:
            return self._solve_cpp(distance_matrix, depot)
        else:
            return self._solve_python_fallback(distance_matrix, depot)
    
    def _solve_cpp(
        self,
        distance_matrix: List[List[int]],
        depot: int
    ) -> Tuple[List[int], float]:
        """Solve using C++ OR-Tools implementation."""
        solver = _tsp_solver_cpp.TSPSolver()
        path, cost = solver.solve(distance_matrix, depot)
        return path, cost
    
    def _solve_python_fallback(
        self,
        distance_matrix: List[List[int]],
        depot: int
    ) -> Tuple[List[int], float]:
        """
        Fallback: Nearest neighbor heuristic in Python.
        
        This is a simple greedy algorithm that's not optimal but works
        when the C++ solver is unavailable.
        """
        logger.info("Using Python nearest-neighbor heuristic for TSP")
        
        n = len(distance_matrix)
        unvisited = set(range(n))
        path = [depot]
        unvisited.remove(depot)
        current = depot
        total_cost = 0
        
        while unvisited:
            # Find nearest unvisited node
            nearest = min(unvisited, key=lambda x: distance_matrix[current][x])
            total_cost += distance_matrix[current][nearest]
            path.append(nearest)
            unvisited.remove(nearest)
            current = nearest
        
        # Return to depot
        total_cost += distance_matrix[current][depot]
        
        logger.info(f"Python TSP solution: {len(path)} nodes, cost = {total_cost}")
        
        return path, float(total_cost)
