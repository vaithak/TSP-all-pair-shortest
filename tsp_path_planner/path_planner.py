"""
Main TSP Path Planner class combining A* path planning with TSP optimization.
"""

import logging
import pickle
import numpy as np
from typing import List, Tuple, Dict, Optional
import time

from .astar_planner import AStarPlanner
from .tsp_solver import TSPSolver

logger = logging.getLogger(__name__)


class TSPPathPlanner:
    """
    Complete path planning solution combining A* and TSP.
    
    This class handles:
    1. Computing all-pair shortest paths using A*
    2. Solving TSP to find optimal waypoint ordering
    3. Generating complete robot trajectories
    4. Saving and loading planning solutions
    
    Example:
        >>> planner = TSPPathPlanner(navigable_map, pixels_per_meter=20)
        >>> waypoints = np.array([[0, 0], [2, 1], [-1, 2]])
        >>> solution = planner.plan(waypoints)
        >>> print(solution['ordered_indices'])
    """
    
    def __init__(
        self,
        navigable_map: np.ndarray,
        pixels_per_meter: int = 20,
        astar_config: Optional[Dict] = None
    ):
        """
        Initialize TSP path planner.
        
        Args:
            navigable_map: Boolean 2D array where True = navigable/free cell
            pixels_per_meter: Grid resolution (pixels per meter)
            astar_config: Optional configuration dict for A* planner with keys:
                - waypoint_threshold: Distance to consider waypoint reached (default: 0.3m)
                - max_speed: Maximum linear speed (default: 0.25 m/s)
                - max_angular_speed: Maximum angular speed (default: 0.15 rad/s)
                - dilation_radius: Obstacle dilation for safety (default: 0.0m)
                - dilation_retry_step: Dilation reduction step (default: 0.05m)
        """
        self.navigable_map = navigable_map.astype(bool)
        self.pixels_per_meter = pixels_per_meter
        
        # Initialize A* planner
        astar_params = astar_config or {}
        self.astar_planner = AStarPlanner(
            waypoint_threshold=astar_params.get('waypoint_threshold', 0.3),
            max_speed=astar_params.get('max_speed', 0.25),
            max_angular_speed=astar_params.get('max_angular_speed', 0.15),
            dilation_radius=astar_params.get('dilation_radius', 0.0),
            dilation_retry_step=astar_params.get('dilation_retry_step', 0.05),
        )
        
        # Initialize TSP solver
        self.tsp_solver = TSPSolver()
        
        logger.info(f"TSPPathPlanner initialized with map size {navigable_map.shape}, "
                   f"resolution {pixels_per_meter} pixels/meter")
    
    def compute_all_pair_distances(
        self,
        waypoints: np.ndarray,
        verbose: bool = True
    ) -> np.ndarray:
        """
        Compute shortest path distances between all waypoint pairs using A*.
        
        Args:
            waypoints: Array of shape (N, 2) with [x, y] coordinates in meters
            verbose: Whether to log progress
            
        Returns:
            distance_matrix: (N, N) array where element [i,j] is the path 
                           distance from waypoint i to waypoint j
        """
        n = len(waypoints)
        distance_matrix = np.zeros((n, n))
        
        if verbose:
            logger.info(f"Computing all-pair distances for {n} waypoints...")
        
        total_pairs = (n * (n - 1)) // 2
        computed = 0
        start_time = time.time()
        
        for i in range(n):
            for j in range(i + 1, n):
                # Compute path from i to j
                path = self.astar_planner.plan(
                    start=waypoints[i],
                    goal=waypoints[j],
                    navigable_map=self.navigable_map,
                    pixels_per_meter=self.pixels_per_meter,
                    input_interval=0.5
                )
                
                # Calculate path length
                if len(path) > 1:
                    path_length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
                else:
                    # No path found, use large penalty
                    path_length = 1e6
                    logger.warning(f"No path found between waypoint {i} and {j}")
                
                distance_matrix[i, j] = path_length
                distance_matrix[j, i] = path_length  # Symmetric
                
                computed += 1
                if verbose and computed % 10 == 0:
                    elapsed = time.time() - start_time
                    progress = (computed / total_pairs) * 100
                    logger.info(f"Progress: {computed}/{total_pairs} ({progress:.1f}%) "
                              f"- Elapsed: {elapsed:.1f}s")
        
        if verbose:
            elapsed = time.time() - start_time
            logger.info(f"All-pair distance computation complete in {elapsed:.1f}s")
        
        return distance_matrix
    
    def solve_tsp(
        self,
        distance_matrix: np.ndarray,
        start_index: int = 0
    ) -> Tuple[List[int], float]:
        """
        Solve TSP to find optimal waypoint ordering.
        
        Args:
            distance_matrix: (N, N) array of pairwise distances
            start_index: Index of starting waypoint (default 0)
            
        Returns:
            ordered_indices: List of waypoint indices in optimal visiting order
            total_cost: Total path length of the solution
        """
        logger.info(f"Solving TSP for {len(distance_matrix)} waypoints...")
        
        # Convert to integer distance matrix (TSP solver expects integers)
        # Scale by 10 for precision
        int_distance_matrix = (distance_matrix * 10).astype(int).tolist()
        
        # Solve TSP
        ordered_indices, total_cost = self.tsp_solver.solve(
            distance_matrix=int_distance_matrix,
            depot=start_index
        )
        
        # Convert cost back to meters
        total_cost = total_cost / 10.0
        
        logger.info(f"TSP solution found: cost = {total_cost:.2f}m")
        logger.info(f"Waypoint order: {ordered_indices}")
        
        return ordered_indices, total_cost
    
    def compute_full_path(
        self,
        waypoints: np.ndarray,
        ordered_indices: List[int],
        input_interval: float = 0.5,
        verbose: bool = True
    ) -> Tuple[np.ndarray, List[np.ndarray]]:
        """
        Compute detailed path following TSP ordering.
        
        Args:
            waypoints: Original waypoint coordinates (N, 2)
            ordered_indices: TSP solution ordering
            input_interval: Spacing between trajectory points in meters
            verbose: Whether to log progress
            
        Returns:
            full_path: Dense waypoint array (M, 2) for robot to follow
            segment_paths: List of path segments between consecutive waypoints
        """
        if verbose:
            logger.info(f"Computing full path through {len(ordered_indices)} waypoints...")
        
        full_path = []
        segment_paths = []
        
        for i in range(len(ordered_indices) - 1):
            from_idx = ordered_indices[i]
            to_idx = ordered_indices[i + 1]
            
            # Compute path segment
            segment = self.astar_planner.plan(
                start=waypoints[from_idx],
                goal=waypoints[to_idx],
                navigable_map=self.navigable_map,
                pixels_per_meter=self.pixels_per_meter,
                input_interval=input_interval
            )
            
            segment_paths.append(segment)
            
            # Add to full path (avoid duplicating waypoints)
            if i == 0:
                full_path.extend(segment)
            else:
                full_path.extend(segment[1:])  # Skip first point (duplicate)
            
            if verbose:
                logger.info(f"Segment {i+1}/{len(ordered_indices)-1}: "
                          f"waypoint {from_idx} -> {to_idx}, "
                          f"{len(segment)} points")
        
        full_path = np.array(full_path)
        
        if verbose:
            logger.info(f"Full path computed: {len(full_path)} waypoints total")
        
        return full_path, segment_paths
    
    def plan(
        self,
        waypoints: np.ndarray,
        start_index: int = 0,
        input_interval: float = 0.5,
        verbose: bool = True
    ) -> Dict:
        """
        Complete planning pipeline: compute distances, solve TSP, generate path.
        
        This is the main method that runs the complete planning process:
        1. Compute all-pair shortest path distances using A*
        2. Solve TSP to find optimal waypoint ordering
        3. Generate complete trajectory following the TSP solution
        
        Args:
            waypoints: Array of shape (N, 2) with [x, y] coordinates in meters
            start_index: Index of starting waypoint (default 0)
            input_interval: Spacing between trajectory points (default 0.5m)
            verbose: Whether to log detailed progress
            
        Returns:
            solution: Dictionary containing:
                - 'waypoints': Original waypoint coordinates
                - 'ordered_indices': TSP solution (list of indices)
                - 'distance_matrix': All-pair shortest path distances
                - 'full_path': Complete dense trajectory (np.ndarray)
                - 'segment_paths': List of path segments between waypoints
                - 'total_cost': Total path length in meters
                - 'metadata': Additional info (resolution, computation time, etc.)
        """
        start_time = time.time()
        
        if verbose:
            logger.info("=" * 70)
            logger.info("Starting complete TSP path planning pipeline")
            logger.info("=" * 70)
        
        # Step 1: Compute all-pair distances
        distance_matrix = self.compute_all_pair_distances(waypoints, verbose=verbose)
        
        # Step 2: Solve TSP
        ordered_indices, total_cost = self.solve_tsp(distance_matrix, start_index)
        
        # Step 3: Compute full path
        full_path, segment_paths = self.compute_full_path(
            waypoints, ordered_indices, input_interval, verbose=verbose
        )
        
        elapsed = time.time() - start_time
        
        # Package results
        solution = {
            'waypoints': waypoints.copy(),
            'ordered_indices': ordered_indices,
            'distance_matrix': distance_matrix,
            'full_path': full_path,
            'segment_paths': segment_paths,
            'total_cost': total_cost,
            'metadata': {
                'pixels_per_meter': self.pixels_per_meter,
                'map_shape': self.navigable_map.shape,
                'num_waypoints': len(waypoints),
                'path_length': len(full_path),
                'computation_time_seconds': elapsed,
                'input_interval': input_interval,
            }
        }
        
        if verbose:
            logger.info("=" * 70)
            logger.info("Planning complete!")
            logger.info(f"  Total cost: {total_cost:.2f}m")
            logger.info(f"  Path length: {len(full_path)} waypoints")
            logger.info(f"  Computation time: {elapsed:.2f}s")
            logger.info("=" * 70)
        
        return solution
    
    def save_solution(
        self,
        solution: Dict,
        filepath: str,
        format: str = 'npz'
    ):
        """
        Save planning solution to file.
        
        Args:
            solution: Solution dictionary from plan()
            filepath: Output file path
            format: File format - 'npz' (default) or 'pickle'
        """
        if format == 'npz':
            # Save as compressed numpy archive
            np.savez_compressed(
                filepath,
                waypoints=solution['waypoints'],
                ordered_indices=np.array(solution['ordered_indices']),
                distance_matrix=solution['distance_matrix'],
                full_path=solution['full_path'],
                total_cost=solution['total_cost'],
                metadata=pickle.dumps(solution['metadata']),
                # Segment paths saved separately
                **{f'segment_{i}': seg for i, seg in enumerate(solution['segment_paths'])}
            )
            logger.info(f"Solution saved to {filepath} (npz format)")
        
        elif format == 'pickle':
            with open(filepath, 'wb') as f:
                pickle.dump(solution, f)
            logger.info(f"Solution saved to {filepath} (pickle format)")
        
        else:
            raise ValueError(f"Unknown format: {format}. Use 'npz' or 'pickle'")
    
    def load_solution(
        self,
        filepath: str,
        format: str = 'npz'
    ) -> Dict:
        """
        Load planning solution from file.
        
        Args:
            filepath: Input file path
            format: File format - 'npz' (default) or 'pickle'
            
        Returns:
            solution: Solution dictionary
        """
        if format == 'npz':
            data = np.load(filepath, allow_pickle=True)
            
            # Reconstruct segment paths
            segment_paths = []
            i = 0
            while f'segment_{i}' in data:
                segment_paths.append(data[f'segment_{i}'])
                i += 1
            
            solution = {
                'waypoints': data['waypoints'],
                'ordered_indices': data['ordered_indices'].tolist(),
                'distance_matrix': data['distance_matrix'],
                'full_path': data['full_path'],
                'segment_paths': segment_paths,
                'total_cost': float(data['total_cost']),
                'metadata': pickle.loads(data['metadata'].tobytes()),
            }
            logger.info(f"Solution loaded from {filepath} (npz format)")
        
        elif format == 'pickle':
            with open(filepath, 'rb') as f:
                solution = pickle.load(f)
            logger.info(f"Solution loaded from {filepath} (pickle format)")
        
        else:
            raise ValueError(f"Unknown format: {format}. Use 'npz' or 'pickle'")
        
        return solution
