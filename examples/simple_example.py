"""
Simple example demonstrating basic TSP path planning.

This example creates a small 5m x 5m grid with obstacles and plans
a path through 5 waypoints using TSP optimization.
"""

import numpy as np
import logging
import sys
import os

# Add parent directory to path to import tsp_path_planner
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from tsp_path_planner import TSPPathPlanner, visualize_grid
from tsp_path_planner.visualization import plot_distance_matrix

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def main():
    """Run simple TSP path planning example."""
    
    logger.info("=" * 70)
    logger.info("Simple TSP Path Planning Example")
    logger.info("=" * 70)
    
    # ========== Step 1: Create Grid World ==========
    logger.info("\n[Step 1] Creating 5m x 5m grid world...")
    
    # Create 100x100 grid (5m x 5m at 20 pixels/meter)
    pixels_per_meter = 20
    grid_size = 100
    navigable_map = np.ones((grid_size, grid_size), dtype=bool)
    
    # Add vertical wall obstacle
    navigable_map[30:70, 45:55] = False
    logger.info(f"  Grid size: {grid_size}x{grid_size} pixels")
    logger.info(f"  Resolution: {pixels_per_meter} pixels/meter")
    logger.info(f"  Physical size: {grid_size/pixels_per_meter}m x {grid_size/pixels_per_meter}m")
    logger.info(f"  Added vertical wall obstacle")
    
    # ========== Step 2: Define Waypoints ==========
    logger.info("\n[Step 2] Defining waypoints...")
    
    waypoints = np.array([
        [0.0, -2.0],      # Start point (center)
        [2.0, 1.5],      # Top right
        [-1.0, 2.0],     # Top left
        [1.5, -1.5],     # Bottom right
        [-2.0, -1.0],    # Bottom left
    ])
    
    logger.info(f"  Number of waypoints: {len(waypoints)}")
    for i, wp in enumerate(waypoints):
        logger.info(f"    Waypoint {i}: [{wp[0]:.2f}, {wp[1]:.2f}]")
    
    # ========== Step 3: Create Planner ==========
    logger.info("\n[Step 3] Creating TSP path planner...")
    
    planner = TSPPathPlanner(
        navigable_map=navigable_map,
        pixels_per_meter=pixels_per_meter,
        astar_config={
            'waypoint_threshold': 0.3,
            'dilation_radius': 0.1,  # Small safety margin
        }
    )
    logger.info("  Planner initialized successfully")
    
    # ========== Step 4: Solve Complete Problem ==========
    logger.info("\n[Step 4] Solving TSP path planning problem...")
    logger.info("  This will:")
    logger.info("    a) Compute shortest paths between all waypoint pairs")
    logger.info("    b) Solve TSP to find optimal visit order")
    logger.info("    c) Generate complete trajectory")
    
    solution = planner.plan(waypoints, start_index=0, verbose=True)
    
    # ========== Step 5: Display Results ==========
    logger.info("\n[Step 5] Results:")
    logger.info("=" * 70)
    logger.info(f"  TSP Visit Order: {solution['ordered_indices']}")
    logger.info(f"  Total Path Cost: {solution['total_cost']:.2f} meters")
    logger.info(f"  Path Waypoints: {len(solution['full_path'])} points")
    logger.info(f"  Computation Time: {solution['metadata']['computation_time_seconds']:.2f}s")
    logger.info("=" * 70)
    
    # Print waypoint order with coordinates
    logger.info("\nDetailed visit sequence:")
    for i, idx in enumerate(solution['ordered_indices']):
        wp = waypoints[idx]
        logger.info(f"  {i+1}. Waypoint {idx}: [{wp[0]:.2f}, {wp[1]:.2f}]")
    
    # ========== Step 6: Save Solution ==========
    logger.info("\n[Step 6] Saving solution...")
    
    output_dir = "output"
    os.makedirs(output_dir, exist_ok=True)
    
    solution_file = os.path.join(output_dir, "simple_solution.npz")
    planner.save_solution(solution, solution_file)
    logger.info(f"  Solution saved to: {solution_file}")
    
    # ========== Step 7: Visualize ==========
    logger.info("\n[Step 7] Creating visualizations...")
    
    # Main visualization
    viz_file = os.path.join(output_dir, "simple_visualization.png")
    visualize_grid(
        navigable_map=navigable_map,
        waypoints=waypoints,
        ordered_indices=solution['ordered_indices'],
        full_path=solution['full_path'],
        segment_paths=solution['segment_paths'],
        pixels_per_meter=pixels_per_meter,
        save_path=viz_file,
        show=True
    )
    logger.info(f"  Main visualization saved to: {viz_file}")
    
    # Distance matrix visualization
    dist_matrix_file = os.path.join(output_dir, "simple_distance_matrix.png")
    plot_distance_matrix(
        distance_matrix=solution['distance_matrix'],
        waypoint_labels=[f"W{i}" for i in range(len(waypoints))],
        save_path=dist_matrix_file,
        show=False
    )
    logger.info(f"  Distance matrix saved to: {dist_matrix_file}")
    
    # ========== Step 8: Verify Solution Can Be Loaded ==========
    logger.info("\n[Step 8] Verifying save/load functionality...")
    
    loaded_solution = planner.load_solution(solution_file)
    logger.info("  Solution loaded successfully!")
    logger.info(f"  Loaded TSP order: {loaded_solution['ordered_indices']}")
    logger.info(f"  Loaded path cost: {loaded_solution['total_cost']:.2f}m")
    
    # Verify data matches
    assert np.array_equal(solution['ordered_indices'], loaded_solution['ordered_indices'])
    assert abs(solution['total_cost'] - loaded_solution['total_cost']) < 1e-6
    logger.info("  ✓ Verification passed!")
    
    logger.info("\n" + "=" * 70)
    logger.info("Example completed successfully!")
    logger.info("=" * 70)
    logger.info(f"\nOutput files created in '{output_dir}/' directory:")
    logger.info(f"  - {solution_file}")
    logger.info(f"  - {viz_file}")
    logger.info(f"  - {dist_matrix_file}")


if __name__ == "__main__":
    main()
