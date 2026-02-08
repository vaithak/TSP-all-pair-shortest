"""
Complex example with more waypoints, obstacles, and animation.

This example demonstrates:
- Larger grid with multiple obstacles
- 10 waypoints requiring complex routing
- Animated visualization of robot following path
- Performance analysis
"""

import numpy as np
import logging
import sys
import os
import time

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from tsp_path_planner import TSPPathPlanner, visualize_grid
from tsp_path_planner.visualization import plot_distance_matrix, create_animation

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def create_complex_grid(size=200, pixels_per_meter=20):
    """
    Create a complex grid with multiple obstacles.
    
    Args:
        size: Grid size in pixels
        pixels_per_meter: Resolution
        
    Returns:
        navigable_map: Boolean navigability map
    """
    navigable_map = np.ones((size, size), dtype=bool)
    
    # Add vertical walls
    navigable_map[60:140, 90:95] = False
    navigable_map[60:140, 105:110] = False
    
    # Add horizontal walls
    navigable_map[40:45, 40:160] = False
    navigable_map[155:160, 40:160] = False
    
    # Add central obstacle (L-shape)
    navigable_map[80:120, 80:120] = False
    navigable_map[60:75, 80:135] = False
    
    # Add scattered small obstacles
    obstacle_positions = [
        (30, 30, 30, 30),
        (30, 160, 10, 10),
        (160, 30, 10, 10),
        (160, 160, 10, 10),
    ]
    
    for r, c, h, w in obstacle_positions:
        navigable_map[r:r+h, c:c+w] = False
    
    return navigable_map


def main():
    """Run complex TSP path planning example."""
    
    logger.info("=" * 70)
    logger.info("Complex TSP Path Planning Example with Animation")
    logger.info("=" * 70)
    
    # ========== Step 1: Create Complex Grid ==========
    logger.info("\n[Step 1] Creating 10m x 10m complex grid world...")
    
    pixels_per_meter = 20
    grid_size = 200
    navigable_map = create_complex_grid(grid_size, pixels_per_meter)
    
    navigable_count = np.sum(navigable_map)
    obstacle_count = grid_size * grid_size - navigable_count
    
    logger.info(f"  Grid size: {grid_size}x{grid_size} pixels")
    logger.info(f"  Resolution: {pixels_per_meter} pixels/meter")
    logger.info(f"  Physical size: {grid_size/pixels_per_meter}m x {grid_size/pixels_per_meter}m")
    logger.info(f"  Navigable cells: {navigable_count} ({100*navigable_count/(grid_size**2):.1f}%)")
    logger.info(f"  Obstacle cells: {obstacle_count} ({100*obstacle_count/(grid_size**2):.1f}%)")
    
    # ========== Step 2: Define More Waypoints ==========
    logger.info("\n[Step 2] Defining 10 waypoints...")
    
    waypoints = np.array([
        [-1.2, 0.0],       # 0: Center start
        [2.2, 3.2],       # 1: Top right quadrant
        [-2.2, 3.2],      # 2: Top left quadrant
        [-1.8, -3.2],     # 3: Bottom left quadrant
        [3.8, -3.5],      # 4: Bottom right quadrant
        [2.0, 0.5],       # 5: Right center
        [-2.0, 0.5],      # 6: Left center
        [0.5, 2.5],       # 7: Top center
        [0.5, -2.5],      # 8: Bottom center
        [1.3, 1.0],       # 9: Near center
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
            'dilation_radius': 0.25,  # Slightly larger safety margin
            'dilation_retry_step': 0.05,
        }
    )
    logger.info("  Planner initialized successfully")
    
    # ========== Step 4: Solve TSP Problem ==========
    logger.info("\n[Step 4] Solving TSP path planning problem...")
    logger.info("  (This may take a minute due to 10 waypoints = 45 path pairs)")
    
    start_time = time.time()
    solution = planner.plan(waypoints, start_index=0, verbose=True)
    total_time = time.time() - start_time
    
    # ========== Step 5: Analyze Results ==========
    logger.info("\n[Step 5] Results Analysis:")
    logger.info("=" * 70)
    logger.info(f"  TSP Visit Order: {solution['ordered_indices']}")
    logger.info(f"  Total Path Cost: {solution['total_cost']:.2f} meters")
    logger.info(f"  Path Waypoints: {len(solution['full_path'])} points")
    logger.info(f"  Number of Segments: {len(solution['segment_paths'])}")
    logger.info(f"  Total Computation Time: {total_time:.2f}s")
    logger.info("=" * 70)
    
    # Compute statistics
    segment_lengths = [len(seg) for seg in solution['segment_paths']]
    logger.info(f"\nPath segment statistics:")
    logger.info(f"  Average segment length: {np.mean(segment_lengths):.1f} waypoints")
    logger.info(f"  Min segment length: {np.min(segment_lengths)} waypoints")
    logger.info(f"  Max segment length: {np.max(segment_lengths)} waypoints")
    
    # ========== Step 6: Save Solution ==========
    logger.info("\n[Step 6] Saving solution...")
    
    output_dir = "output"
    os.makedirs(output_dir, exist_ok=True)
    
    solution_file = os.path.join(output_dir, "complex_solution.npz")
    planner.save_solution(solution, solution_file)
    logger.info(f"  Solution saved to: {solution_file}")
    
    # ========== Step 7: Create Visualizations ==========
    logger.info("\n[Step 7] Creating visualizations...")
    
    # Main visualization
    viz_file = os.path.join(output_dir, "complex_visualization.png")
    visualize_grid(
        navigable_map=navigable_map,
        waypoints=waypoints,
        ordered_indices=solution['ordered_indices'],
        full_path=solution['full_path'],
        segment_paths=solution['segment_paths'],
        pixels_per_meter=pixels_per_meter,
        save_path=viz_file,
        show=False,
        figsize=(14, 12)
    )
    logger.info(f"  Main visualization saved to: {viz_file}")
    
    # Distance matrix
    dist_matrix_file = os.path.join(output_dir, "complex_distance_matrix.png")
    plot_distance_matrix(
        distance_matrix=solution['distance_matrix'],
        waypoint_labels=[f"W{i}" for i in range(len(waypoints))],
        save_path=dist_matrix_file,
        show=False
    )
    logger.info(f"  Distance matrix saved to: {dist_matrix_file}")
    
    # ========== Step 8: Create Animation ==========
    logger.info("\n[Step 8] Creating animation...")
    logger.info("  (This may take a minute...)")
    
    animation_file = os.path.join(output_dir, "complex_animation.gif")
    
    try:
        create_animation(
            navigable_map=navigable_map,
            full_path=solution['full_path'],
            waypoints=waypoints,
            ordered_indices=solution['ordered_indices'],
            pixels_per_meter=pixels_per_meter,
            save_path=animation_file,
            fps=15,
            figsize=(12, 10)
        )
        logger.info(f"  Animation saved to: {animation_file}")
    except Exception as e:
        logger.error(f"  Failed to create animation: {e}")
        logger.info("  Skipping animation (matplotlib animation may require additional setup)")
    
    # ========== Step 9: Performance Comparison ==========
    logger.info("\n[Step 9] Performance comparison:")
    
    # Compare with naive nearest-neighbor order
    logger.info("  Computing naive nearest-neighbor path for comparison...")
    naive_order = [0]  # Start at waypoint 0
    remaining = set(range(1, len(waypoints)))
    
    while remaining:
        current = naive_order[-1]
        nearest = min(remaining, key=lambda x: solution['distance_matrix'][current][x])
        naive_order.append(nearest)
        remaining.remove(nearest)
    
    # Calculate naive path cost
    naive_cost = sum(solution['distance_matrix'][naive_order[i]][naive_order[i+1]] 
                     for i in range(len(naive_order)-1))
    
    improvement = ((naive_cost - solution['total_cost']) / naive_cost) * 100
    
    logger.info(f"\n  Comparison:")
    logger.info(f"    Naive nearest-neighbor cost: {naive_cost:.2f}m")
    logger.info(f"    TSP optimized cost: {solution['total_cost']:.2f}m")
    logger.info(f"    Improvement: {improvement:.1f}%")
    logger.info(f"    Saved distance: {naive_cost - solution['total_cost']:.2f}m")
    
    logger.info("\n" + "=" * 70)
    logger.info("Complex example completed successfully!")
    logger.info("=" * 70)
    logger.info(f"\nOutput files created in '{output_dir}/' directory:")
    logger.info(f"  - {solution_file}")
    logger.info(f"  - {viz_file}")
    logger.info(f"  - {dist_matrix_file}")
    if os.path.exists(animation_file):
        logger.info(f"  - {animation_file}")


if __name__ == "__main__":
    main()
