"""
Visualization utilities for TSP path planning.
"""

import logging
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch
from matplotlib.animation import FuncAnimation, PillowWriter
from typing import List, Optional
import matplotlib.colors as mcolors

logger = logging.getLogger(__name__)


def visualize_grid(
    navigable_map: np.ndarray,
    waypoints: Optional[np.ndarray] = None,
    ordered_indices: Optional[List[int]] = None,
    full_path: Optional[np.ndarray] = None,
    segment_paths: Optional[List[np.ndarray]] = None,
    pixels_per_meter: int = 20,
    save_path: Optional[str] = None,
    show: bool = True,
    figsize: tuple = (12, 10)
):
    """
    Visualize grid with obstacles, waypoints, and paths.
    
    Features:
    - Display navigable/obstacle cells with different colors
    - Mark waypoints with numbers showing TSP order
    - Draw path segments between consecutive waypoints
    - Show complete trajectory
    - Add legend and grid
    
    Args:
        navigable_map: Boolean grid map where True = navigable
        waypoints: Waypoint coordinates (N, 2) in meters
        ordered_indices: TSP ordering of waypoints
        full_path: Complete dense trajectory (M, 2) in meters
        segment_paths: Individual path segments between waypoints
        pixels_per_meter: Grid resolution
        save_path: Optional path to save figure
        show: Whether to display plot
        figsize: Figure size (width, height)
    """
    fig, ax = plt.subplots(figsize=figsize)
    
    # Get map dimensions in meters
    height, width = navigable_map.shape
    height_m = height / pixels_per_meter
    width_m = width / pixels_per_meter
    
    # Set axis limits centered at origin
    ax.set_xlim(-width_m/2, width_m/2)
    ax.set_ylim(-height_m/2, height_m/2)
    ax.set_aspect('equal')
    
    # Plot navigable map
    # Convert boolean map to RGB image
    map_image = np.zeros((height, width, 3))
    map_image[navigable_map] = [1.0, 1.0, 1.0]  # White for navigable
    map_image[~navigable_map] = [0.2, 0.2, 0.2]  # Dark gray for obstacles
    
    # Flip vertically to match VLFM's OpenCV convention (cv2.flip(img, 0))
    map_image = np.flip(map_image, axis=0)
    
    # Display map with origin='upper' to match the flipped image
    ax.imshow(
        map_image,
        extent=[-width_m/2, width_m/2, -height_m/2, height_m/2],
        origin='upper',
        zorder=0
    )
    
    # Plot full path if available
    if full_path is not None and len(full_path) > 0:
        ax.plot(
            full_path[:, 0],
            full_path[:, 1],
            'c-',
            linewidth=2,
            alpha=0.6,
            label='Full trajectory',
            zorder=2
        )
    
    # Plot individual segment paths with different colors
    if segment_paths is not None and len(segment_paths) > 0:
        colors = plt.cm.rainbow(np.linspace(0, 1, len(segment_paths)))
        for i, segment in enumerate(segment_paths):
            if len(segment) > 0:
                ax.plot(
                    segment[:, 0],
                    segment[:, 1],
                    '-',
                    color=colors[i],
                    linewidth=1.5,
                    alpha=0.7,
                    zorder=1
                )
    
    # Plot waypoints
    if waypoints is not None:
        # If we have TSP ordering, use it
        if ordered_indices is not None:
            # Plot waypoints with TSP order numbers
            for i, idx in enumerate(ordered_indices):
                color = 'green' if i == 0 else 'red' if i == len(ordered_indices)-1 else 'blue'
                ax.plot(
                    waypoints[idx, 0],
                    waypoints[idx, 1],
                    'o',
                    color=color,
                    markersize=12,
                    markeredgecolor='black',
                    markeredgewidth=2,
                    zorder=4,
                    label=f'Start' if i == 0 else f'End' if i == len(ordered_indices)-1 else None
                )
                
                # Add order number
                ax.text(
                    waypoints[idx, 0],
                    waypoints[idx, 1],
                    str(i),
                    fontsize=10,
                    fontweight='bold',
                    color='white',
                    ha='center',
                    va='center',
                    zorder=5
                )
        else:
            # Just plot waypoints without order
            ax.plot(
                waypoints[:, 0],
                waypoints[:, 1],
                'ro',
                markersize=10,
                markeredgecolor='black',
                markeredgewidth=2,
                label='Waypoints',
                zorder=4
            )
    
    # Add grid
    ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    
    # Labels and title
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    
    title = 'TSP Path Planning Visualization'
    if ordered_indices is not None and waypoints is not None:
        title += f'\n{len(waypoints)} waypoints, Visit order: {ordered_indices}'
    ax.set_title(title, fontsize=14, fontweight='bold')
    
    # Legend
    handles, labels = ax.get_legend_handles_labels()
    # Remove duplicate labels
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='upper right', fontsize=10)
    
    plt.tight_layout()
    
    # Save if requested
    if save_path is not None:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        logger.info(f"Figure saved to {save_path}")
    
    # Show if requested
    if show:
        plt.show()
    
    return fig, ax


def create_animation(
    navigable_map: np.ndarray,
    full_path: np.ndarray,
    waypoints: np.ndarray,
    ordered_indices: List[int],
    pixels_per_meter: int = 20,
    save_path: Optional[str] = None,
    fps: int = 10,
    figsize: tuple = (10, 8)
):
    """
    Create animated visualization of robot following path.
    
    Args:
        navigable_map: Boolean grid map
        full_path: Complete trajectory waypoints (M, 2)
        waypoints: TSP waypoints (N, 2)
        ordered_indices: Visit order
        pixels_per_meter: Grid resolution
        save_path: Path to save animation (should end with .gif or .mp4)
        fps: Frames per second
        figsize: Figure size
    """
    fig, ax = plt.subplots(figsize=figsize)
    
    # Get map dimensions
    height, width = navigable_map.shape
    height_m = height / pixels_per_meter
    width_m = width / pixels_per_meter
    
    ax.set_xlim(-width_m/2, width_m/2)
    ax.set_ylim(-height_m/2, height_m/2)
    ax.set_aspect('equal')
    
    # Plot navigable map
    map_image = np.zeros((height, width, 3))
    map_image[navigable_map] = [1.0, 1.0, 1.0]
    map_image[~navigable_map] = [0.2, 0.2, 0.2]
    
    # Flip vertically to match VLFM's OpenCV convention (cv2.flip(img, 0))
    map_image = np.flip(map_image, axis=0)
    
    ax.imshow(
        map_image,
        extent=[-width_m/2, width_m/2, -height_m/2, height_m/2],
        origin='upper',
        zorder=0
    )
    
    # Plot waypoints
    for i, idx in enumerate(ordered_indices):
        color = 'green' if i == 0 else 'red' if i == len(ordered_indices)-1 else 'blue'
        ax.plot(
            waypoints[idx, 0],
            waypoints[idx, 1],
            'o',
            color=color,
            markersize=10,
            markeredgecolor='black',
            markeredgewidth=2,
            zorder=3
        )
        ax.text(
            waypoints[idx, 0],
            waypoints[idx, 1],
            str(i),
            fontsize=8,
            fontweight='bold',
            color='white',
            ha='center',
            va='center',
            zorder=4
        )
    
    # Initialize animated elements
    robot, = ax.plot([], [], 'ro', markersize=15, markeredgecolor='yellow', 
                     markeredgewidth=3, label='Robot', zorder=5)
    trail, = ax.plot([], [], 'c-', linewidth=2, alpha=0.6, label='Path taken', zorder=2)
    
    ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    ax.set_xlabel('X (meters)', fontsize=11)
    ax.set_ylabel('Y (meters)', fontsize=11)
    ax.set_title('Robot Navigation Animation', fontsize=13, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9)
    
    # Animation data
    path_x = []
    path_y = []
    
    def init():
        robot.set_data([], [])
        trail.set_data([], [])
        return robot, trail
    
    def animate(frame):
        if frame < len(full_path):
            # Update robot position
            robot.set_data([full_path[frame, 0]], [full_path[frame, 1]])
            
            # Update trail
            path_x.append(full_path[frame, 0])
            path_y.append(full_path[frame, 1])
            trail.set_data(path_x, path_y)
        
        return robot, trail
    
    # Create animation
    anim = FuncAnimation(
        fig,
        animate,
        init_func=init,
        frames=len(full_path),
        interval=1000/fps,
        blit=True,
        repeat=True
    )
    
    # Save if requested
    if save_path is not None:
        if save_path.endswith('.gif'):
            writer = PillowWriter(fps=fps)
            anim.save(save_path, writer=writer)
            logger.info(f"Animation saved to {save_path}")
        elif save_path.endswith('.mp4'):
            try:
                anim.save(save_path, writer='ffmpeg', fps=fps)
                logger.info(f"Animation saved to {save_path}")
            except Exception as e:
                logger.error(f"Failed to save MP4 (ffmpeg required): {e}")
                logger.info("Falling back to GIF format")
                gif_path = save_path.replace('.mp4', '.gif')
                writer = PillowWriter(fps=fps)
                anim.save(gif_path, writer=writer)
                logger.info(f"Animation saved to {gif_path}")
        else:
            logger.warning(f"Unknown animation format: {save_path}. Use .gif or .mp4")
    
    plt.tight_layout()
    plt.show()
    
    return anim


def plot_distance_matrix(
    distance_matrix: np.ndarray,
    waypoint_labels: Optional[List[str]] = None,
    save_path: Optional[str] = None,
    show: bool = True
):
    """
    Visualize distance matrix as a heatmap.
    
    Args:
        distance_matrix: NxN distance matrix
        waypoint_labels: Optional labels for waypoints
        save_path: Optional path to save figure
        show: Whether to display plot
    """
    fig, ax = plt.subplots(figsize=(8, 7))
    
    im = ax.imshow(distance_matrix, cmap='viridis', aspect='auto')
    
    # Add colorbar
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label('Distance (meters)', rotation=270, labelpad=20)
    
    # Set ticks and labels
    n = len(distance_matrix)
    if waypoint_labels is None:
        waypoint_labels = [f'W{i}' for i in range(n)]
    
    ax.set_xticks(np.arange(n))
    ax.set_yticks(np.arange(n))
    ax.set_xticklabels(waypoint_labels)
    ax.set_yticklabels(waypoint_labels)
    
    # Rotate x labels
    plt.setp(ax.get_xticklabels(), rotation=45, ha="right", rotation_mode="anchor")
    
    # Add text annotations
    for i in range(n):
        for j in range(n):
            text = ax.text(j, i, f'{distance_matrix[i, j]:.1f}',
                          ha="center", va="center", color="w", fontsize=8)
    
    ax.set_title('All-Pair Shortest Path Distances', fontsize=13, fontweight='bold')
    plt.tight_layout()
    
    if save_path is not None:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        logger.info(f"Distance matrix plot saved to {save_path}")
    
    if show:
        plt.show()
    
    return fig, ax
