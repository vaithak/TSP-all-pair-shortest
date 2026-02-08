"""
Unit tests for TSP path planner.
"""

import pytest
import numpy as np
import sys
import os
import tempfile

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from tsp_path_planner import TSPPathPlanner


class TestTSPPathPlanner:
    """Test suite for TSPPathPlanner class."""
    
    @pytest.fixture
    def simple_map(self):
        """Create a simple 50x50 navigable map."""
        return np.ones((50, 50), dtype=bool)
    
    @pytest.fixture
    def map_with_obstacle(self):
        """Create a map with a central obstacle."""
        nav_map = np.ones((50, 50), dtype=bool)
        nav_map[20:30, 20:30] = False
        return nav_map
    
    @pytest.fixture
    def simple_waypoints(self):
        """Create 3 simple waypoints."""
        return np.array([
            [0.0, 0.0],
            [1.0, 1.0],
            [-1.0, 1.0]
        ])
    
    def test_planner_initialization(self, simple_map):
        """Test that planner initializes correctly."""
        planner = TSPPathPlanner(simple_map, pixels_per_meter=10)
        
        assert planner.navigable_map.shape == (50, 50)
        assert planner.pixels_per_meter == 10
        assert planner.astar_planner is not None
        assert planner.tsp_solver is not None
    
    def test_all_pair_distances(self, simple_map, simple_waypoints):
        """Test computation of all-pair distances."""
        planner = TSPPathPlanner(simple_map, pixels_per_meter=10)
        
        distance_matrix = planner.compute_all_pair_distances(
            simple_waypoints, 
            verbose=False
        )
        
        assert distance_matrix.shape == (3, 3)
        # Diagonal should be zero
        assert np.allclose(np.diag(distance_matrix), 0)
        # Matrix should be symmetric
        assert np.allclose(distance_matrix, distance_matrix.T)
        # All distances should be positive
        assert np.all(distance_matrix >= 0)
    
    def test_tsp_solve(self, simple_map, simple_waypoints):
        """Test TSP solving."""
        planner = TSPPathPlanner(simple_map, pixels_per_meter=10)
        
        distance_matrix = planner.compute_all_pair_distances(
            simple_waypoints,
            verbose=False
        )
        
        ordered_indices, total_cost = planner.solve_tsp(distance_matrix, start_index=0)
        
        assert len(ordered_indices) == len(simple_waypoints)
        assert ordered_indices[0] == 0  # Should start at depot
        assert total_cost > 0
        # All waypoints should be visited
        assert set(ordered_indices) == set(range(len(simple_waypoints)))
    
    def test_full_path_computation(self, simple_map, simple_waypoints):
        """Test full path generation."""
        planner = TSPPathPlanner(simple_map, pixels_per_meter=10)
        
        ordered_indices = [0, 1, 2]
        full_path, segment_paths = planner.compute_full_path(
            simple_waypoints,
            ordered_indices,
            input_interval=0.5,
            verbose=False
        )
        
        assert len(full_path) > len(simple_waypoints)
        assert len(segment_paths) == len(ordered_indices) - 1
        assert full_path.shape[1] == 2  # 2D coordinates
    
    def test_complete_planning_pipeline(self, simple_map, simple_waypoints):
        """Test the complete planning pipeline."""
        planner = TSPPathPlanner(simple_map, pixels_per_meter=10)
        
        solution = planner.plan(simple_waypoints, start_index=0, verbose=False)
        
        assert 'waypoints' in solution
        assert 'ordered_indices' in solution
        assert 'distance_matrix' in solution
        assert 'full_path' in solution
        assert 'segment_paths' in solution
        assert 'total_cost' in solution
        assert 'metadata' in solution
        
        assert len(solution['ordered_indices']) == len(simple_waypoints)
        assert solution['total_cost'] > 0
        assert len(solution['full_path']) > 0
    
    def test_obstacle_avoidance(self, map_with_obstacle):
        """Test that paths avoid obstacles."""
        planner = TSPPathPlanner(map_with_obstacle, pixels_per_meter=10)
        
        # Waypoints on opposite sides of obstacle
        waypoints = np.array([
            [0.0, 0.0],
            [2.0, 2.0]
        ])
        
        solution = planner.plan(waypoints, verbose=False)
        
        # Path should exist and avoid obstacle
        assert solution['full_path'] is not None
        assert len(solution['full_path']) > 2  # Should be longer than direct line
    
    def test_save_and_load_solution(self, simple_map, simple_waypoints):
        """Test solution save/load functionality."""
        planner = TSPPathPlanner(simple_map, pixels_per_meter=10)
        
        solution = planner.plan(simple_waypoints, verbose=False)
        
        with tempfile.TemporaryDirectory() as tmpdir:
            # Test npz format
            npz_path = os.path.join(tmpdir, 'test_solution.npz')
            planner.save_solution(solution, npz_path, format='npz')
            loaded_npz = planner.load_solution(npz_path, format='npz')
            
            assert np.array_equal(solution['waypoints'], loaded_npz['waypoints'])
            assert solution['ordered_indices'] == loaded_npz['ordered_indices']
            assert abs(solution['total_cost'] - loaded_npz['total_cost']) < 1e-6
            
            # Test pickle format
            pkl_path = os.path.join(tmpdir, 'test_solution.pkl')
            planner.save_solution(solution, pkl_path, format='pickle')
            loaded_pkl = planner.load_solution(pkl_path, format='pickle')
            
            assert np.array_equal(solution['waypoints'], loaded_pkl['waypoints'])
            assert solution['ordered_indices'] == loaded_pkl['ordered_indices']
    
    def test_different_start_indices(self, simple_map, simple_waypoints):
        """Test planning with different start waypoints."""
        planner = TSPPathPlanner(simple_map, pixels_per_meter=10)
        
        solution0 = planner.plan(simple_waypoints, start_index=0, verbose=False)
        solution1 = planner.plan(simple_waypoints, start_index=1, verbose=False)
        
        assert solution0['ordered_indices'][0] == 0
        assert solution1['ordered_indices'][0] == 1
        
        # Costs might be different depending on start point
        assert solution0['total_cost'] > 0
        assert solution1['total_cost'] > 0
    
    def test_single_waypoint(self, simple_map):
        """Test with a single waypoint."""
        planner = TSPPathPlanner(simple_map, pixels_per_meter=10)
        
        waypoints = np.array([[0.0, 0.0]])
        solution = planner.plan(waypoints, verbose=False)
        
        assert len(solution['ordered_indices']) == 1
        assert solution['ordered_indices'][0] == 0
    
    def test_two_waypoints(self, simple_map):
        """Test with two waypoints."""
        planner = TSPPathPlanner(simple_map, pixels_per_meter=10)
        
        waypoints = np.array([
            [0.0, 0.0],
            [1.0, 1.0]
        ])
        
        solution = planner.plan(waypoints, verbose=False)
        
        assert len(solution['ordered_indices']) == 2
        assert solution['ordered_indices'][0] == 0
        assert solution['total_cost'] > 0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
