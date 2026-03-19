#!/usr/bin/env python3
"""Unit tests for pathfinding algorithms."""

import numpy as np
import pytest

from amr_nav.auto_nav import (
    occupancy_to_grid,
    find_next_target,
    astar_wall_penalty,
    cluster_path,
)


# ═══════════════════════════════════════════════════════════
# occupancy_to_grid
# ═══════════════════════════════════════════════════════════
class TestOccupancyToGrid:
    def test_basic_conversion(self):
        # 3x3: free, free, wall, unknown, free, wall, free, below-thresh, free
        data = [0, 0, 50, -1, 0, 100, 0, 5, 0]
        grid = occupancy_to_grid(data, 3, 3, wall_threshold=10)
        expected = np.array([
            [0, 0, 1],   # 50 >= 10
            [1, 0, 1],   # -1 → occupied, 100 >= 10
            [0, 0, 0],   # 5 < 10 → free
        ], dtype=np.int8)
        np.testing.assert_array_equal(grid, expected)

    def test_empty_grid(self):
        grid = occupancy_to_grid([], 0, 0)
        assert grid.shape == (0, 0)

    def test_all_free(self):
        grid = occupancy_to_grid([0] * 9, 3, 3)
        assert np.all(grid == 0)

    def test_all_occupied(self):
        grid = occupancy_to_grid([100] * 9, 3, 3)
        assert np.all(grid == 1)

    def test_unknown_is_occupied(self):
        grid = occupancy_to_grid([-1] * 4, 2, 2)
        assert np.all(grid == 1)

    def test_custom_threshold(self):
        data = [0, 5, 15, 50]
        grid = occupancy_to_grid(data, 2, 2, wall_threshold=20)
        expected = np.array([[0, 0], [0, 1]], dtype=np.int8)
        np.testing.assert_array_equal(grid, expected)


# ═══════════════════════════════════════════════════════════
# find_next_target
# ═══════════════════════════════════════════════════════════
class TestFindNextTarget:
    def test_simple_target(self):
        grid = np.zeros((10, 10), dtype=np.int8)
        visited = {(5, 5)}
        target = find_next_target(grid, visited, distance_threshold=20)
        assert target is not None
        tr, tc = target
        # Target should be away from the visited cell
        assert abs(tr - 5) + abs(tc - 5) > 1

    def test_target_is_furthest(self):
        grid = np.zeros((10, 10), dtype=np.int8)
        visited = {(0, 0)}
        target = find_next_target(grid, visited, distance_threshold=50)
        assert target is not None
        tr, tc = target
        # Should be a corner far from (0, 0)
        assert tr + tc > 10

    def test_no_target_all_visited(self):
        grid = np.zeros((3, 3), dtype=np.int8)
        visited = {(r, c) for r in range(3) for c in range(3)}
        assert find_next_target(grid, visited) is None

    def test_no_target_all_walls(self):
        grid = np.ones((3, 3), dtype=np.int8)
        # Only source cell happens to be on a wall — no expansion
        visited = {(1, 1)}
        assert find_next_target(grid, visited) is None

    def test_empty_visited(self):
        grid = np.zeros((5, 5), dtype=np.int8)
        assert find_next_target(grid, set()) is None

    def test_distance_threshold_caps(self):
        grid = np.zeros((20, 20), dtype=np.int8)
        visited = {(0, 0)}
        target = find_next_target(grid, visited, distance_threshold=3)
        if target is not None:
            tr, tc = target
            assert tr + tc <= 3

    def test_walls_block_expansion(self):
        grid = np.zeros((5, 5), dtype=np.int8)
        # Wall row isolates top from bottom
        grid[2, :] = 1
        visited = {(0, 0)}
        target = find_next_target(grid, visited, distance_threshold=50)
        # Target must be in top section (rows 0-1)
        if target is not None:
            assert target[0] < 2


# ═══════════════════════════════════════════════════════════
# astar_wall_penalty
# ═══════════════════════════════════════════════════════════
class TestAstarWallPenalty:
    def test_straight_path(self):
        grid = np.zeros((5, 5), dtype=np.int8)
        path = astar_wall_penalty(grid, (0, 0), (4, 4),
                                  wall_penalty=1, wall_cost=0)
        assert len(path) > 0
        assert path[0] == (0, 0)
        assert path[-1] == (4, 4)

    def test_path_continuity(self):
        grid = np.zeros((5, 5), dtype=np.int8)
        path = astar_wall_penalty(grid, (0, 0), (4, 4),
                                  wall_penalty=1, wall_cost=0)
        for i in range(len(path) - 1):
            r1, c1 = path[i]
            r2, c2 = path[i + 1]
            assert abs(r1 - r2) + abs(c1 - c2) == 1

    def test_no_path_blocked(self):
        grid = np.zeros((5, 5), dtype=np.int8)
        grid[2, :] = 1  # impassable wall
        path = astar_wall_penalty(grid, (0, 0), (4, 0))
        assert len(path) == 0

    def test_avoids_walls(self):
        grid = np.zeros((10, 10), dtype=np.int8)
        grid[:, 0] = 1  # wall on left edge
        path = astar_wall_penalty(grid, (0, 5), (9, 5),
                                  wall_penalty=2, wall_cost=200)
        assert len(path) > 0
        for r, c in path:
            assert c >= 1  # never on the wall itself

    def test_start_equals_goal(self):
        grid = np.zeros((5, 5), dtype=np.int8)
        path = astar_wall_penalty(grid, (2, 2), (2, 2))
        assert path == [(2, 2)]

    def test_start_on_wall(self):
        grid = np.zeros((5, 5), dtype=np.int8)
        grid[0, 0] = 1
        assert astar_wall_penalty(grid, (0, 0), (4, 4)) == []

    def test_goal_on_wall(self):
        grid = np.zeros((5, 5), dtype=np.int8)
        grid[4, 4] = 1
        assert astar_wall_penalty(grid, (0, 0), (4, 4)) == []

    def test_out_of_bounds(self):
        grid = np.zeros((5, 5), dtype=np.int8)
        assert astar_wall_penalty(grid, (-1, 0), (4, 4)) == []
        assert astar_wall_penalty(grid, (0, 0), (5, 5)) == []

    def test_wall_penalty_steers_away(self):
        """With a narrow corridor, penalty should still find the path."""
        grid = np.zeros((5, 5), dtype=np.int8)
        grid[0, :] = 1
        grid[4, :] = 1
        grid[:, 0] = 1
        grid[:, 4] = 1
        # Open interior 3x3
        path = astar_wall_penalty(grid, (1, 1), (3, 3),
                                  wall_penalty=1, wall_cost=50)
        assert len(path) > 0
        assert path[0] == (1, 1)
        assert path[-1] == (3, 3)


# ═══════════════════════════════════════════════════════════
# cluster_path
# ═══════════════════════════════════════════════════════════
class TestClusterPath:
    def test_basic_clustering(self):
        path = [(0, i) for i in range(20)]
        clusters = cluster_path(path, cluster_distance=6)
        assert clusters[0] == (0, 0)
        assert clusters[-1] == (0, 19)
        assert len(clusters) < len(path)

    def test_spacing(self):
        path = [(0, i) for i in range(20)]
        clusters = cluster_path(path, cluster_distance=6)
        for i in range(len(clusters) - 2):
            r1, c1 = clusters[i]
            r2, c2 = clusters[i + 1]
            assert abs(r1 - r2) + abs(c1 - c2) >= 6

    def test_single_point(self):
        assert cluster_path([(5, 5)]) == [(5, 5)]

    def test_empty_path(self):
        assert cluster_path([]) == []

    def test_two_close_points(self):
        clusters = cluster_path([(0, 0), (0, 1)], cluster_distance=6)
        assert (0, 0) in clusters
        assert (0, 1) in clusters

    def test_preserves_start_end(self):
        path = [(0, 0), (0, 3), (0, 6), (0, 10), (0, 15)]
        clusters = cluster_path(path, cluster_distance=6)
        assert clusters[0] == (0, 0)
        assert clusters[-1] == (0, 15)

    def test_all_far_apart(self):
        path = [(0, 0), (0, 10), (0, 20)]
        clusters = cluster_path(path, cluster_distance=6)
        assert clusters == path
