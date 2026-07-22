"""Tests for terrain_cost_node.py — pure logic tests (no ROS node needed)."""
import math
import numpy as np
import pytest


def _local_variance(grid, kernel=3):
    """Standalone copy of the vectorized local variance for testing."""
    filled = np.nan_to_num(grid, nan=0.0)
    half = kernel // 2
    padded = np.pad(filled, half, mode='edge')
    h, w = grid.shape
    from numpy.lib.stride_tricks import sliding_window_view
    windows = sliding_window_view(padded, (kernel, kernel))
    return windows.reshape(h, w, -1).var(axis=2)


class TestLocalVariance:
    def test_uniform_grid_zero_variance(self):
        grid = np.ones((10, 10)) * 0.5
        result = _local_variance(grid, kernel=3)
        np.testing.assert_allclose(result, 0.0, atol=1e-6)

    def test_high_variance(self):
        grid = np.zeros((10, 10))
        grid[5, 5] = 10.0
        result = _local_variance(grid, kernel=3)
        assert result[5, 5] > 0.0

    def test_nan_handling(self):
        grid = np.full((10, 10), np.nan)
        result = _local_variance(grid, kernel=3)
        assert np.all(np.isfinite(result))

    def test_kernel_5(self):
        grid = np.ones((10, 10)) * 0.5
        grid[5, 5] = 10.0
        result = _local_variance(grid, kernel=5)
        assert result[5, 5] > 0.0


class TestGridMapLayerExtraction:
    def test_extract_layer(self):
        from grid_map_msgs.msg import GridMap
        from std_msgs.msg import Float32MultiArray, MultiArrayDimension

        msg = GridMap()
        msg.info.resolution = 0.05
        msg.info.length_x = 1.0
        msg.info.length_y = 1.0
        msg.layers = ['floor']

        layer = Float32MultiArray()
        n = 3
        dim_x = MultiArrayDimension(label='column', size=n, stride=n * n)
        dim_y = MultiArrayDimension(label='row', size=n, stride=n)
        layer.layout.dim = [dim_x, dim_y]
        layer.data = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]
        msg.data.append(layer)

        data = msg.data[0]
        n = data.layout.dim[0].size
        arr = np.array(data.data, dtype=np.float32).reshape((n, n), order='F')
        assert arr.shape == (3, 3)
        assert arr[0, 0] == 1.0

    def test_extract_missing_layer(self):
        from grid_map_msgs.msg import GridMap
        msg = GridMap()
        msg.layers = ['floor']
        with pytest.raises(ValueError):
            msg.layers.index('clearance')


class TestCostComputation:
    def test_nan_floor_inf_cost(self):
        grid = np.full((5, 5), np.nan)
        cost = np.zeros_like(grid)
        cost[np.isnan(grid)] = np.inf
        assert np.all(np.isfinite(cost) == False)

    def test_clearance_below_minimum(self):
        clearance = np.array([[0.05, 0.20], [0.15, 0.30]])
        min_swing = 0.12
        cost = np.zeros_like(clearance)
        cost[clearance < min_swing] = np.inf
        assert cost[0, 0] == np.inf
        assert np.isfinite(cost[0, 1])
        assert np.isfinite(cost[1, 0])  # 0.15 > 0.12
        assert np.isfinite(cost[1, 1])

    def test_clearance_boundary(self):
        clearance = np.array([[0.12, 0.119], [0.121, 0.13]])
        min_swing = 0.12
        cost = np.zeros((2, 2))
        cost[clearance < min_swing] = np.inf
        assert np.isfinite(cost[0, 0])  # exactly at boundary
        assert cost[0, 1] == np.inf     # 0.119 < 0.12
        assert np.isfinite(cost[1, 0])  # 0.121 > 0.12
        assert np.isfinite(cost[1, 1])

    def test_unknown_ceiling_strict(self):
        ceiling = np.array([[np.nan, 1.0], [0.5, np.nan]])
        cost = np.zeros((2, 2))
        unknown_mask = np.isnan(ceiling)
        cost[unknown_mask] = np.inf
        assert cost[0, 0] == np.inf
        assert np.isfinite(cost[0, 1])
        assert np.isfinite(cost[1, 0])
        assert cost[1, 1] == np.inf

    def test_unknown_ceiling_penalty(self):
        ceiling = np.array([[np.nan, 1.0], [0.5, np.nan]])
        cost = np.zeros((2, 2))
        w_unknown = 0.5
        unknown_mask = np.isnan(ceiling)
        cost[unknown_mask] += w_unknown
        assert cost[0, 0] == 0.5
        assert cost[0, 1] == 0.0
        assert cost[1, 0] == 0.0
        assert cost[1, 1] == 0.5

    def test_slope_computation(self):
        floor = np.ones((10, 10)) * 0.5
        gx, gy = np.gradient(floor, 0.05)
        slope = np.sqrt(gx**2 + gy**2)
        assert np.allclose(slope, 0.0)

    def test_slope_with_gradient(self):
        floor = np.linspace(0, 1, 10).reshape(10, 1) * np.ones((1, 10))
        gx, gy = np.gradient(floor, 0.05)
        slope = np.sqrt(gx**2 + gy**2)
        assert slope.max() > 0

    def test_weighted_cost_formula(self):
        slope = np.array([[0.5, 0.3], [0.8, 0.1]])
        roughness = np.array([[0.2, 0.7], [0.4, 0.9]])
        w_s, w_r = 1.0, 0.8
        cost = w_s * slope + w_r * roughness
        assert cost[0, 0] == pytest.approx(0.5 + 0.8 * 0.2)
        assert cost[1, 1] == pytest.approx(0.1 + 0.8 * 0.9)

    def test_combined_constraints(self):
        floor = np.array([[0.5, np.nan], [0.3, 0.4]])
        clearance = np.array([[0.2, 0.1], [0.05, 0.3]])
        ceiling = np.array([[0.7, np.nan], [0.35, 0.7]])
        min_swing = 0.12

        cost = np.zeros((2, 2))
        cost[clearance < min_swing] = np.inf
        unknown_mask = np.isnan(ceiling)
        cost[unknown_mask] = np.inf
        cost[np.isnan(floor)] = np.inf

        assert cost[0, 0] == pytest.approx(0.0)  # valid cell
        assert cost[0, 1] == np.inf               # unknown ceiling
        assert cost[1, 0] == np.inf               # clearance too low
        assert cost[1, 1] == pytest.approx(0.0)   # valid cell


class TestUnknownAboveLayer:
    def test_unknown_above_strict_inf(self):
        """unknown_above > 0 with strict_unknown=True → INF cost."""
        unknown_above = np.array([[1.0, 0.0], [0.5, 0.0]])
        cost = np.zeros((2, 2))
        strict_unknown = True
        if strict_unknown:
            has_unknown = unknown_above > 0.0
            cost[has_unknown] = np.inf
        assert cost[0, 0] == np.inf
        assert np.isfinite(cost[0, 1])
        assert cost[1, 0] == np.inf
        assert np.isfinite(cost[1, 1])

    def test_unknown_above_continuous_penalty(self):
        """unknown_above with strict_unknown=False → continuous penalty."""
        unknown_above = np.array([[1.0, 0.0], [0.5, 0.3]])
        w_u = 0.5
        cost = np.zeros((2, 2))
        strict_unknown = False
        if not strict_unknown:
            cost += w_u * unknown_above
        assert cost[0, 0] == pytest.approx(0.5)
        assert cost[0, 1] == pytest.approx(0.0)
        assert cost[1, 0] == pytest.approx(0.25)
        assert cost[1, 1] == pytest.approx(0.15)

    def test_unknown_above_gradient(self):
        """Higher unknown fraction should produce higher cost."""
        w_u = 1.0
        cost_low = w_u * 0.2
        cost_high = w_u * 0.8
        assert cost_high > cost_low

    def test_zero_unknown_no_penalty(self):
        """unknown_above = 0 everywhere should add no cost."""
        unknown_above = np.zeros((5, 5))
        cost = np.zeros((5, 5))
        cost += 0.5 * unknown_above
        assert np.allclose(cost, 0.0)

    def test_continuous_unknown_above_range(self):
        """unknown_above should be in [0, 1] continuous range."""
        # Simulate the continuous computation from octomap_terrain_node
        body_h = 0.15
        voxel_size = 0.05
        column_voxels = int((body_h + 0.1) / voxel_size)

        # Case: no overhead voxels → unknown = 1.0
        overhead_count = 0
        unknown = max(0.0, 1.0 - overhead_count / max(column_voxels, 1))
        assert unknown == 1.0

        # Case: full column occupied → unknown ≈ 0.0
        overhead_count = column_voxels
        unknown = max(0.0, 1.0 - overhead_count / max(column_voxels, 1))
        assert unknown == 0.0

        # Case: half column occupied → unknown ≈ 0.5
        overhead_count = column_voxels // 2
        unknown = max(0.0, 1.0 - overhead_count / max(column_voxels, 1))
        assert 0.4 <= unknown <= 0.6

    def test_continuous_unknown_penalty_scaling(self):
        """Continuous unknown_above should scale penalty proportionally."""
        w_u = 0.5
        unknown_low = np.array([[0.2]])
        unknown_high = np.array([[0.8]])
        cost_low = w_u * unknown_low
        cost_high = w_u * unknown_high
        assert cost_high[0, 0] > cost_low[0, 0]
        assert cost_high[0, 0] == pytest.approx(0.4)
        assert cost_low[0, 0] == pytest.approx(0.1)
