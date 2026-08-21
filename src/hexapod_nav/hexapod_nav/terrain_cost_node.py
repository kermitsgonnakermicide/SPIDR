#!/usr/bin/env python3
"""
terrain_cost_node.py

Computes foothold cost per grid cell from terrain properties:
  - Slope cost        (gradient of floor height)
  - Roughness cost    (local variance of floor height)
  - Clearance cost    (ceiling - floor; hard-zero below swing threshold)
  - Unknown penalty   (unknown_above layer: continuous fraction 0..1)
  - Friction cost     (surface friction vs slope angle — penalty for slip risk)

Cost formula:
  cost(x,y) = w_s * slope(x,y) + w_r * roughness(x,y) + w_u * unknown_above(x,y)
              + w_f * friction_loss(x,y)
  subject to: cost = INF if clearance(x,y) < min_swing_height
              or (strict_unknown=True AND unknown_above(x,y) > 0)
"""

import rclpy
from rclpy.node import Node
import numpy as np
from grid_map_msgs.msg import GridMap

from .grid_map_utils import make_layer


class TerrainCostNode(Node):
    def __init__(self):
        super().__init__('terrain_cost_node')

        # Weights (tunable — these become your experimental variables)
        self.declare_parameter('w_slope', 1.0)
        self.declare_parameter('w_roughness', 0.8)
        self.declare_parameter('w_unknown', 0.5)         # Penalty for NaN/unknown ceiling
        self.declare_parameter('w_friction', 0.6)        # Penalty for low-friction surfaces
        self.declare_parameter('surface_friction_coefficient', 0.8)  # 0=ice, 1=rubber
        self.declare_parameter('min_swing_height', 0.12) # Minimum clearance for leg swing (m)
        self.declare_parameter('strict_unknown', True)   # Treat NaN ceiling as INFINITE cost

        self.w_s = self.get_parameter('w_slope').value
        self.w_r = self.get_parameter('w_roughness').value
        self.w_u = self.get_parameter('w_unknown').value
        self.w_f = self.get_parameter('w_friction').value
        self.mu = self.get_parameter('surface_friction_coefficient').value
        self.min_swing = self.get_parameter('min_swing_height').value
        self.strict_unknown = self.get_parameter('strict_unknown').value

        self.create_subscription(GridMap, '/terrain_grid_map', self.grid_map_callback, 10)
        self.cost_pub = self.create_publisher(GridMap, '/terrain_costmap', 10)

    def grid_map_callback(self, msg: GridMap):
        layers = {name: self._extract_layer(msg, i) for i, name in enumerate(msg.layers)}

        floor = layers.get('floor')
        ceiling = layers.get('ceiling')
        clearance = layers.get('clearance')
        unknown_above = layers.get('unknown_above')

        if floor is None:
            return

        n = floor.shape[0]
        cost = np.zeros_like(floor)

        # --- Slope cost ---
        gx, gy = np.gradient(np.nan_to_num(floor, nan=0.0), self.get_res(msg))
        slope = np.sqrt(gx**2 + gy**2)
        slope_norm = slope / (slope.max() + 1e-6)
        cost += self.w_s * slope_norm

        # --- Roughness cost ---
        roughness = self._local_variance(floor, kernel=3)
        rough_norm = roughness / (roughness.max() + 1e-6)
        cost += self.w_r * rough_norm

        # --- Friction cost ---
        # Friction loss = 1 - mu * cos(slope_angle)
        # On flat ground with mu=0.8: loss = 1 - 0.8*1.0 = 0.2 (low)
        # On 45deg slope with mu=0.3: loss = 1 - 0.3*0.707 = 0.79 (high)
        slope_angle = np.arctan(slope)
        friction_factor = self.mu * np.cos(slope_angle)
        friction_loss = 1.0 - friction_factor
        cost += self.w_f * friction_loss

        # --- Clearance constraint ---
        if clearance is not None:
            too_low = clearance < self.min_swing
            cost[too_low] = np.inf

        # --- Unknown overhead penalty (continuous 0..1 fraction) ---
        if unknown_above is not None:
            if self.strict_unknown:
                # Any unknown overhead → INF cost
                has_unknown = unknown_above > 0.0
                cost[has_unknown] = np.inf
            else:
                # Continuous penalty proportional to unknown fraction
                cost += self.w_u * unknown_above

        # NaN floor = no terrain data = infinite cost
        cost[np.isnan(floor)] = np.inf

        self._publish_cost(cost, msg)

    def _local_variance(self, grid, kernel=3):
        """Compute local variance around each cell (vectorized)."""
        filled = np.nan_to_num(grid, nan=0.0)
        half = kernel // 2
        padded = np.pad(filled, half, mode='edge')
        h, w = grid.shape
        # Use stride_tricks for sliding window view (no copy on numpy >=1.20)
        from numpy.lib.stride_tricks import sliding_window_view
        windows = sliding_window_view(padded, (kernel, kernel))
        # windows shape: (h, w, kernel, kernel)
        return windows.reshape(h, w, -1).var(axis=2)

    def _extract_layer(self, msg: GridMap, index: int) -> np.ndarray:
        data = msg.data[index]
        n = msg.data[index].layout.dim[0].size
        arr = np.array(data.data, dtype=np.float32).reshape((n, n), order='F')
        return arr

    def get_res(self, msg: GridMap) -> float:
        return msg.info.resolution

    def _publish_cost(self, cost: np.ndarray, original: GridMap):
        gm = GridMap()
        gm.header = original.header
        gm.info = original.info
        gm.layers = ['cost']
        gm.data.append(make_layer(np.asarray(cost, dtype=np.float32)))
        self.cost_pub.publish(gm)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainCostNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()