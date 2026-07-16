#!/usr/bin/env python3
"""
terrain_cost_node.py

Computes foothold cost per grid cell from terrain properties:
  - Slope cost       (gradient of floor height)
  - Roughness cost   (local variance of floor height)
  - Clearance cost   (ceiling - floor; hard-zero below swing threshold)
  - Unknown penalty  (NaN ceiling = uncertain overhead = cost penalty)

Cost formula:
  cost(x,y) = w_s * slope(x,y) + w_r * roughness(x,y) + w_u * unknown(x,y)
  subject to: cost = INF if clearance(x,y) < min_swing_height OR clearance(x,y) is NaN and use_strict_unknown=True
"""

import rclpy
from rclpy.node import Node
import numpy as np
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class TerrainCostNode(Node):
    def __init__(self):
        super().__init__('terrain_cost_node')

        # Weights (tunable — these become your experimental variables)
        self.declare_parameter('w_slope', 1.0)
        self.declare_parameter('w_roughness', 0.8)
        self.declare_parameter('w_unknown', 0.5)         # Penalty for NaN/unknown ceiling
        self.declare_parameter('min_swing_height', 0.12) # Minimum clearance for leg swing (m)
        self.declare_parameter('strict_unknown', True)   # Treat NaN ceiling as INFINITE cost

        self.w_s = self.get_parameter('w_slope').value
        self.w_r = self.get_parameter('w_roughness').value
        self.w_u = self.get_parameter('w_unknown').value
        self.min_swing = self.get_parameter('min_swing_height').value
        self.strict_unknown = self.get_parameter('strict_unknown').value

        self.create_subscription(GridMap, '/terrain_grid_map', self.grid_map_callback, 10)
        self.cost_pub = self.create_publisher(GridMap, '/terrain_costmap', 10)

    def grid_map_callback(self, msg: GridMap):
        layers = {name: self._extract_layer(msg, i) for i, name in enumerate(msg.layers)}

        floor = layers.get('floor')
        ceiling = layers.get('ceiling')
        clearance = layers.get('clearance')

        if floor is None:
            return

        n = floor.shape[0]
        cost = np.zeros_like(floor)

        # --- Slope cost ---
        # Gradient magnitude of floor height
        gx, gy = np.gradient(np.nan_to_num(floor, nan=0.0), self.get_res(msg))
        slope = np.sqrt(gx**2 + gy**2)
        slope_norm = slope / (slope.max() + 1e-6)
        cost += self.w_s * slope_norm

        # --- Roughness cost ---
        # Local variance using 3x3 kernel
        roughness = self._local_variance(floor, kernel=3)
        rough_norm = roughness / (roughness.max() + 1e-6)
        cost += self.w_r * rough_norm

        # --- Clearance constraint ---
        if clearance is not None:
            # Hard constraint: if clearance < min_swing, cost = INF
            too_low = clearance < self.min_swing
            cost[too_low] = np.inf

            # Unknown ceiling penalty
            unknown_mask = np.isnan(ceiling) if ceiling is not None else np.zeros_like(floor, dtype=bool)
            if self.strict_unknown:
                cost[unknown_mask] = np.inf
            else:
                cost[unknown_mask] += self.w_u

        # NaN floor = no terrain data = infinite cost
        cost[np.isnan(floor)] = np.inf

        self._publish_cost(cost, msg)

    def _local_variance(self, grid, kernel=3):
        """Compute local variance around each cell."""
        padded = np.pad(np.nan_to_num(grid, nan=0.0), kernel//2, mode='edge')
        result = np.zeros_like(grid)
        half = kernel // 2
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                patch = padded[i:i+kernel, j:j+kernel]
                result[i, j] = np.var(patch)
        return result

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

        layer = Float32MultiArray()
        n = cost.shape[0]
        dim_x = MultiArrayDimension(label='column', size=n, stride=n*n)
        dim_y = MultiArrayDimension(label='row', size=n, stride=n)
        layer.layout.dim = [dim_x, dim_y]
        flat = cost.flatten(order='F').astype(np.float32)
        layer.data = flat.tolist()
        gm.data.append(layer)

        self.cost_pub.publish(gm)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainCostNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()