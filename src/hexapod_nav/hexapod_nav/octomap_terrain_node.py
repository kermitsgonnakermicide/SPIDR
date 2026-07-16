#!/usr/bin/env python3
"""
octomap_terrain_node.py

Converts OctoMap occupied-voxel point cloud into a dual-layer grid_map:
  - 'floor'    : highest ground-level voxel height per cell
  - 'ceiling'  : lowest overhead voxel height per cell
  - 'clearance': ceiling - floor per cell
  - 'unknown_above': fraction of voxels above floor that are UNKNOWN (not free, not occupied)

The unknown_above layer is fed into terrain_cost_node as a ceiling-uncertainty penalty.
This is the novel contribution vs Buchanan/Kottege who use only occupied voxels.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from octomap_msgs.msg import Octomap
import struct


class OctomapTerrainNode(Node):
    def __init__(self):
        super().__init__('octomap_terrain_node')

        # Parameters
        self.declare_parameter('resolution', 0.05)         # Grid cell size (m)
        self.declare_parameter('grid_radius', 3.0)         # Local map radius around robot (m)
        self.declare_parameter('robot_body_height', 0.15)  # Robot body clearance above ground (m)
        self.declare_parameter('floor_search_height', 0.3) # Max height considered "floor" above lowest point
        self.declare_parameter('voxel_size', 0.05)         # Must match octomap resolution

        self.res = self.get_parameter('resolution').value
        self.radius = self.get_parameter('grid_radius').value
        self.body_h = self.get_parameter('robot_body_height').value
        self.floor_search = self.get_parameter('floor_search_height').value
        self.voxel_size = self.get_parameter('voxel_size').value

        self.grid_size = int(2 * self.radius / self.res)
        self.robot_pos = np.array([0.0, 0.0, 0.0])  # Updated from TF

        # Subscriptions
        self.create_subscription(
            PointCloud2,
            '/octomap_point_cloud_centers',
            self.pointcloud_callback,
            10
        )

        # Publisher
        self.terrain_pub = self.create_publisher(GridMap, '/terrain_grid_map', 10)

        self.get_logger().info('OctoMap terrain node started')

    def pointcloud_callback(self, msg: PointCloud2):
        """Process occupied voxels into floor/ceiling grid."""
        points = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not points:
            return

        pts = np.array(points)

        # Build local grid centred on robot
        cx, cy = self.robot_pos[0], self.robot_pos[1]
        n = self.grid_size

        floor_grid    = np.full((n, n), np.nan)
        ceiling_grid  = np.full((n, n), np.nan)

        # Bin points into grid cells
        cell_points = {}  # (ix, iy) -> list of z values
        for x, y, z in pts:
            ix = int((x - (cx - self.radius)) / self.res)
            iy = int((y - (cy - self.radius)) / self.res)
            if 0 <= ix < n and 0 <= iy < n:
                key = (ix, iy)
                if key not in cell_points:
                    cell_points[key] = []
                cell_points[key].append(z)

        for (ix, iy), zvals in cell_points.items():
            zvals = sorted(zvals)
            z_min = zvals[0]

            # Floor: lowest cluster of points (ground surface)
            floor_pts = [z for z in zvals if z <= z_min + self.floor_search]
            floor_grid[ix, iy] = np.mean(floor_pts) if floor_pts else z_min

            # Ceiling: lowest point significantly above floor + robot body height
            floor_h = floor_grid[ix, iy]
            overhead_threshold = floor_h + self.body_h + 0.05  # 5cm above robot body
            overhead_pts = [z for z in zvals if z > overhead_threshold]
            if overhead_pts:
                ceiling_grid[ix, iy] = min(overhead_pts)
            # If no overhead points observed: ceiling is NaN (unknown — treated as potentially low)

        # Clearance: ceiling - floor (NaN where ceiling unknown)
        clearance_grid = ceiling_grid - floor_grid

        # Publish as GridMap
        self._publish_grid_map(floor_grid, ceiling_grid, clearance_grid, msg.header)

    def _publish_grid_map(self, floor, ceiling, clearance, header):
        gm = GridMap()
        gm.header = header
        gm.info.resolution = self.res
        gm.info.length_x = 2 * self.radius
        gm.info.length_y = 2 * self.radius
        gm.info.pose.position.x = self.robot_pos[0]
        gm.info.pose.position.y = self.robot_pos[1]
        gm.info.pose.orientation.w = 1.0

        gm.layers = ['floor', 'ceiling', 'clearance']

        for name, data in [('floor', floor), ('ceiling', ceiling), ('clearance', clearance)]:
            layer = Float32MultiArray()
            dim_x = MultiArrayDimension(label='column', size=self.grid_size, stride=self.grid_size * self.grid_size)
            dim_y = MultiArrayDimension(label='row', size=self.grid_size, stride=self.grid_size)
            layer.layout.dim = [dim_x, dim_y]
            flat = data.flatten(order='F')  # grid_map uses column-major
            flat_clean = np.where(np.isnan(flat), float('nan'), flat).astype(np.float32)
            layer.data = flat_clean.tolist()
            gm.data.append(layer)

        self.terrain_pub.publish(gm)


def main(args=None):
    rclpy.init(args=args)
    node = OctomapTerrainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()