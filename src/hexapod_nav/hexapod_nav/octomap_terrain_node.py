#!/usr/bin/env python3
"""
octomap_terrain_node.py

Converts OctoMap occupied-voxel point cloud into a dual-layer grid_map:
  - 'floor'    : highest ground-level voxel height per cell
  - 'ceiling'  : lowest overhead voxel height per cell
  - 'clearance': ceiling - floor per cell
  - 'unknown_above': fraction of voxels above floor that are UNKNOWN

Tilt-aware: filters points relative to robot height so stale data from
when the robot was at a different elevation / orientation does not persist.
GridMap pose includes robot yaw so RViz renders the grid correctly oriented.
"""

import math
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from grid_map_msgs.msg import GridMap

from .grid_map_utils import make_layer
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener


class OctomapTerrainNode(Node):
    def __init__(self):
        super().__init__('octomap_terrain_node')

        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('grid_radius', 3.0)
        self.declare_parameter('robot_body_height', 0.15)
        self.declare_parameter('floor_search_height', 0.3)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('base_frame', 'spooder/base_footprint')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('z_margin', 0.4)
        self.declare_parameter('stale_frames', 5)

        self.res = self.get_parameter('resolution').value
        self.radius = self.get_parameter('grid_radius').value
        self.body_h = self.get_parameter('robot_body_height').value
        self.floor_search = self.get_parameter('floor_search_height').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.base_frame = self.get_parameter('base_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.z_margin = self.get_parameter('z_margin').value
        self.stale_frames = self.get_parameter('stale_frames').value

        self.grid_size = int(2 * self.radius / self.res)
        self.robot_pos = np.array([0.0, 0.0, 0.0])
        self.robot_yaw = 0.0

        # Per-cell freshness: how many frames since last update
        self._cell_age = np.full((self.grid_size, self.grid_size),
                                 self.stale_frames + 1, dtype=np.int32)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PointCloud2,
            '/octomap_point_cloud_centers',
            self.pointcloud_callback,
            10
        )

        self.terrain_pub = self.create_publisher(GridMap, '/terrain_grid_map', 10)

        self.get_logger().info('OctoMap terrain node started (tilt-aware)')

    def _update_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.base_frame, rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            self.robot_pos = np.array([t.x, t.y, t.z])
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny, cosy)
        except Exception:
            pass

    def pointcloud_callback(self, msg: PointCloud2):
        self._update_robot_pose()

        points = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.stack([points['x'], points['y'], points['z']], axis=1)
        if len(pts) == 0:
            return

        dx = pts[:, 0] - self.robot_pos[0]
        dy = pts[:, 1] - self.robot_pos[1]
        dz = pts[:, 2] - self.robot_pos[2]

        h_dist_sq = dx * dx + dy * dy
        mask_h = h_dist_sq <= self.radius * self.radius

        z_range = self.body_h + self.floor_search + self.z_margin
        mask_z = np.abs(dz) <= z_range

        keep = mask_h & mask_z
        pts = pts[keep]

        if len(pts) == 0:
            self._age_and_publish()
            return

        n = self.grid_size
        cx, cy = self.robot_pos[0], self.robot_pos[1]

        ix_arr = np.floor((pts[:, 0] - (cx - self.radius)) / self.res).astype(np.int32)
        iy_arr = np.floor((pts[:, 1] - (cy - self.radius)) / self.res).astype(np.int32)
        valid = (ix_arr >= 0) & (ix_arr < n) & (iy_arr >= 0) & (iy_arr < n)
        ix_arr = ix_arr[valid]
        iy_arr = iy_arr[valid]
        z_arr = pts[valid, 2]

        cell_ids = ix_arr * n + iy_arr
        order = np.argsort(cell_ids)
        cell_ids = cell_ids[order]
        z_arr = z_arr[order]
        unique_cells, starts, counts = np.unique(cell_ids, return_index=True, return_counts=True)

        floor_grid = np.full((n, n), np.nan)
        ceiling_grid = np.full((n, n), np.nan)
        unknown_grid = np.full((n, n), 1.0)
        updated_mask = np.zeros((n, n), dtype=bool)

        column_voxels = max(int((self.body_h + 0.1) / self.voxel_size), 1)

        for k in range(len(unique_cells)):
            cell = int(unique_cells[k])
            ix_k = cell // n
            iy_k = cell % n
            zs = np.sort(z_arr[starts[k]:starts[k] + counts[k]])
            z_min = zs[0]

            floor_mask = zs <= z_min + self.floor_search
            floor_h = float(np.mean(zs[floor_mask]))
            floor_grid[ix_k, iy_k] = floor_h

            overhead_threshold = floor_h + self.body_h + 0.05
            overhead = zs[zs > overhead_threshold]
            if len(overhead) > 0:
                ceiling_grid[ix_k, iy_k] = float(overhead[0])

            col_top = floor_h + self.body_h + 0.1
            col_bot = floor_h + self.body_h + 0.05
            overhead_count = int(np.sum((zs > col_bot) & (zs <= col_top)))
            unknown_grid[ix_k, iy_k] = max(0.0, 1.0 - overhead_count / column_voxels)

            updated_mask[ix_k, iy_k] = True

        self._cell_age[~updated_mask] += 1
        self._cell_age[updated_mask] = 0
        stale = self._cell_age > self.stale_frames
        floor_grid[stale] = np.nan
        ceiling_grid[stale] = np.nan
        unknown_grid[stale] = 1.0

        clearance_grid = ceiling_grid - floor_grid

        self._publish_grid_map(floor_grid, ceiling_grid, clearance_grid,
                               unknown_grid, msg.header)

    def _age_and_publish(self):
        self._cell_age += 1
        stale = self._cell_age > self.stale_frames
        n = self.grid_size
        floor_grid = np.full((n, n), np.nan)
        ceiling_grid = np.full((n, n), np.nan)
        clearance_grid = np.full((n, n), np.nan)
        unknown_grid = np.full((n, n), 1.0)
        floor_grid[stale] = np.nan
        self._publish_grid_map(floor_grid, ceiling_grid, clearance_grid,
                               unknown_grid, None)

    def _publish_grid_map(self, floor, ceiling, clearance, unknown_above, header):
        gm = GridMap()
        if header is not None:
            gm.header = header
        gm.header.frame_id = self.world_frame
        gm.info.resolution = self.res
        gm.info.length_x = 2 * self.radius
        gm.info.length_y = 2 * self.radius
        gm.info.pose.position.x = self.robot_pos[0]
        gm.info.pose.position.y = self.robot_pos[1]

        qz = math.sin(self.robot_yaw / 2.0)
        qw = math.cos(self.robot_yaw / 2.0)
        gm.info.pose.orientation.z = qz
        gm.info.pose.orientation.w = qw

        gm.layers = ['floor', 'ceiling', 'clearance', 'unknown_above']

        floor = floor[::-1, ::-1]
        ceiling = ceiling[::-1, ::-1]
        clearance = clearance[::-1, ::-1]
        unknown_above = unknown_above[::-1, ::-1]

        for name, data in [('floor', floor), ('ceiling', ceiling),
                           ('clearance', clearance), ('unknown_above', unknown_above)]:
            flat_clean = data.astype(np.float32)
            gm.data.append(make_layer(flat_clean))

        self.terrain_pub.publish(gm)


def main(args=None):
    rclpy.init(args=args)
    node = OctomapTerrainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
