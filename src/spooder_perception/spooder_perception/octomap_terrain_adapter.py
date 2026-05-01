#!/usr/bin/env python3
"""Convert OctoMap occupied cell centers into traversability and step height."""

import math
import threading

from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, Header
import tf2_ros


class OctomapTerrainAdapter(Node):
    def __init__(self):
        super().__init__('octomap_terrain_adapter')

        self.declare_parameter('input_topic', '/octomap_server/octomap_point_cloud_centers')
        self.declare_parameter('traversability_topic', '/terrain/traversability')
        self.declare_parameter('terrain_height_topic', '/perception/terrain_height')
        self.declare_parameter('debug_topic', '/perception/debug_terrain_pc')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'spooder/base_footprint')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width_m', 20.0)
        self.declare_parameter('height_m', 8.0)
        self.declare_parameter('publish_rate', 2.0)

        self.declare_parameter('floor_ignore_z', 0.04)
        self.declare_parameter('climbable_min_z', 0.05)
        self.declare_parameter('climbable_max_z', 0.20)
        self.declare_parameter('lethal_min_z', 0.22)
        self.declare_parameter('lethal_max_z', 0.38)
        self.declare_parameter('overhead_min_z', 0.40)
        self.declare_parameter('climbable_cost', 35)
        self.declare_parameter('lethal_cost', 100)

        self.declare_parameter('roi_x_min', 0.2)
        self.declare_parameter('roi_x_max', 0.8)
        self.declare_parameter('roi_y_min', -0.3)
        self.declare_parameter('roi_y_max', 0.3)

        self.input_topic = self.get_parameter('input_topic').value
        self.traversability_topic = self.get_parameter('traversability_topic').value
        self.terrain_height_topic = self.get_parameter('terrain_height_topic').value
        self.debug_topic = self.get_parameter('debug_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.resolution = float(self.get_parameter('resolution').value)
        self.width_m = float(self.get_parameter('width_m').value)
        self.height_m = float(self.get_parameter('height_m').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.floor_ignore_z = float(self.get_parameter('floor_ignore_z').value)
        self.climbable_min_z = float(self.get_parameter('climbable_min_z').value)
        self.climbable_max_z = float(self.get_parameter('climbable_max_z').value)
        self.lethal_min_z = float(self.get_parameter('lethal_min_z').value)
        self.lethal_max_z = float(self.get_parameter('lethal_max_z').value)
        self.overhead_min_z = float(self.get_parameter('overhead_min_z').value)
        self.climbable_cost = int(self.get_parameter('climbable_cost').value)
        self.lethal_cost = int(self.get_parameter('lethal_cost').value)

        self.roi_x_min = float(self.get_parameter('roi_x_min').value)
        self.roi_x_max = float(self.get_parameter('roi_x_max').value)
        self.roi_y_min = float(self.get_parameter('roi_y_min').value)
        self.roi_y_max = float(self.get_parameter('roi_y_max').value)

        self.width_cells = max(1, int(round(self.width_m / self.resolution)))
        self.height_cells = max(1, int(round(self.height_m / self.resolution)))
        self.latest_points = np.empty((0, 3), dtype=np.float32)
        self.latest_frame = self.map_frame
        self.latest_stamp = None
        self.lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.traversability_pub = self.create_publisher(
            OccupancyGrid,
            self.traversability_topic,
            map_qos,
        )
        self.height_pub = self.create_publisher(Float32, self.terrain_height_topic, 10)
        self.debug_pub = self.create_publisher(PointCloud2, self.debug_topic, 10)

        timer_period = 1.0 / publish_rate if publish_rate > 0.0 else 0.5
        self.timer = self.create_timer(timer_period, self.publish_outputs)

        self.get_logger().info('OctoMap terrain adapter initialized')
        self.get_logger().info(f'  Input: {self.input_topic}')
        self.get_logger().info(f'  Traversability: {self.traversability_topic}')

    def pointcloud_callback(self, msg):
        points = self.pointcloud2_to_numpy(msg)
        if points is None:
            return

        with self.lock:
            self.latest_points = points
            self.latest_frame = msg.header.frame_id or self.map_frame
            self.latest_stamp = msg.header.stamp

    def publish_outputs(self):
        with self.lock:
            points = self.latest_points.copy()
            source_frame = self.latest_frame

        now = self.get_clock().now().to_msg()
        robot_xy = self.lookup_robot_xy()
        if robot_xy is None:
            return

        map_points = self.points_in_frame(points, source_frame, self.map_frame)
        base_points = self.points_in_frame(points, source_frame, self.base_frame)
        if map_points is None or base_points is None:
            return

        self.traversability_pub.publish(self.make_grid(map_points, robot_xy, now))
        self.publish_terrain_height(base_points, map_points)
        self.publish_debug_cloud(base_points, map_points, now)

    def lookup_robot_xy(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception as exc:
            self.get_logger().warn(
                f'Waiting for {self.map_frame}->{self.base_frame} transform: {exc}',
                throttle_duration_sec=5.0,
            )
            return None

        translation = transform.transform.translation
        return float(translation.x), float(translation.y)

    def points_in_frame(self, points, source_frame, target_frame):
        if len(points) == 0:
            return points

        if source_frame == target_frame:
            return points

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception as exc:
            self.get_logger().warn(
                f'Waiting for {target_frame}->{source_frame} transform: {exc}',
                throttle_duration_sec=5.0,
            )
            return None

        return self.transform_points(points, transform)

    @staticmethod
    def transform_points(points, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 0.0:
            return points
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        rot = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ], dtype=np.float32)
        offset = np.array([translation.x, translation.y, translation.z], dtype=np.float32)
        return np.dot(points, rot.T) + offset

    def make_grid(self, points, robot_xy, stamp):
        origin_x = robot_xy[0] - self.width_m * 0.5
        origin_y = robot_xy[1] - self.height_m * 0.5
        data = np.zeros(self.width_cells * self.height_cells, dtype=np.int8)

        if len(points) > 0:
            x = points[:, 0]
            y = points[:, 1]
            z = points[:, 2]
            in_grid = (
                np.isfinite(points).all(axis=1) &
                (x >= origin_x) & (x < origin_x + self.width_m) &
                (y >= origin_y) & (y < origin_y + self.height_m) &
                (z > self.floor_ignore_z) &
                (z < self.overhead_min_z)
            )
            grid_points = points[in_grid]

            if len(grid_points) > 0:
                ix = np.floor((grid_points[:, 0] - origin_x) / self.resolution).astype(np.int32)
                iy = np.floor((grid_points[:, 1] - origin_y) / self.resolution).astype(np.int32)
                flat = iy * self.width_cells + ix
                z = grid_points[:, 2]

                lethal = flat[(z >= self.lethal_min_z) & (z <= self.lethal_max_z)]
                if len(lethal) > 0:
                    data[np.unique(lethal)] = self.lethal_cost

                climbable = flat[(z >= self.climbable_min_z) & (z <= self.climbable_max_z)]
                if len(climbable) > 0:
                    climbable_cells = np.unique(climbable)
                    climbable_cells = climbable_cells[
                        data[climbable_cells] < self.lethal_cost
                    ]
                    data[climbable_cells] = self.climbable_cost

        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = self.map_frame
        grid.info = MapMetaData()
        grid.info.map_load_time = stamp
        grid.info.resolution = self.resolution
        grid.info.width = self.width_cells
        grid.info.height = self.height_cells
        grid.info.origin = Pose()
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = data.tolist()
        return grid

    def publish_terrain_height(self, base_points, map_points):
        height = 0.0
        if len(base_points) > 0 and len(map_points) == len(base_points):
            roi = (
                np.isfinite(base_points).all(axis=1) &
                np.isfinite(map_points).all(axis=1) &
                (base_points[:, 0] >= self.roi_x_min) &
                (base_points[:, 0] <= self.roi_x_max) &
                (base_points[:, 1] >= self.roi_y_min) &
                (base_points[:, 1] <= self.roi_y_max) &
                (map_points[:, 2] >= self.climbable_min_z) &
                (map_points[:, 2] <= self.climbable_max_z)
            )
            roi_points = map_points[roi]
            if len(roi_points) > 0:
                height = float(np.percentile(roi_points[:, 2], 95))

        msg = Float32()
        msg.data = height
        self.height_pub.publish(msg)

    def publish_debug_cloud(self, base_points, map_points, stamp):
        if self.debug_pub.get_subscription_count() == 0:
            return

        points = np.empty((0, 3), dtype=np.float32)
        if len(base_points) > 0 and len(map_points) == len(base_points):
            in_roi = (
                np.isfinite(base_points).all(axis=1) &
                np.isfinite(map_points).all(axis=1) &
                (base_points[:, 0] >= self.roi_x_min) &
                (base_points[:, 0] <= self.roi_x_max) &
                (base_points[:, 1] >= self.roi_y_min) &
                (base_points[:, 1] <= self.roi_y_max) &
                (map_points[:, 2] > self.floor_ignore_z) &
                (map_points[:, 2] < self.overhead_min_z)
            )
            points = map_points[in_roi]

        header = Header()
        header.stamp = stamp
        header.frame_id = self.map_frame
        self.debug_pub.publish(self.numpy_to_pointcloud2(header, points))

    def pointcloud2_to_numpy(self, cloud_msg):
        try:
            fields = {field.name: field for field in cloud_msg.fields}
            if not all(name in fields for name in ('x', 'y', 'z')):
                self.get_logger().warn(
                    'PointCloud2 missing x/y/z fields',
                    throttle_duration_sec=5.0,
                )
                return None

            point_step = cloud_msg.point_step
            num_points = cloud_msg.width * cloud_msg.height
            if num_points == 0 or point_step == 0:
                return np.empty((0, 3), dtype=np.float32)

            raw = np.frombuffer(cloud_msg.data, dtype=np.uint8)
            expected_size = num_points * point_step
            if raw.size < expected_size:
                self.get_logger().warn(
                    f'PointCloud2 data is shorter than expected: {raw.size} < {expected_size}',
                    throttle_duration_sec=5.0,
                )
                return None

            data = raw[:expected_size].reshape(num_points, point_step)
            float_dtype = np.dtype('>f4' if cloud_msg.is_bigendian else '<f4')
            xyz = np.zeros((num_points, 3), dtype=np.float32)
            for column, field_name in enumerate(('x', 'y', 'z')):
                offset = fields[field_name].offset
                xyz[:, column] = data[:, offset:offset + 4].copy().view(float_dtype).reshape(-1)

            return xyz[np.isfinite(xyz).all(axis=1)]
        except Exception as exc:
            self.get_logger().warn(f'PointCloud2 parsing failed: {exc}', throttle_duration_sec=5.0)
            return None

    @staticmethod
    def numpy_to_pointcloud2(header, points):
        msg = PointCloud2()
        msg.header.stamp = header.stamp
        msg.header.frame_id = header.frame_id
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = points.astype(np.float32).tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = OctomapTerrainAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
