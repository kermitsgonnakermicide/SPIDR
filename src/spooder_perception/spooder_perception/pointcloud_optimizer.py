#!/usr/bin/env python3
"""
Change-aware PointCloud2 optimizer.

Publishes a downsampled cloud only when the voxelized scene changes enough to
matter. This avoids repeatedly feeding identical depth frames to live consumers
while the pointcloud saver independently maintains the persistent map.
"""

import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class PointCloudOptimizer(Node):
    def __init__(self):
        super().__init__('pointcloud_optimizer')

        self.declare_parameter('input_topic', '/camera/points')
        self.declare_parameter('output_topic', '/camera/points/optimized')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('change_ratio', 0.02)
        self.declare_parameter('min_changed_voxels', 150)
        self.declare_parameter('min_publish_interval', 0.2)
        self.declare_parameter('force_publish_interval', 0.0)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('max_output_points', 120000)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.change_ratio = float(self.get_parameter('change_ratio').value)
        self.min_changed_voxels = int(self.get_parameter('min_changed_voxels').value)
        self.min_publish_interval = float(self.get_parameter('min_publish_interval').value)
        self.force_publish_interval = float(self.get_parameter('force_publish_interval').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.max_output_points = int(self.get_parameter('max_output_points').value)

        self.last_published_keys = None
        self.last_publish_time = 0.0
        self.raw_clouds_seen = 0
        self.clouds_published = 0
        self.clouds_skipped = 0

        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, output_qos)

        self.get_logger().info('Point Cloud Optimizer initialized')
        self.get_logger().info(f'  Input topic: {self.input_topic}')
        self.get_logger().info(f'  Output topic: {self.output_topic}')
        self.get_logger().info(f'  Voxel size: {self.voxel_size}m')
        self.get_logger().info(
            f'  Change threshold: {self.min_changed_voxels} voxels and '
            f'{self.change_ratio:.1%}'
        )

    def pointcloud_callback(self, msg):
        self.raw_clouds_seen += 1

        points = self.pointcloud2_to_numpy(msg)
        if points is None or len(points) == 0:
            return

        points = self.filter_points(points)
        if len(points) == 0:
            return

        voxel_coords, indices = self.downsample_to_voxels(points)
        if len(voxel_coords) == 0:
            return

        keys = self.voxel_keys(voxel_coords)
        now = time.monotonic()

        should_publish, changed_voxels, ratio, reason = self.should_publish(keys, now)
        if not should_publish:
            self.clouds_skipped += 1
            self.get_logger().debug(
                f'Skipped unchanged cloud ({changed_voxels} changed, {ratio:.3f})'
            )
            return

        if self.min_publish_interval > 0.0 and now - self.last_publish_time < self.min_publish_interval:
            return

        optimized_points = points[indices]
        if len(optimized_points) > self.max_output_points:
            sample_idx = np.linspace(0, len(optimized_points) - 1, self.max_output_points, dtype=np.int64)
            optimized_points = optimized_points[sample_idx]

        self.publisher.publish(self.numpy_to_pointcloud2(msg.header, optimized_points))
        self.last_published_keys = keys.copy()
        self.last_publish_time = now
        self.clouds_published += 1

        self.get_logger().info(
            f'Published optimized cloud ({len(optimized_points)} points, '
            f'{changed_voxels} changed voxels, {ratio:.1%}, {reason})',
            throttle_duration_sec=2.0
        )

    def filter_points(self, points):
        distances = np.linalg.norm(points, axis=1)
        mask = (
            np.isfinite(points).all(axis=1) &
            (distances >= self.range_min) &
            (distances <= self.range_max)
        )
        return points[mask]

    def downsample_to_voxels(self, points):
        voxel_coords = np.floor(points / self.voxel_size).astype(np.int32)
        unique_voxels, indices = np.unique(voxel_coords, axis=0, return_index=True)
        return unique_voxels, indices

    @staticmethod
    def voxel_keys(voxel_coords):
        contiguous = np.ascontiguousarray(voxel_coords)
        key_dtype = np.dtype((np.void, contiguous.dtype.itemsize * contiguous.shape[1]))
        return contiguous.view(key_dtype).reshape(-1)

    def should_publish(self, keys, now):
        if self.last_published_keys is None:
            return True, len(keys), 1.0, 'initial'

        if self.force_publish_interval > 0.0 and now - self.last_publish_time >= self.force_publish_interval:
            return True, 0, 0.0, 'forced'

        added = np.setdiff1d(keys, self.last_published_keys, assume_unique=True)
        removed = np.setdiff1d(self.last_published_keys, keys, assume_unique=True)
        changed_voxels = len(added) + len(removed)
        ratio = changed_voxels / max(len(keys), len(self.last_published_keys), 1)

        enough_absolute_change = changed_voxels >= self.min_changed_voxels
        enough_relative_change = ratio >= self.change_ratio
        return (
            enough_absolute_change and enough_relative_change,
            changed_voxels,
            ratio,
            'changed'
        )

    def pointcloud2_to_numpy(self, cloud_msg):
        try:
            fields = {field.name: field for field in cloud_msg.fields}
            if not all(field_name in fields for field_name in ('x', 'y', 'z')):
                self.get_logger().warn('PointCloud2 missing x/y/z fields', throttle_duration_sec=5.0)
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
                    throttle_duration_sec=5.0
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
        msg.header = header
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
    node = PointCloudOptimizer()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
