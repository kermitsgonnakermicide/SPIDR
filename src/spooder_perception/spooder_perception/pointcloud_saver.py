#!/usr/bin/env python3
"""
Point Cloud Saver Node for SPIDR
Accumulates depth camera point clouds and exports to PCD format
"""

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import numpy as np
import struct
import os
from datetime import datetime
from collections import defaultdict


class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        
        # Parameters
        self.declare_parameter('voxel_size', 0.05) # Increased for performance
        self.declare_parameter('dist_threshold', 0.01) # Replace if changed by 1cm
        self.declare_parameter('max_points', 1000000)
        self.declare_parameter('auto_save_interval', 0.0)
        self.declare_parameter('output_directory', os.path.expanduser('~/spooder_maps'))
        self.declare_parameter('map_frame', 'map')
        
        self.voxel_size = self.get_parameter('voxel_size').value
        self.dist_threshold = self.get_parameter('dist_threshold').value
        self.max_points = self.get_parameter('max_points').value
        self.auto_save_interval = self.get_parameter('auto_save_interval').value
        self.output_dir = os.path.expanduser(str(self.get_parameter('output_directory').value))
        self.map_frame = self.get_parameter('map_frame').value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Voxel grid for accumulation (using dict for sparse storage)
        self.voxel_grid = defaultdict(lambda: {'xyz': None, 'rgb': None, 'count': 0})
        self.total_points = 0
        
        # TF buffer for transforming clouds
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.pointcloud_callback,
            10
        )
        
        # Service
        self.save_service = self.create_service(
            Trigger,
            'save_pointcloud',
            self.save_callback
        )
        
        # Auto-save timer
        if self.auto_save_interval > 0.0:
            self.auto_save_timer = self.create_timer(
                self.auto_save_interval,
                self.auto_save_callback
            )
        
        self.get_logger().info(f'Point Cloud Saver initialized')
        self.get_logger().info(f'  Voxel size: {self.voxel_size}m')
        self.get_logger().info(f'  Max points: {self.max_points}')
        self.get_logger().info(f'  Output dir: {self.output_dir}')
        self.get_logger().info(f'  Map frame: {self.map_frame}')
    
    def voxel_key(self, x, y, z):
        """Convert coordinates to voxel grid key"""
        return (
            int(np.floor(x / self.voxel_size)),
            int(np.floor(y / self.voxel_size)),
            int(np.floor(z / self.voxel_size))
        )
    
    def pointcloud_callback(self, msg):
        """Accumulate point cloud data"""
        try:
            if not msg.header.frame_id:
                self.get_logger().warn('Skipping point cloud with empty frame_id', throttle_duration_sec=5.0)
                return

            points, rgbs = self.pointcloud2_to_numpy(msg)
            if points is None or len(points) == 0:
                return

            if msg.header.frame_id != self.map_frame:
                try:
                    transform = self.lookup_transform(
                        self.map_frame,
                        msg.header.frame_id,
                        msg.header.stamp
                    )
                    points = self.transform_points(points, transform)
                except Exception as e:
                    self.get_logger().warn(f'Transform failed: {e}', throttle_duration_sec=5.0)
                    return

            points_added = 0
            for i in range(len(points)):
                x, y, z = points[i]
                rgb = rgbs[i]
                
                # Get voxel key
                key = self.voxel_key(x, y, z)
                
                # Add or update voxel
                voxel = self.voxel_grid[key]
                if voxel['xyz'] is None:
                    # New voxel
                    voxel['xyz'] = np.array([x, y, z])
                    voxel['rgb'] = rgb
                    voxel['count'] = 1
                    points_added += 1
                    self.total_points += 1
                else:
                    # OPTIMIZATION: "Replace if changed" logic
                    # Calculate distance from existing voxel centroid
                    dist = np.linalg.norm(voxel['xyz'] - np.array([x, y, z]))
                    if dist > self.dist_threshold:
                        # Significant change, update voxel (moving average)
                        count = voxel['count']
                        voxel['xyz'] = (voxel['xyz'] * count + np.array([x, y, z])) / (count + 1)
                        voxel['count'] += 1
                        # We don't increment total_points as it's an update
                    # Else: skip, the point hasn't "changed" enough to justify processing
                
                # Check max points limit
                if self.total_points >= self.max_points:
                    self.get_logger().warn('Max points reached, stopping accumulation', throttle_duration_sec=10.0)
                    break
            
            if points_added > 0:
                self.get_logger().info(
                    f'Accumulated {points_added} new voxels (total: {self.total_points})',
                    throttle_duration_sec=5.0
                )
        
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def lookup_transform(self, target_frame, source_frame, stamp):
        """Look up a transform, falling back to the latest TF if stamped TF lags."""
        lookup_time = Time.from_msg(stamp)
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                lookup_time,
                timeout=Duration(seconds=0.2)
            )
        except Exception:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.2)
            )

    def transform_points(self, points, transform):
        """Apply a TransformStamped to an Nx3 point array without native cloud helpers."""
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        rot_mat = self.quaternion_to_rotation_matrix(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w
        )
        offset = np.array([translation.x, translation.y, translation.z], dtype=np.float32)
        return np.dot(points, rot_mat.T) + offset

    @staticmethod
    def quaternion_to_rotation_matrix(x, y, z, w):
        quat = np.array([x, y, z, w], dtype=np.float64)
        norm = np.linalg.norm(quat)
        if norm < 1e-12:
            return np.eye(3, dtype=np.float32)

        x, y, z, w = quat / norm
        return np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ], dtype=np.float32)

    def pointcloud2_to_numpy(self, cloud_msg):
        """Robustlly extract XYZ and RGB from PointCloud2 without dtype mismatches"""
        try:
            fields = {field.name: field for field in cloud_msg.fields}
            if not all(field_name in fields for field_name in ('x', 'y', 'z')):
                return None, None
            
            point_step = cloud_msg.point_step
            num_points = cloud_msg.width * cloud_msg.height
            if num_points == 0 or point_step == 0:
                return np.empty((0, 3), dtype=np.float32), np.empty(0, dtype=np.uint32)

            raw = np.frombuffer(cloud_msg.data, dtype=np.uint8)
            expected_size = num_points * point_step
            if raw.size < expected_size:
                self.get_logger().warn(
                    f'PointCloud2 data is shorter than expected: {raw.size} < {expected_size}',
                    throttle_duration_sec=5.0
                )
                return None, None

            data = raw[:expected_size].reshape(num_points, point_step)
            float_dtype = np.dtype('>f4' if cloud_msg.is_bigendian else '<f4')
            uint_dtype = np.dtype('>u4' if cloud_msg.is_bigendian else '<u4')
            
            off_x = fields['x'].offset
            off_y = fields['y'].offset
            off_z = fields['z'].offset
            
            xyz = np.zeros((num_points, 3), dtype=np.float32)
            xyz[:, 0] = data[:, off_x:off_x + 4].copy().view(float_dtype).reshape(-1)
            xyz[:, 1] = data[:, off_y:off_y + 4].copy().view(float_dtype).reshape(-1)
            xyz[:, 2] = data[:, off_z:off_z + 4].copy().view(float_dtype).reshape(-1)
            
            rgb_field = 'rgb' if 'rgb' in fields else 'rgba' if 'rgba' in fields else None
            if rgb_field is not None:
                off_rgb = fields[rgb_field].offset
                rgbs = data[:, off_rgb:off_rgb + 4].copy().view(uint_dtype).reshape(-1)
            else:
                rgbs = np.zeros(num_points, dtype=np.uint32)
            
            mask = np.isfinite(xyz).all(axis=1)
            return xyz[mask], rgbs[mask]
        except Exception as e:
            self.get_logger().warn(f"Manual parsing failed: {e}")
            return None, None
    
    def save_callback(self, request, response):
        """Service callback to save point cloud"""
        try:
            filename = self.save_pointcloud()
            response.success = True
            response.message = f'Saved {self.total_points} points to {filename}'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Failed to save: {e}'
            self.get_logger().error(response.message)
        
        return response
    
    def auto_save_callback(self):
        """Auto-save timer callback"""
        if self.total_points > 0:
            try:
                filename = self.save_pointcloud()
                self.get_logger().info(f'Auto-saved {self.total_points} points to {filename}')
            except Exception as e:
                self.get_logger().error(f'Auto-save failed: {e}')
    
    def save_pointcloud(self):
        """Save accumulated point cloud to PCD file"""
        if self.total_points == 0:
            raise ValueError('No points to save')
        
        # Generate filename
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.output_dir, f'spooder_map_{timestamp}.pcd')
        
        # Collect all points
        points = []
        for voxel in self.voxel_grid.values():
            if voxel['xyz'] is not None:
                x, y, z = voxel['xyz']
                rgb = voxel['rgb']
                
                # Unpack RGB
                if isinstance(rgb, float):
                    rgb_int = struct.unpack('I', struct.pack('f', rgb))[0]
                else:
                    rgb_int = int(rgb)
                rgb_int &= 0x00FFFFFF
                
                r = (rgb_int >> 16) & 0xFF
                g = (rgb_int >> 8) & 0xFF
                b = rgb_int & 0xFF
                
                points.append([x, y, z, r, g, b])
        
        points = np.array(points)
        
        # Write PCD file (ASCII format for compatibility)
        with open(filename, 'w', encoding='utf-8') as f:
            # Header
            f.write('# .PCD v0.7 - Point Cloud Data file format\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z rgb\n')
            f.write('SIZE 4 4 4 4\n')
            f.write('TYPE F F F U\n')
            f.write('COUNT 1 1 1 1\n')
            f.write(f'WIDTH {len(points)}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(points)}\n')
            f.write('DATA ascii\n')
            
            # Data
            for point in points:
                x, y, z, r, g, b = point
                # Pack RGB back to single uint32
                rgb_packed = (int(r) << 16) | (int(g) << 8) | int(b)
                f.write(f'{x:.6f} {y:.6f} {z:.6f} {rgb_packed}\n')
        
        return filename


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    
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
