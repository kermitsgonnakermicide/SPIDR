#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
import numpy as np
import struct
import math

class TerrainAnalyzer(Node):
    def __init__(self):
        super().__init__('terrain_analyzer')
        
        # Parameters
        self.declare_parameter('roi_x_min', 0.2)
        self.declare_parameter('roi_x_max', 0.8)
        self.declare_parameter('roi_y_min', -0.3)
        self.declare_parameter('roi_y_max', 0.3)
        self.declare_parameter('floor_threshold', 0.05)
        self.declare_parameter('max_climbable_height', 0.25)
        
        self.roi_x_min = self.get_parameter('roi_x_min').value
        self.roi_x_max = self.get_parameter('roi_x_max').value
        self.roi_y_min = self.get_parameter('roi_y_min').value
        self.roi_y_max = self.get_parameter('roi_y_max').value
        self.floor_threshold = self.get_parameter('floor_threshold').value
        self.max_climbable_height = self.get_parameter('max_climbable_height').value
        
        # Subscription
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.pointcloud_callback,
            10
        )
        
        # Publisher
        self.height_pub = self.create_publisher(Float32, '/perception/terrain_height', 10)
        self.debug_pc_pub = self.create_publisher(PointCloud2, '/perception/debug_terrain_pc', 10)
        
        self.get_logger().info("3D Terrain Analyzer Initialized")

    def pointcloud_callback(self, msg):
        """Analyze point cloud to find maximum terrain height in front of robot"""
        
        # Convert PointCloud2 to numpy array
        # Note: This is a bit slow for large point clouds, but good for prototyping
        # For production, numba or cython would be better
        
        points = self.pointcloud2_to_array(msg)
        if points is None or len(points) == 0:
            return

        # ROI Filtering
        # Points are in camera frame (usually optical) or transformed frame?
        # Assuming the camera is already correctly oriented (Z is up, X is forward)
        # based on our previous fixes to the depth camera orientation.
        
        mask = (points[:, 0] > self.roi_x_min) & (points[:, 0] < self.roi_x_max) & \
               (points[:, 1] > self.roi_y_min) & (points[:, 1] < self.roi_y_max)
        
        roi_points = points[mask]
        
        if len(roi_points) == 0:
            height_msg = Float32()
            height_msg.data = 0.0
            self.height_pub.publish(height_msg)
            return

        # Height is in Z coordinate
        # Filter out points below the floor threshold
        climbable_mask = (roi_points[:, 2] > self.floor_threshold) & (roi_points[:, 2] < self.max_climbable_height)
        climbable_points = roi_points[climbable_mask]
        
        max_height = 0.0
        if len(climbable_points) > 0:
            max_height = float(np.percentile(climbable_points[:, 2], 95)) # Use 95th percentile to avoid outliers
        
        # Publish result
        height_msg = Float32()
        height_msg.data = max_height
        self.height_pub.publish(height_msg)
        
        # Optional: Publish debug point cloud of the ROI
        if self.debug_pc_pub.get_subscription_count() > 0:
            self.publish_debug_pc(msg.header, climbable_points if len(climbable_points) > 0 else roi_points)

    def pointcloud2_to_array(self, cloud_msg):
        """Simplified conversion of PointCloud2 to numpy array (X, Y, Z)"""
        # Optimized assuming float32 fields x, y, z
        field_names = [field.name for field in cloud_msg.fields]
        if not ('x' in field_names and 'y' in field_names and 'z' in field_names):
            return None
            
        # Get data directly from buffer
        dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
        # We need to skip extra fields if present
        point_step = cloud_msg.point_step
        data = np.frombuffer(cloud_msg.data, dtype=np.uint8)
        
        # Reshape to (num_points, point_step)
        num_points = cloud_msg.width * cloud_msg.height
        data = data.reshape(num_points, point_step)
        
        # Extract x, y, z bytes (first 12 bytes usually)
        xyz_data = data[:, :12].copy().view(dtype=np.float32).reshape(num_points, 3)
        
        # Filter out NaNs
        mask = ~np.isnan(xyz_data).any(axis=1)
        return xyz_data[mask]

    def publish_debug_pc(self, header, points):
        """Publish points for visualization in RViz"""
        # Simplified PointCloud2 creation for debug
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        
        from sensor_msgs.msg import PointField
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.data = points.astype(np.float32).tobytes()
        self.debug_pc_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TerrainAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
