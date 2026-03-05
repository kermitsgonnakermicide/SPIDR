import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
import numpy as np
import tf2_ros
from rclpy.duration import Duration
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
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('change_threshold', 0.02)
        
        self.roi_x_min = self.get_parameter('roi_x_min').value
        self.roi_x_max = self.get_parameter('roi_x_max').value
        self.roi_y_min = self.get_parameter('roi_y_min').value
        self.roi_y_max = self.get_parameter('roi_y_max').value
        self.floor_threshold = self.get_parameter('floor_threshold').value
        self.max_climbable_height = self.get_parameter('max_climbable_height').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.change_threshold = self.get_parameter('change_threshold').value
        
        self.last_published_height = 0.0
        
        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.target_frame = 'spooder/base_footprint'
        
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
        
        self.get_logger().info("3D Terrain Analyzer (TF-Aware) Initialized")

    def pointcloud_callback(self, msg):
        """Analyze point cloud to find maximum terrain height in front of robot"""
        
        # 1. Transform PointCloud2 to robot base frame manually to avoid Jazzy bugs
        try:
            # Wait for transform to be available
            if not self.tf_buffer.can_transform(self.target_frame, msg.header.frame_id, msg.header.stamp, timeout=Duration(seconds=0.1)):
                return
                
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp
            )
            
            # Extract points
            points = self.pointcloud2_to_array(msg)
            if points is None or len(points) == 0:
                return
                
            # Apply TF transformation manually
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            
            # Convert quaternion to rotation matrix
            q = [rotation.x, rotation.y, rotation.z, rotation.w]
            r00 = 1 - 2 * (q[1]**2 + q[2]**2)
            r01 = 2 * (q[0]*q[1] - q[2]*q[3])
            r02 = 2 * (q[0]*q[2] + q[1]*q[3])
            r10 = 2 * (q[0]*q[1] + q[2]*q[3])
            r11 = 1 - 2 * (q[0]**2 + q[2]**2)
            r12 = 2 * (q[1]*q[2] - q[0]*q[3])
            r20 = 2 * (q[0]*q[2] - q[1]*q[3])
            r21 = 2 * (q[1]*q[2] + q[0]*q[3])
            r22 = 1 - 2 * (q[0]**2 + q[1]**2)
            
            rot_mat = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
            
            # Point-wise transform
            points = np.dot(points, rot_mat.T) + np.array([translation.x, translation.y, translation.z])
            
        except Exception as e:
            self.get_logger().warn(f"TF Transform failed: {str(e)}", throttle_duration_sec=5.0)
            return

        # 3. ROI Filtering (X=forward, Y=left, Z=up)
        mask = (points[:, 0] > self.roi_x_min) & (points[:, 0] < self.roi_x_max) & \
               (points[:, 1] > self.roi_y_min) & (points[:, 1] < self.roi_y_max)
        
        roi_points = points[mask]
        
        if len(roi_points) == 0:
            if abs(self.last_published_height) > 0.001:
                height_msg = Float32()
                height_msg.data = 0.0
                if rclpy.ok():
                    self.height_pub.publish(height_msg)
                self.last_published_height = 0.0
            return

        # OPTIMIZATION: Simple Voxel Downsampling within ROI
        # Round to nearest voxel_size
        voxel_coords = np.round(roi_points / self.voxel_size) * self.voxel_size
        _, indices = np.unique(voxel_coords, axis=0, return_index=True)
        roi_points = roi_points[indices]

        # 4. Height calculation (Z is absolute height from floor)
        climbable_mask = (roi_points[:, 2] > self.floor_threshold) & (roi_points[:, 2] < self.max_climbable_height)
        climbable_points = roi_points[climbable_mask]
        
        max_height = 0.0
        if len(climbable_points) > 5: # Lowered threshold since we downsampled
            max_height = float(np.percentile(climbable_points[:, 2], 95))
        
        # Change detection: only publish if changed significantly
        if abs(max_height - self.last_published_height) > self.change_threshold:
            height_msg = Float32()
            height_msg.data = max_height
            if rclpy.ok():
                self.height_pub.publish(height_msg)
            self.last_published_height = max_height
        
        # Optional: Publish debug point cloud
        if self.debug_pc_pub.get_subscription_count() > 0:
            debug_header = msg.header
            debug_header.frame_id = self.target_frame
            self.publish_debug_pc(debug_header, climbable_points if len(climbable_points) > 0 else roi_points)

    def pointcloud2_to_array(self, cloud_msg):
        """Robustlly extract XYZ points from PointCloud2 without dtype mismatches"""
        try:
            field_indices = {field.name: i for i, field in enumerate(cloud_msg.fields)}
            if not all(f in field_indices for f in ('x', 'y', 'z')):
                return None
            
            point_step = cloud_msg.point_step
            num_points = cloud_msg.width * cloud_msg.height
            data = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(num_points, point_step)
            
            off_x = cloud_msg.fields[field_indices['x']].offset
            off_y = cloud_msg.fields[field_indices['y']].offset
            off_z = cloud_msg.fields[field_indices['z']].offset
            
            xyz = np.zeros((num_points, 3), dtype=np.float32)
            xyz[:, 0] = data[:, off_x:off_x+4].view(np.float32).flatten()
            xyz[:, 1] = data[:, off_y:off_y+4].view(np.float32).flatten()
            xyz[:, 2] = data[:, off_z:off_z+4].view(np.float32).flatten()
            
            mask = ~np.isnan(xyz).any(axis=1)
            return xyz[mask]
        except Exception as e:
            self.get_logger().warn(f"Manual parsing failed: {e}")
            return None

    def publish_debug_pc(self, header, points):
        """Publish points for visualization in RViz"""
        from sensor_msgs.msg import PointField
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
        if rclpy.ok():
            self.debug_pc_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TerrainAnalyzer()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()
