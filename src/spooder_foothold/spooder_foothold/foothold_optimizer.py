import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray, Header
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import numpy as np
import math

from spooder_foothold.voxel_map import VoxelMap
from spooder_foothold.candidate_sampler import sample_polar_grid, filter_by_reachability
from spooder_foothold.terrain_features import extract_all, compute_cost


LEG_PARAMS = {
    'rf': {'x_off': 0.0835, 'y_off': -0.063, 'yaw': -0.7853, 'tripod': 0},
    'rm': {'x_off': 0.0,    'y_off': -0.063, 'yaw': -1.5708, 'tripod': 1},
    'rr': {'x_off': -0.0835,'y_off': -0.063, 'yaw': -2.3561, 'tripod': 0},
    'lf': {'x_off': 0.0835, 'y_off': 0.063,  'yaw': 0.7853,  'tripod': 1},
    'lm': {'x_off': 0.0,    'y_off': 0.063,  'yaw': 1.5708,  'tripod': 0},
    'lr': {'x_off': -0.0835,'y_off': 0.063,  'yaw': 2.3561,  'tripod': 1},
}
LEG_NAMES = ['rf', 'rm', 'rr', 'lf', 'lm', 'lr']
BASE_FOOTPRINT_TO_BASE_LINK_Z = 0.154


def pointcloud2_to_numpy(cloud_msg):
    try:
        fields = {field.name: field for field in cloud_msg.fields}
        if not all(f in fields for f in ('x', 'y', 'z')):
            return None
        point_step = cloud_msg.point_step
        num_points = cloud_msg.width * cloud_msg.height
        if num_points == 0 or point_step == 0:
            return np.empty((0, 3), dtype=np.float32)
        raw = np.frombuffer(cloud_msg.data, dtype=np.uint8)
        expected_size = num_points * point_step
        if raw.size < expected_size:
            return None
        data = raw[:expected_size].reshape(num_points, point_step)
        float_dtype = np.dtype('>f4' if cloud_msg.is_bigendian else '<f4')
        xyz = np.zeros((num_points, 3), dtype=np.float32)
        for col, name in enumerate(('x', 'y', 'z')):
            offset = fields[name].offset
            xyz[:, col] = data[:, offset:offset + 4].copy().view(float_dtype).reshape(-1)
        return xyz[np.isfinite(xyz).all(axis=1)]
    except Exception:
        return None


class FootholdOptimizer(Node):
    def __init__(self):
        super().__init__('foothold_optimizer')
        self.get_logger().info('FootholdOptimizer starting')

        self.voxel_map = VoxelMap(resolution=0.05, neighbor_radius=0.15)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.octomap_sub = self.create_subscription(
            PointCloud2,
            '/octomap_server/octomap_point_cloud_centers',
            self.octomap_callback, 10
        )

        self.gait_phase_sub = self.create_subscription(
            Float32,
            '/spooder/gait_phase',
            self.gait_phase_callback, 10
        )

        self.target_pub = self.create_publisher(
            Float64MultiArray,
            '/spooder/foothold_targets', 10
        )
        self.candidates_pub = self.create_publisher(
            MarkerArray,
            '/spooder/foothold_candidates', 10
        )
        self.selected_pub = self.create_publisher(
            MarkerArray,
            '/spooder/foothold_selected', 10
        )

        self.gait_phase = 0.0
        self.gait_speed = 4.0
        self.last_gait_time = self.get_clock().now()
        self.default_z = -0.12
        self.step_height = 0.05
        self.stride_amp = 0.07
        self.timer_period = 0.05

        self.optimizer_timer = self.create_timer(0.1, self.optimize_callback)
        self.get_logger().info('FootholdOptimizer initialized')

    def octomap_callback(self, msg):
        points = pointcloud2_to_numpy(msg)
        if points is not None and len(points) > 0:
            self.voxel_map.update(points)

    def gait_phase_callback(self, msg):
        self.gait_phase = msg.data
        self.last_gait_time = self.get_clock().now()

    def nominal_foot_in_base_footprint(self, leg_name, leg_idx):
        params = LEG_PARAMS[leg_name]
        yaw = params['yaw']
        x_off = params['x_off']
        y_off = params['y_off']

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        foot_in_coxa = np.array([0.12, 0.0, self.default_z])
        R = np.array([
            [cos_yaw, -sin_yaw, 0],
            [sin_yaw,  cos_yaw, 0],
            [0,        0,       1]
        ])
        foot_in_base_link = R @ foot_in_coxa + np.array([x_off, y_off, 0.0])
        foot_in_base_footprint = foot_in_base_link + np.array([0.0, 0.0, BASE_FOOTPRINT_TO_BASE_LINK_Z])
        return foot_in_base_footprint

    def optimize_callback(self):
        if not self.voxel_map.is_updated:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'spooder/base_footprint', rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            robot_z = transform.transform.translation.z
        except Exception:
            return

        phase = self.gait_phase
        optimized_targets = []

        for i, leg_name in enumerate(LEG_NAMES):
            tripod_group = LEG_PARAMS[leg_name]['tripod']
            group_offset = math.pi if tripod_group == 1 else 0.0
            leg_phase = phase + group_offset
            in_swing = math.sin(leg_phase) > 0

            nominal_bf = self.nominal_foot_in_base_footprint(leg_name, i)
            nominal_map = np.array([
                nominal_bf[0] + robot_x,
                nominal_bf[1] + robot_y,
                nominal_bf[2] + robot_z,
            ])

            if in_swing and self.voxel_map.is_updated:
                candidates = sample_polar_grid(
                    nominal_map[0], nominal_map[1],
                    radial_steps=3, angular_steps=8,
                    min_radius=0.0, max_radius=0.05
                )
                surface_candidates = []
                for (cx, cy, r, theta) in candidates:
                    h = self.voxel_map.height_at(cx, cy)
                    if h is not None:
                        surface_candidates.append([cx, cy, h])
                    else:
                        surface_candidates.append([cx, cy, nominal_map[2]])

                if surface_candidates:
                    cand_arr = np.array(surface_candidates, dtype=np.float32)
                    features = extract_all(self.voxel_map, cand_arr)
                    if len(features) > 0:
                        costs = compute_cost(features)
                        best_idx = int(np.argmin(costs))
                        best_candidate = cand_arr[best_idx]

                        self.publish_candidates(cand_arr, robot_x, robot_y, robot_z)
                        self.publish_selected(best_candidate, robot_x, robot_y, robot_z)
                    else:
                        best_candidate = nominal_map
                else:
                    best_candidate = nominal_map
            else:
                best_candidate = nominal_map

            best_bf = np.array([
                best_candidate[0] - robot_x,
                best_candidate[1] - robot_y,
                best_candidate[2] - robot_z,
            ])
            optimized_targets.extend([float(best_bf[0]), float(best_bf[1]), float(best_bf[2])])

        msg = Float64MultiArray()
        msg.data = optimized_targets
        self.target_pub.publish(msg)

    def publish_candidates(self, candidates, robot_x, robot_y, robot_z):
        if not candidates or len(candidates) == 0:
            return
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.ns = 'foothold_candidates'
        marker.id = 0
        marker.scale.x = 0.015
        marker.scale.y = 0.015
        marker.color.a = 0.6
        marker.color.g = 1.0
        marker.points = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2])) for c in candidates]
        marker_array.markers.append(marker)
        self.candidates_pub.publish(marker_array)

    def publish_selected(self, candidate, robot_x, robot_y, robot_z):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.ns = 'foothold_selected'
        marker.id = 0
        marker.pose.position.x = float(candidate[0])
        marker.pose.position.y = float(candidate[1])
        marker.pose.position.z = float(candidate[2])
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.a = 1.0
        marker.color.b = 1.0
        marker.color.r = 1.0
        marker_array.markers.append(marker)
        self.selected_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FootholdOptimizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
