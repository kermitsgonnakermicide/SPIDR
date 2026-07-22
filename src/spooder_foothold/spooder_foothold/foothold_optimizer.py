import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import numpy as np
import math
from collections import deque

from spooder_foothold.voxel_map import VoxelMap
from spooder_foothold.candidate_sampler import sample_polar_grid, filter_by_reachability
from spooder_foothold.terrain_features import compute_total_cost


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

        self.declare_parameter('timer_period', 0.1)
        self.declare_parameter('default_z', -0.12)
        self.declare_parameter('stride_amp', 0.3)
        self.declare_parameter('step_height', 0.05)

        self.voxel_map = VoxelMap(resolution=0.05, neighbor_radius=0.15)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.octomap_sub = self.create_subscription(
            PointCloud2,
            '/octomap_point_cloud_centers',
            self.octomap_callback, 10
        )

        self.gait_phase_sub = self.create_subscription(
            Float32,
            '/spooder/gait_phase',
            self.gait_phase_callback, 10
        )

        self.goal_3d_sub = self.create_subscription(
            PoseStamped,
            '/spooder/goal_3d',
            self.goal_3d_callback, 10
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
        self.body_height_pub = self.create_publisher(
            Float32,
            '/spooder/target_body_height', 10
        )

        self.gait_phase = 0.0
        self.last_gait_time = self.get_clock().now()
        self.default_z = self.get_parameter('default_z').value
        self.step_height = self.get_parameter('step_height').value
        self.stride_amp = self.get_parameter('stride_amp').value
        self.timer_period = self.get_parameter('timer_period').value

        self.target_goal_z = 0.0
        self.target_body_lift = 0.0
        self.current_body_lift = 0.0
        self.goal_z_timeout = 5.0
        self.last_goal_stamp = self.get_clock().now()

        self.leg_origins_body = [
            np.array([LEG_PARAMS[n]['x_off'], LEG_PARAMS[n]['y_off'], 0.0])
            for n in LEG_NAMES
        ]

        self.prev_footholds_map = deque(maxlen=12)
        self.current_footholds_map = {}

        self.optimizer_timer = self.create_timer(self.timer_period, self.optimize_callback)
        self.get_logger().info('FootholdOptimizer initialized with Phase 2 costs')

    def octomap_callback(self, msg):
        points = pointcloud2_to_numpy(msg)
        if points is not None and len(points) > 0:
            self.voxel_map.update(points)

    def gait_phase_callback(self, msg):
        self.gait_phase = msg.data
        self.last_gait_time = self.get_clock().now()

    def goal_3d_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', msg.header.frame_id, rclpy.time.Time())
            t = transform.transform.translation
            self.target_goal_z = t.z + msg.pose.position.z
        except Exception:
            self.target_goal_z = msg.pose.position.z
        self.target_body_lift = max(0.0, self.target_goal_z)
        self.last_goal_stamp = self.get_clock().now()
        self.get_logger().info(f'3D goal z={self.target_goal_z:.3f}, body lift target={self.target_body_lift:.3f}')

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
        t_start = self.get_clock().now()

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

        goal_age = (self.get_clock().now() - self.last_goal_stamp).nanoseconds * 1e-9
        valid_goal = goal_age < self.goal_z_timeout and self.target_body_lift > 0.01
        if valid_goal:
            smooth_rate = 0.02
            if self.current_body_lift < self.target_body_lift:
                self.current_body_lift = min(
                    self.current_body_lift + smooth_rate, self.target_body_lift)
            elif self.current_body_lift > self.target_body_lift:
                self.current_body_lift = max(
                    self.current_body_lift - smooth_rate, self.target_body_lift)
        else:
            self.current_body_lift *= 0.95

        body_height_msg = Float32()
        body_height_msg.data = float(self.current_body_lift)
        self.body_height_pub.publish(body_height_msg)

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
                    cand_bf = cand_arr - np.array([[robot_x, robot_y, robot_z]])

                    active_leg_mask = [False] * 6
                    for j in range(6):
                        gj = LEG_PARAMS[LEG_NAMES[j]]['tripod']
                        gj_offset = math.pi if gj == 1 else 0.0
                        active_leg_mask[j] = math.sin(phase + gj_offset) > 0

                    costs = compute_total_cost(
                        self.voxel_map, cand_arr,
                        self.leg_origins_body, active_leg_mask,
                        self.prev_footholds_map, i,
                        default_z=self.default_z,
                    )

                    if len(costs) > 0:
                        best_idx = int(np.argmin(costs))
                        best_candidate = cand_arr[best_idx]
                        self.current_footholds_map[i] = best_candidate.tolist()

                        self.publish_candidates(cand_arr)
                        self.publish_selected(best_candidate)
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

        if len(self.prev_footholds_map) >= 12:
            self.prev_footholds_map.pop()
        if self.current_footholds_map:
            avg_foothold = np.mean(list(self.current_footholds_map.values()), axis=0)
            self.prev_footholds_map.append(avg_foothold)

        dt = (self.get_clock().now() - t_start).nanoseconds * 1e-6
        if dt > self.timer_period * 1000:
            self.get_logger().warn(
                f'Optimizer cycle took {dt:.1f}ms (exceeds {self.timer_period*1000:.0f}ms budget)',
                throttle_duration_sec=2.0)

    def publish_candidates(self, candidates):
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

    def publish_selected(self, candidate):
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
