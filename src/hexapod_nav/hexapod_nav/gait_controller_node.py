#!/usr/bin/env python3
"""
gait_controller_node.py

Tripod gait state machine + Bezier swing arc generator.
Replaces spooder_control's gait_controller when hexapod_nav is active.
Publishes Float64MultiArray to /spooder_controller/commands (ros2_control compatible).
"""

import math
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, PointStamped, Point
from nav_msgs.msg import Odometry
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float64MultiArray, UInt8MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from . import kinematics as kin

NUM_LEGS = 6

JOINT_NAMES = [
    'rf_coxa_joint', 'rf_femur_joint', 'rf_tibia_joint',
    'rm_coxa_joint', 'rm_femur_joint', 'rm_tibia_joint',
    'rr_coxa_joint', 'rr_femur_joint', 'rr_tibia_joint',
    'lf_coxa_joint', 'lf_femur_joint', 'lf_tibia_joint',
    'lm_coxa_joint', 'lm_femur_joint', 'lm_tibia_joint',
    'lr_coxa_joint', 'lr_femur_joint', 'lr_tibia_joint',
]

TRIPOD_A = [0, 2, 4]
TRIPOD_B = [1, 3, 5]

CMD_VEL_TIMEOUT = 0.5

LEG_COLORS = [
    (1.0, 0.2, 0.2, 0.85),
    (1.0, 0.6, 0.1, 0.85),
    (1.0, 1.0, 0.2, 0.85),
    (0.2, 1.0, 0.3, 0.85),
    (0.2, 0.6, 1.0, 0.85),
    (0.7, 0.3, 1.0, 0.85),
]


def bezier_arc(p_start, p_end, height, t):
    """Cubic Bezier swing trajectory. t in [0, 1]. All points in coxa frame."""
    p1 = p_start + np.array([0, 0, height])
    p2 = p_end + np.array([0, 0, height])
    return ((1 - t)**3 * p_start
            + 3 * (1 - t)**2 * t * p1
            + 3 * (1 - t) * t**2 * p2
            + t**3 * p_end)


class GaitControllerNode(Node):
    def __init__(self):
        super().__init__('gait_controller_node')

        self.declare_parameter('swing_duration', 0.5)
        self.declare_parameter('max_swing_height', 0.06)
        self.declare_parameter('step_frequency', 20.0)
        # nominal_stance_height is the foot Z in coxa-frame. Coords are coxa-local
        # (origin = coxa joint, +Z up). The coxa joints sit at z=0 in base_link and
        # base_link sits z=BASE_HEIGHT (0.135m, set by spooder.xacro base_joint)
        # above base_footprint (ground proxy). For the foot tip to TOUCH GROUND at
        # rest, foot_world_z must be 0, so foot_coxa_z = -BASE_HEIGHT. We use -0.140
        # so the simulated feet compress ~5 mm into the floor for stable single-point
        # contact. (Previous default -0.158 had body_clearance=0.154, which placed
        # the default foot (0.12, 0, -0.158) outside leg reach (FEMUR+TIBIA=0.164) and
        # made every IK call return (0,0,0), collapsing the gait.)
        # Reachable check at (0.085, 0, -0.140): distance from femur joint =
        # sqrt(0.042^2 + 0.140^2) = 0.146 — 18mm inside the 0.164 reach limit.
        self.declare_parameter('nominal_stance_height', -0.140)
        self.declare_parameter('nominal_stance_forward', 0.085)

        self.swing_dur = self.get_parameter('swing_duration').value
        self.max_height = self.get_parameter('max_swing_height').value
        self.freq = self.get_parameter('step_frequency').value
        self.default_z = self.get_parameter('nominal_stance_height').value
        self.default_x = self.get_parameter('nominal_stance_forward').value

        self.foot_positions = [np.array([self.default_x, 0.0, self.default_z])
                               for _ in range(NUM_LEGS)]
        self.swing_targets = [None] * NUM_LEGS
        self.swing_progress = [0.0] * NUM_LEGS
        self.active_tripod = 0
        self.cmd_vel = np.zeros(3)  # vx, vy, yaw_rate
        self.last_cmd_stamp = self.get_clock().now()
        self.terrain_grid = None

        self.body_x = 0.0
        self.body_y = 0.0
        self.body_z = 0.0
        self.body_yaw = 0.0

        self.last_body_x = 0.0
        self.last_body_y = 0.0
        self.stuck_count = 0
        self.recovery_mode = False
        self.recovery_scale = 1.0

        # Nav2 collision_monitor outputs /cmd_vel; this is what the rest of the chain (legacy
        # controller, unstuck_monitor, hexapod_nav_cpp, rerun_bridge) subscribes to.
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(GridMap, '/terrain_grid_map', self.terrain_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        for i in range(NUM_LEGS):
            self.create_subscription(
                PointStamped, f'/leg_{i}/foothold_target',
                lambda msg, leg=i: self.target_callback(msg, leg), 10)
            self.create_subscription(
                PointStamped, f'/leg_{i}/foothold_replan',
                lambda msg, leg=i: self.replan_callback(msg, leg), 10)

        self.joint_pub = self.create_publisher(
            Float64MultiArray, '/spooder_controller/commands', 10)
        self.phase_pub = self.create_publisher(
            UInt8MultiArray, '/leg_phase', 10)
        self.foot_marker_pub = self.create_publisher(
            MarkerArray, '/gait_foot_markers', 10)

        self.create_timer(1.0 / self.freq, self.control_loop)
        self.get_logger().info('Gait controller node started')

    def cmd_vel_callback(self, msg):
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        self.last_cmd_stamp = self.get_clock().now()

    def terrain_callback(self, msg):
        self.terrain_grid = msg

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.body_x = p.x
        self.body_y = p.y
        self.body_z = self._get_terrain_floor_height(p.x, p.y)
        self.body_yaw = 2.0 * np.arctan2(q.z, q.w)

    def _update_stuck_detection(self):
        dx = abs(self.body_x - self.last_body_x)
        dy = abs(self.body_y - self.last_body_y)
        moved = (dx > 0.001) or (dy > 0.001)
        self.last_body_x = self.body_x
        self.last_body_y = self.body_y
        if moved:
            self.stuck_count = 0
            self.recovery_mode = False
            self.recovery_scale = 1.0
        else:
            self.stuck_count += 1
            if self.stuck_count > int(self.freq * 1.5):
                if not self.recovery_mode:
                    self.get_logger().warn(
                        f'Stuck for {self.stuck_count} cycles, entering recovery mode '
                        f'(scale={self.recovery_scale:.2f})')
                self.recovery_mode = True
                self.recovery_scale = max(0.3, self.recovery_scale - 0.02)

    def target_callback(self, msg, leg):
        world_pt = np.array([msg.point.x, msg.point.y, msg.point.z])
        body_pose = np.array([self.body_x, self.body_y, self.body_z, self.body_yaw])
        coxa_pt = kin.world_to_coxa(world_pt, leg, body_pose)
        self.swing_targets[leg] = np.array(coxa_pt)

    def replan_callback(self, msg, leg):
        world_pt = np.array([msg.point.x, msg.point.y, msg.point.z])
        body_pose = np.array([self.body_x, self.body_y, self.body_z, self.body_yaw])
        coxa_pt = kin.world_to_coxa(world_pt, leg, body_pose)
        self.swing_targets[leg] = np.array(coxa_pt)
        self.get_logger().info(f'Leg {leg}: mid-swing replan')

    def _get_terrain_floor_height(self, wx, wy):
        """Query the 'floor' layer of the terrain grid map at (wx, wy) to get terrain height."""
        if self.terrain_grid is None:
            return 0.0
        try:
            idx = self.terrain_grid.layers.index('floor')
        except ValueError:
            return 0.0
        data = self.terrain_grid.data[idx]
        n = data.layout.dim[0].size
        res = self.terrain_grid.info.resolution
        cx = self.terrain_grid.info.pose.position.x
        cy = self.terrain_grid.info.pose.position.y
        # GridMap convention: index 0 = positive corner, so invert the mapping
        ix = int((cx + self.terrain_grid.info.length_x / 2 - wx) / res)
        iy = int((cy + self.terrain_grid.info.length_y / 2 - wy) / res)
        if 0 <= ix < n and 0 <= iy < n:
            flat = np.array(data.data, dtype=np.float32)
            val = flat[ix + iy * n]
            if np.isfinite(val):
                return float(val)
        return 0.0

    def _get_ceiling_clearance(self, world_pos):
        if self.terrain_grid is None:
            return self.max_height
        try:
            idx = self.terrain_grid.layers.index('clearance')
        except ValueError:
            return self.max_height
        data = self.terrain_grid.data[idx]
        n = data.layout.dim[0].size
        res = self.terrain_grid.info.resolution
        cx = self.terrain_grid.info.pose.position.x
        cy = self.terrain_grid.info.pose.position.y
        # GridMap convention: index 0 = positive corner
        ix = int((cx + self.terrain_grid.info.length_x / 2 - world_pos[0]) / res)
        iy = int((cy + self.terrain_grid.info.length_y / 2 - world_pos[1]) / res)
        if 0 <= ix < n and 0 <= iy < n:
            flat = np.array(data.data, dtype=np.float32)
            val = flat[ix + iy * n]
            if np.isfinite(val) and val > 0:
                return float(val)
        return self.max_height

    def _default_foot(self, leg):
        """Return the default (standing) coxa-frame foot position for a leg."""
        return np.array([self.default_x, 0.0, self.default_z])

    def _body_vel_to_coxa(self, vx_body, vy_body, leg):
        """Rotate body-frame (vx, vy) into the given leg's coxa frame."""
        angle = kin.LEG_ANGLES[leg]
        cos_a = math.cos(-angle)
        sin_a = math.sin(-angle)
        local_vx = vx_body * cos_a - vy_body * sin_a
        local_vy = vx_body * sin_a + vy_body * cos_a
        return local_vx, local_vy

    def _solve_and_publish(self):
        msg = Float64MultiArray()
        joint_positions = []
        for leg in range(NUM_LEGS):
            fx, fy, fz = self.foot_positions[leg]
            if not kin.is_reachable(fx, fy, fz):
                self.foot_positions[leg] = np.array(
                    kin.clamp_to_reach(fx, fy, fz))
                fx, fy, fz = self.foot_positions[leg]
            q1, q2, q3 = kin.ik_coxa(fx, fy, fz)
            joint_positions.extend([
                max(kin.COXA_LIMITS[0], min(kin.COXA_LIMITS[1], q1)),
                max(kin.FEMUR_LIMITS[0], min(kin.FEMUR_LIMITS[1], q2)),
                max(kin.TIBIA_LIMITS[0], min(kin.TIBIA_LIMITS[1], q3)),
            ])
        msg.data = joint_positions
        self.joint_pub.publish(msg)

    def _publish_leg_phase(self, moving: bool):
        """Publish phase: all STANCE when stopped; swinging tripod = SWING when moving."""
        msg = UInt8MultiArray()
        phase = [0] * NUM_LEGS
        if moving:
            swinging = TRIPOD_A if self.active_tripod == 0 else TRIPOD_B
            for leg in swinging:
                phase[leg] = 1
        msg.data = phase
        self.phase_pub.publish(msg)

    def _publish_foot_markers(self):
        """Show current foot tips in map frame (cylinders) for gait visualization."""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = stamp
        clear.ns = 'gait_feet'
        clear.id = 0
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        body_pose = np.array([self.body_x, self.body_y, self.body_z, self.body_yaw])
        swinging = set(TRIPOD_A if self.active_tripod == 0 else TRIPOD_B)

        for leg in range(NUM_LEGS):
            world = kin.coxa_to_world(self.foot_positions[leg], leg, body_pose)
            r, g, b, a = LEG_COLORS[leg]
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'gait_feet'
            m.id = leg + 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(world[0])
            m.pose.position.y = float(world[1])
            m.pose.position.z = float(world[2]) + 0.01
            m.pose.orientation.w = 1.0
            # Taller cylinder while swinging
            tall = leg in swinging and (
                abs(self.cmd_vel[0]) > 0.001 or abs(self.cmd_vel[1]) > 0.001
                or abs(self.cmd_vel[2]) > 0.001)
            m.scale.x = m.scale.y = 0.018
            m.scale.z = 0.04 if tall else 0.02
            m.color = ColorRGBA(r=r, g=g, b=b, a=a)
            ma.markers.append(m)

        self.foot_marker_pub.publish(ma)

    def control_loop(self):
        dt = 1.0 / self.freq

        vel_age = (self.get_clock().now() - self.last_cmd_stamp).nanoseconds * 1e-9
        if vel_age > CMD_VEL_TIMEOUT:
            self.cmd_vel = np.zeros(3)

        vx_global, vy_global, yaw_rate = self.cmd_vel
        cos_y = math.cos(-self.body_yaw)
        sin_y = math.sin(-self.body_yaw)
        vx = vx_global * cos_y - vy_global * sin_y
        vy = vx_global * sin_y + vy_global * cos_y
        moving = abs(vx_global) >= 0.001 or abs(vy_global) >= 0.001 or abs(yaw_rate) >= 0.01

        self._update_stuck_detection()

        if not moving:
            self.stuck_count = 0
            self.recovery_mode = False
            self.recovery_scale = 1.0
            for leg in range(NUM_LEGS):
                self.foot_positions[leg] = self._default_foot(leg)
            self.swing_progress = [0.0] * NUM_LEGS
            self.swing_targets = [None] * NUM_LEGS
            self.active_tripod = 0
            self._solve_and_publish()
            self._publish_leg_phase(moving=False)
            self._publish_foot_markers()
            return

        if self.recovery_mode:
            vx *= self.recovery_scale
            vy *= self.recovery_scale
            yaw_rate *= self.recovery_scale

        swinging = TRIPOD_A if self.active_tripod == 0 else TRIPOD_B
        stance = TRIPOD_B if self.active_tripod == 0 else TRIPOD_A

        all_done = True
        for leg in swinging:
            self.swing_progress[leg] = min(
                1.0, self.swing_progress[leg] + dt / self.swing_dur)
            if self.swing_progress[leg] < 1.0:
                all_done = False

        if all_done:
            for leg in swinging:
                self.swing_progress[leg] = 0.0
                self.swing_targets[leg] = None
            self.active_tripod = 1 - self.active_tripod
            swinging = TRIPOD_A if self.active_tripod == 0 else TRIPOD_B
            stance = TRIPOD_B if self.active_tripod == 0 else TRIPOD_A

        # Swing: Bezier arc to foothold target (or default step from cmd_vel)
        for leg in swinging:
            t = self.swing_progress[leg]
            target = self.swing_targets[leg]
            if target is None:
                local_vx, local_vy = self._body_vel_to_coxa(vx, vy, leg)
                if self.recovery_mode:
                    blend = 0.5
                    target = self.foot_positions[leg] * (1 - blend) + self._default_foot(leg) * blend
                else:
                    target = self.foot_positions[leg].copy()
                target[0] += local_vx * self.swing_dur
                target[1] += local_vy * self.swing_dur
                # Yaw: rotate default foothold slightly in coxa XY
                if abs(yaw_rate) > 0.01:
                    dtheta = yaw_rate * self.swing_dur
                    c, s = math.cos(dtheta), math.sin(dtheta)
                    x, y = target[0], target[1]
                    target[0] = c * x - s * y
                    target[1] = s * x + c * y
                target[0], target[1], target[2] = kin.clamp_to_reach(
                    target[0], target[1], target[2])
            body_pose = np.array([self.body_x, self.body_y, self.body_z, self.body_yaw])
            world_guess = kin.coxa_to_world(target, leg, body_pose)
            arc_h = min(self.max_height, self._get_ceiling_clearance(world_guess) * 0.6)
            self.foot_positions[leg] = bezier_arc(
                self.foot_positions[leg], target, arc_h, t)

        # Stance: push feet opposite to body motion (incl. yaw)
        stance_vel_scale = 0.4 if self.recovery_mode else 1.0
        for leg in stance:
            local_vx, local_vy = self._body_vel_to_coxa(vx, vy, leg)
            self.foot_positions[leg][0] -= local_vx * dt * stance_vel_scale
            self.foot_positions[leg][1] -= local_vy * dt * stance_vel_scale
            if abs(yaw_rate) > 0.01:
                dtheta = -yaw_rate * dt
                c, s = math.cos(dtheta), math.sin(dtheta)
                x, y = self.foot_positions[leg][0], self.foot_positions[leg][1]
                self.foot_positions[leg][0] = c * x - s * y
                self.foot_positions[leg][1] = s * x + c * y
            self.foot_positions[leg][0], self.foot_positions[leg][1], self.foot_positions[leg][2] = \
                kin.clamp_to_reach(
                    self.foot_positions[leg][0],
                    self.foot_positions[leg][1],
                    self.foot_positions[leg][2])
            if self.recovery_mode:
                blend = 0.1
                self.foot_positions[leg] = (
                    self.foot_positions[leg] * (1 - blend)
                    + self._default_foot(leg) * blend)

        self._solve_and_publish()
        self._publish_leg_phase(moving=True)
        self._publish_foot_markers()


def main(args=None):
    rclpy.init(args=args)
    node = GaitControllerNode()
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
