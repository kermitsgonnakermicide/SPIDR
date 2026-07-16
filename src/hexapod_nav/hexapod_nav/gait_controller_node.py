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
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float64MultiArray

from . import kinematics as kin

NUM_LEGS = 6

# Joint names in ros2_control order
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
        self.declare_parameter('nominal_stance_height', -0.12)
        self.declare_parameter('nominal_stance_forward', 0.12)

        self.swing_dur = self.get_parameter('swing_duration').value
        self.max_height = self.get_parameter('max_swing_height').value
        self.freq = self.get_parameter('step_frequency').value
        self.default_z = self.get_parameter('nominal_stance_height').value
        self.default_x = self.get_parameter('nominal_stance_forward').value

        # Foot positions in COXA frame (same for all legs initially)
        self.foot_positions = [np.array([self.default_x, 0.0, self.default_z])
                               for _ in range(NUM_LEGS)]
        self.swing_targets = [None] * NUM_LEGS
        self.swing_progress = [0.0] * NUM_LEGS
        self.active_tripod = 0
        self.cmd_vel = np.zeros(2)
        self.last_cmd_stamp = self.get_clock().now()
        self.terrain_grid = None

        # Body pose in odom frame
        self.body_x = 0.0
        self.body_y = 0.0
        self.body_yaw = 0.0

        # Subscriptions
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

        # Publisher — Float64MultiArray to ros2_control
        self.joint_pub = self.create_publisher(
            Float64MultiArray, '/spooder_controller/commands', 10)

        self.create_timer(1.0 / self.freq, self.control_loop)
        self.get_logger().info('Gait controller node started')

    def cmd_vel_callback(self, msg):
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y])
        self.last_cmd_stamp = self.get_clock().now()

    def terrain_callback(self, msg):
        self.terrain_grid = msg

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.body_x = p.x
        self.body_y = p.y
        self.body_yaw = 2.0 * np.arctan2(q.z, q.w)

    def target_callback(self, msg, leg):
        world_pt = np.array([msg.point.x, msg.point.y, msg.point.z])
        body_pose = np.array([self.body_x, self.body_y, 0.0, self.body_yaw])
        coxa_pt = kin.world_to_coxa(world_pt, leg, body_pose)
        self.swing_targets[leg] = np.array(coxa_pt)

    def replan_callback(self, msg, leg):
        world_pt = np.array([msg.point.x, msg.point.y, msg.point.z])
        body_pose = np.array([self.body_x, self.body_y, 0.0, self.body_yaw])
        coxa_pt = kin.world_to_coxa(world_pt, leg, body_pose)
        self.swing_targets[leg] = np.array(coxa_pt)
        self.get_logger().info(f'Leg {leg}: mid-swing replan')

    def _solve_and_publish(self):
        msg = Float64MultiArray()
        joint_positions = []
        for leg in range(NUM_LEGS):
            q1, q2, q3 = kin.ik_coxa(
                self.foot_positions[leg][0],
                self.foot_positions[leg][1],
                self.foot_positions[leg][2])
            joint_positions.extend([
                max(kin.COXA_LIMITS[0], min(kin.COXA_LIMITS[1], q1)),
                max(kin.FEMUR_LIMITS[0], min(kin.FEMUR_LIMITS[1], q2)),
                max(kin.TIBIA_LIMITS[0], min(kin.TIBIA_LIMITS[1], q3)),
            ])
        msg.data = joint_positions
        self.joint_pub.publish(msg)

    def control_loop(self):
        dt = 1.0 / self.freq

        # Check cmd_vel timeout
        vel_age = (self.get_clock().now() - self.last_cmd_stamp).nanoseconds * 1e-9
        if vel_age > CMD_VEL_TIMEOUT:
            self.cmd_vel = np.zeros(2)

        # Standing: no velocity → hold position
        if abs(self.cmd_vel[0]) < 0.001 and abs(self.cmd_vel[1]) < 0.001:
            self._solve_and_publish()
            return

        swinging = TRIPOD_A if self.active_tripod == 0 else TRIPOD_B
        stance = TRIPOD_B if self.active_tripod == 0 else TRIPOD_A

        # Advance swing progress
        all_done = True
        for leg in swinging:
            self.swing_progress[leg] = min(
                1.0, self.swing_progress[leg] + dt / self.swing_dur)
            if self.swing_progress[leg] < 1.0:
                all_done = False

        if all_done:
            for leg in swinging:
                self.swing_progress[leg] = 0.0
            self.active_tripod = 1 - self.active_tripod
            swinging = TRIPOD_A if self.active_tripod == 0 else TRIPOD_B
            stance = TRIPOD_B if self.active_tripod == 0 else TRIPOD_A

        # Swing: Bezier arc to target (all in coxa frame)
        for leg in swinging:
            t = self.swing_progress[leg]
            target = self.swing_targets[leg]
            if target is None:
                # Default: step forward in coxa frame
                target = self.foot_positions[leg].copy()
                target[0] += self.cmd_vel[0] * self.swing_dur
            arc_h = min(self.max_height, 0.06)
            self.foot_positions[leg] = bezier_arc(
                self.foot_positions[leg], target, arc_h, t)

        # Stance: push feet backward (body moves forward)
        for leg in stance:
            self.foot_positions[leg][0] -= self.cmd_vel[0] * dt

        self._solve_and_publish()


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
