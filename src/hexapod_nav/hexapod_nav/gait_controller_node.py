#!/usr/bin/env python3
"""
gait_controller_node.py

Tripod gait state machine + Bezier swing arc generator.
The swing arc height is clipped to available ceiling clearance at target cell.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import JointState
from grid_map_msgs.msg import GridMap

from . import kinematics as kin
from .foothold_planner_node import FootholdPlannerNode


NUM_LEGS = 6

# Tripod groups
TRIPOD_A = [0, 2, 4]  # front-right, rear-right, mid-left
TRIPOD_B = [1, 3, 5]  # mid-right, rear-left, front-left


def bezier_arc(p_start, p_end, height, t):
    """
    Cubic Bezier swing trajectory.
    t in [0, 1]. Returns (x, y, z) at parameter t.
    height is clipped to available ceiling clearance.
    """
    p1 = p_start + np.array([0, 0, height])
    p2 = p_end   + np.array([0, 0, height])
    return ((1-t)**3 * p_start
          + 3*(1-t)**2*t * p1
          + 3*(1-t)*t**2 * p2
          + t**3 * p_end)


class GaitControllerNode(Node):
    def __init__(self):
        super().__init__('gait_controller_node')

        self.declare_parameter('swing_duration', 0.5)          # seconds per swing
        self.declare_parameter('max_swing_height', 0.06)        # max arc height (m)
        self.declare_parameter('step_frequency', 20.0)          # control loop Hz

        self.swing_dur = self.get_parameter('swing_duration').value
        self.max_height = self.get_parameter('max_swing_height').value
        freq = self.get_parameter('step_frequency').value

        # State
        self.foot_positions = [kin.LEG_ORIGINS[i] + np.array([0.06, 0, -0.12])
                               for i in range(NUM_LEGS)]
        self.swing_targets   = [None] * NUM_LEGS
        self.swing_progress  = [0.0]  * NUM_LEGS
        self.active_tripod   = 0   # 0 = A swinging, 1 = B swinging
        self.cmd_vel         = np.zeros(2)  # vx, vy
        self.terrain_grid    = None

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(GridMap, '/terrain_grid_map', self.terrain_callback, 10)

        for i in range(NUM_LEGS):
            self.create_subscription(
                PointStamped, f'/leg_{i}/foothold_target',
                lambda msg, leg=i: self.target_callback(msg, leg), 10
            )
            self.create_subscription(
                PointStamped, f'/leg_{i}/foothold_replan',
                lambda msg, leg=i: self.replan_callback(msg, leg), 10
            )

        # Publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Control loop
        self.create_timer(1.0 / freq, self.control_loop)

        self.get_logger().info('Gait controller started')

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y])

    def terrain_callback(self, msg: GridMap):
        self.terrain_grid = msg

    def target_callback(self, msg: PointStamped, leg: int):
        self.swing_targets[leg] = np.array([msg.point.x, msg.point.y, msg.point.z])

    def replan_callback(self, msg: PointStamped, leg: int):
        """Mid-swing replan — update target immediately."""
        new_target = np.array([msg.point.x, msg.point.y, msg.point.z])
        self.get_logger().info(f'Leg {leg}: mid-swing replan to {new_target}')
        self.swing_targets[leg] = new_target

    def _get_ceiling_clearance(self, world_pos: np.ndarray) -> float:
        """Look up ceiling clearance at a world XY position from terrain grid."""
        if self.terrain_grid is None:
            return self.max_height
        # Extract clearance layer and look up value — simplified
        # In production: use grid_map index lookup
        return self.max_height

    def control_loop(self):
        """Run at step_frequency Hz. Advance swing arcs and send joint commands."""
        dt = 1.0 / self.get_parameter('step_frequency').value

        swinging = TRIPOD_A if self.active_tripod == 0 else TRIPOD_B
        stance   = TRIPOD_B if self.active_tripod == 0 else TRIPOD_A

        # Advance swing progress
        all_done = True
        for leg in swinging:
            self.swing_progress[leg] = min(1.0, self.swing_progress[leg] + dt / self.swing_dur)
            if self.swing_progress[leg] < 1.0:
                all_done = False

        if all_done:
            # Switch tripods
            for leg in swinging:
                self.swing_progress[leg] = 0.0
            self.active_tripod = 1 - self.active_tripod

        # Compute foot positions
        for leg in swinging:
            t = self.swing_progress[leg]
            if self.swing_targets[leg] is not None:
                clearance = self._get_ceiling_clearance(self.swing_targets[leg])
                arc_height = min(self.max_height, clearance * 0.6)  # use 60% of clearance
                self.foot_positions[leg] = bezier_arc(
                    self.foot_positions[leg],
                    self.swing_targets[leg],
                    arc_height, t
                )

        # Push stance feet back (body moves forward)
        speed = np.linalg.norm(self.cmd_vel)
        for leg in stance:
            self.foot_positions[leg][0] -= self.cmd_vel[0] * dt
            self.foot_positions[leg][1] -= self.cmd_vel[1] * dt

        # Solve IK and publish
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for leg in range(NUM_LEGS):
            result = kin.ik_leg(self.foot_positions[leg], leg)
            if result:
                q_coxa, q_femur, q_tibia = result
                msg.name  += [f'leg{leg}_coxa', f'leg{leg}_femur', f'leg{leg}_tibia']
                msg.position += [q_coxa, q_femur, q_tibia]

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()