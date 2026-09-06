#!/usr/bin/env python3
"""
test_pipeline_complete.py - End-to-end test of hexapod nav pipeline

Tests:
1. All required nodes are running
2. Pipeline data flow (oak_d -> octomap -> terrain -> cost -> planner -> gait)
3. Joint commands are being published
4. Robot moves when velocity command is sent
5. Navigation goal sending works

Usage:
    source install/setup.bash
    python3 test_pipeline_complete.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import sys
import time
import math
import numpy as np

from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64MultiArray, UInt8MultiArray
from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import PointCloud2
from nav2_msgs.action import NavigateToPose
import sensor_msgs_py.point_cloud2 as pc2


class PipelineTester(Node):
    def __init__(self):
        super().__init__('pipeline_tester')

        # State tracking
        self.tests_passed = 0
        self.tests_failed = 0
        self.warnings = []

        # Data buffers
        self.odom_samples = []
        self.joint_cmd_samples = []
        self.leg_phase_samples = []
        self.terrain_grid_received = False
        self.terrain_costmap_received = False
        self.octomap_cloud_received = False
        self.cmd_vel_received = 0

        # Test state
        self.test_active = False
        self.test_start_time = None
        self.initial_pose = None
        self.final_pose = None
        self.movement_target = 0.3  # 0.3m forward

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions - all the key topics
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Float64MultiArray, '/spooder_controller/commands', self._joint_cmd_cb, 10)
        self.create_subscription(UInt8MultiArray, '/leg_phase', self._leg_phase_cb, 10)
        self.create_subscription(GridMap, '/terrain_grid_map', self._terrain_grid_cb, 10)
        self.create_subscription(GridMap, '/terrain_costmap', self._terrain_costmap_cb, 10)
        self.create_subscription(PointCloud2, '/octomap_point_cloud_centers', self._octomap_cloud_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self._cmd_vel_nav_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # Publisher for tests
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/spooder/goal_3d', 10)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def _odom_cb(self, msg):
        # Always track odom so the preflight "wait for /odom" check works.
        # We use a separate counter to know when/if motion happened during tests.
        sample = (msg.pose.pose.position.x, msg.pose.pose.position.y, time.time())
        self.odom_samples.append(sample)

    def _joint_cmd_cb(self, msg):
        self.joint_cmd_samples.append(msg.data)

    def _leg_phase_cb(self, msg):
        self.leg_phase_samples.append(list(msg.data))

    def _terrain_grid_cb(self, msg):
        self.terrain_grid_received = True

    def _terrain_costmap_cb(self, msg):
        self.terrain_costmap_received = True

    def _octomap_cloud_cb(self, msg):
        self.octomap_cloud_received = True

    def _cmd_vel_nav_cb(self, msg):
        self.cmd_vel_received += 1

    def _cmd_vel_cb(self, msg):
        pass

    def print_result(self, name, passed, msg=""):
        status = "✓ PASS" if passed else "✗ FAIL"
        full = f"  {status} {name}"
        if msg:
            full += f" - {msg}"
        self.get_logger().info(full)
        if passed:
            self.tests_passed += 1
        else:
            self.tests_failed += 1

    def test_1_data_flow(self, timeout=10.0):
        """Test 1: Verify data flow through the pipeline."""
        self.get_logger().info("\n=== Test 1: Data Flow Verification ===")

        # Reset
        self.terrain_grid_received = False
        self.terrain_costmap_received = False
        self.octomap_cloud_received = False
        self.joint_cmd_samples = []

        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.terrain_grid_received and
                self.terrain_costmap_received and
                self.octomap_cloud_received and
                len(self.joint_cmd_samples) > 10):
                break

        self.print_result("OctoMap point cloud received", self.octomap_cloud_received,
                         f"({len(self.joint_cmd_samples)} joint cmds)")
        self.print_result("Terrain grid map received", self.terrain_grid_received)
        self.print_result("Terrain costmap received", self.terrain_costmap_received)
        self.print_result("Joint commands published", len(self.joint_cmd_samples) > 0,
                         f"({len(self.joint_cmd_samples)} samples)")

    def test_2_gait_controller(self, timeout=8.0):
        """Test 2: Verify gait controller responds to velocity commands."""
        self.get_logger().info("\n=== Test 2: Gait Controller ===")

        # Reset
        self.joint_cmd_samples = []
        self.leg_phase_samples = []

        # Send forward velocity CONTINUOUSLY (gait has a 0.5s cmd_vel timeout,
        # so a single publish per loop iteration expires between iterations).
        vel = Twist()
        vel.linear.x = 0.15  # Forward
        start = time.time()
        next_pub = 0.0
        while time.time() - start < timeout:
            now = time.time()
            if now >= next_pub:
                self.vel_pub.publish(vel)
                next_pub = now + 0.1  # publish at 10 Hz
            rclpy.spin_once(self, timeout_sec=0.05)

        # Stop
        self.vel_pub.publish(Twist())

        # Check joint commands are changing (legs moving)
        if len(self.joint_cmd_samples) < 2:
            self.print_result("Joint commands received", False, "no commands")
            return

        # Verify joint commands are NOT all the same (legs are moving)
        first = np.array(self.joint_cmd_samples[0])
        last = np.array(self.joint_cmd_samples[-1])
        motion = np.max(np.abs(last - first))

        self.print_result("Joint commands changing over time", motion > 0.001,
                         f"max delta = {motion:.4f} rad")

        # Check leg_phase published
        self.print_result("Leg phase published", len(self.leg_phase_samples) > 5,
                         f"({len(self.leg_phase_samples)} samples)")

        # Check phase state machine: should see STANCE and SWING values
        if self.leg_phase_samples:
            phases = set()
            for p in self.leg_phase_samples:
                phases.update(p)
            has_stance = 0 in phases
            has_swing = 1 in phases
            self.print_result("Tripod gait: legs alternating STANCE/SWING",
                             has_stance and has_swing,
                             f"states seen: {phases}")

    def test_3_movement(self, target_distance=0.3, timeout=20.0):
        """Test 3: Verify robot moves when commanded."""
        self.get_logger().info("\n=== Test 3: Robot Movement ===")

        # Reset
        self.odom_samples = []
        self.initial_pose = None
        self.final_pose = None

        # Wait for odom
        start = time.time()
        while time.time() - start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom_samples:
                break

        if not self.odom_samples:
            self.print_result("Odometry available", False, "no /odom data")
            return

        self.initial_pose = self.odom_samples[0]
        start_x, start_y, _ = self.initial_pose

        # Send forward velocity CONTINUOUSLY at 10 Hz so gait controller's
        # 0.5s cmd_vel_timeout doesn't kill motion between iterations.
        self.test_active = True
        vel = Twist()
        vel.linear.x = 0.15  # 0.15 m/s
        start = time.time()
        next_pub = 0.0
        while time.time() - start < timeout:
            now = time.time()
            if now >= next_pub:
                self.vel_pub.publish(vel)
                next_pub = now + 0.1
            rclpy.spin_once(self, timeout_sec=0.05)
            if len(self.odom_samples) >= 5:
                # Check if we've moved
                dx = self.odom_samples[-1][0] - start_x
                dy = self.odom_samples[-1][1] - start_y
                dist = math.sqrt(dx*dx + dy*dy)
                if dist >= target_distance:
                    break

        # Stop
        self.vel_pub.publish(Twist())
        self.test_active = False

        # Wait a bit for odom to settle
        time.sleep(0.5)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.odom_samples:
            self.print_result("Movement detected", False, "no odom samples")
            return

        # Calculate distance moved
        self.final_pose = self.odom_samples[-1]
        end_x, end_y, _ = self.final_pose
        dx = end_x - start_x
        dy = end_y - start_y
        distance = math.sqrt(dx*dx + dy*dy)

        self.print_result(f"Robot moved {distance:.3f}m (target: {target_distance}m)",
                         distance >= target_distance * 0.5,
                         f"distance = {distance:.3f} m in {timeout}s")
        self.print_result("Movement direction correct (forward)",
                         dx > 0.01,
                         f"dx = {dx:.3f}, dy = {dy:.3f}")

    def test_4_navigation_goal(self, x=1.0, y=0.0):
        """Test 4: Send navigation goal and verify Nav2 accepts it."""
        self.get_logger().info("\n=== Test 4: Navigation Goal ===")

        # Check Nav2 is available
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.print_result("Nav2 action server available", False)
            return

        self.print_result("Nav2 action server available", True)

        # Reset odom tracking
        self.odom_samples = []
        self.initial_pose = None
        self.final_pose = None

        # Wait for odom
        start = time.time()
        while time.time() - start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom_samples:
                break

        if self.odom_samples:
            self.initial_pose = self.odom_samples[0]

        # Send goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'  Sending goal: x={x}, y={y}')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

        # Wait for response
        start = time.time()
        while time.time() - start < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        time.sleep(2.0)
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)

        # Check if robot is moving toward goal
        if self.odom_samples and self.initial_pose:
            self.final_pose = self.odom_samples[-1]
            dx = self.final_pose[0] - self.initial_pose[0]
            dy = self.final_pose[1] - self.initial_pose[1]
            distance = math.sqrt(dx*dx + dy*dy)
            self.print_result("Robot started moving after goal",
                             distance > 0.01,
                             f"moved {distance:.3f}m")

    def _goal_response_cb(self, future):
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('  Goal ACCEPTED by Nav2')
            else:
                self.get_logger().warn('  Goal REJECTED by Nav2')
        except Exception as e:
            self.get_logger().error(f'  Goal error: {e}')

    def run_all_tests(self):
        """Run all tests in sequence."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("HEXAPOD NAVIGATION PIPELINE - COMPREHENSIVE TEST")
        self.get_logger().info("="*60)

        # Wait for odom first
        self.get_logger().info("\nWaiting for /odom...")
        start = time.time()
        while time.time() - start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom_samples:
                break

        if not self.odom_samples:
            self.get_logger().error("No /odom data after 10s! Aborting.")
            return False

        # Run tests
        self.test_1_data_flow()
        time.sleep(1.0)
        self.test_2_gait_controller()
        time.sleep(1.0)
        self.test_3_movement()
        time.sleep(1.0)
        self.test_4_navigation_goal()

        # Summary
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("TEST SUMMARY")
        self.get_logger().info("="*60)
        self.get_logger().info(f"  Passed: {self.tests_passed}")
        self.get_logger().info(f"  Failed: {self.tests_failed}")
        total = self.tests_passed + self.tests_failed
        if total > 0:
            self.get_logger().info(f"  Pass rate: {100.0 * self.tests_passed / total:.1f}%")
        self.get_logger().info("="*60)

        if self.warnings:
            self.get_logger().warn("Warnings:")
            for w in self.warnings:
                self.get_logger().warn(f"  - {w}")

        return self.tests_failed == 0


def main():
    rclpy.init(args=sys.argv)
    tester = PipelineTester()

    try:
        success = tester.run_all_tests()
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted')
        success = False
    except Exception as e:
        tester.get_logger().error(f'Test failed with exception: {e}')
        import traceback
        traceback.print_exc()
        success = False
    finally:
        tester.destroy_node()
        rclpy.shutdown()
        sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
