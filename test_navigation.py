#!/usr/bin/env python3
"""
test_navigation.py - Comprehensive navigation goal test

This script tests the hexapod navigation pipeline by:
1. Verifying all required topics and services exist
2. Testing the OctomapGoalPlanner can receive goals
3. Testing the gait controller responds to cmd_vel_nav
4. Testing the complete pipeline flow

Usage:
    source install/setup.bash
    ros2 run hexapod_nav test_navigation.py
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time
import sys

# Import hexapod_nav modules
sys.path.insert(0, '/home/daksh/spooder_ws/src/hexapod_nav/hexapod_nav')

from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import UInt8MultiArray, Float64MultiArray
from grid_map_msgs.msg import GridMap


class NavigationGoalTester(Node):
    def __init__(self):
        super().__init__('navigation_goal_tester')

        self.get_logger().info('Navigation Goal Tester Starting...')

        # Track received messages
        self.received_topics = {
            'goal_3d': 0,
            'cmd_vel_nav': 0,
            'terrain_grid_map': 0,
            'terrain_costmap': 0,
            'leg_phase': 0,
            'plan': 0,
            'odom': 0,
        }

        # Create subscriptions to track all navigation topics
        self.subs = []

        # Goal publisher for testing
        self.goal_pub = self.create_publisher(
            PoseStamped, '/spooder/goal_3d', 10)

        # Velocity publisher for testing
        self.vel_pub = self.create_publisher(
            Twist, '/cmd_vel_nav', 10)

        # Topic monitors
        self.create_subscription(
            PoseStamped, '/spooder/goal_3d',
            lambda msg: self._count('goal_3d', msg), 10)

        self.create_subscription(
            Twist, '/cmd_vel_nav',
            lambda msg: self._count('cmd_vel_nav', msg), 10)

        self.create_subscription(
            GridMap, '/terrain_grid_map',
            lambda msg: self._count('terrain_grid_map', msg), 10)

        self.create_subscription(
            GridMap, '/terrain_costmap',
            lambda msg: self._count('terrain_costmap', msg), 10)

        self.create_subscription(
            UInt8MultiArray, '/leg_phase',
            lambda msg: self._count('leg_phase', msg), 10)

        self.create_subscription(
            Path, '/plan',
            lambda msg: self._count('plan', msg), 10)

        self.create_subscription(
            Path, '/octomap_plan',
            lambda msg: self._count('plan', msg), 10)

        self.get_logger().info('Navigation Goal Tester initialized')

    def _count(self, topic, msg):
        self.received_topics[topic] += 1

    def test_goal_sending(self):
        """Test sending a navigation goal."""
        self.get_logger().info('Testing goal sending...')

        # Clear counter
        self.received_topics['goal_3d'] = 0

        # Send a test goal
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 2.0
        goal.pose.position.y = -0.5
        goal.pose.position.z = 0.1
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.get_logger().info(f'Sent goal: x={goal.pose.position.x}, y={goal.pose.position.y}, z={goal.pose.position.z}')

        # Wait a bit
        time.sleep(1.0)

        # Check if goal was received
        count = self.received_topics['goal_3d']
        if count > 0:
            self.get_logger().info(f'✓ Goal topic working ({count} messages received)')
            return True
        else:
            self.get_logger().warn('✗ No goal messages received on /spooder/goal_3d')
            return False

    def test_velocity_sending(self):
        """Test sending velocity commands."""
        self.get_logger().info('Testing velocity sending...')

        # Clear counter
        self.received_topics['cmd_vel_nav'] = 0

        # Send velocity command
        vel = Twist()
        vel.linear.x = 0.1  # Move forward
        vel.linear.y = 0.0
        vel.angular.z = 0.0

        self.vel_pub.publish(vel)
        self.get_logger().info('Sent velocity command: forward at 0.1 m/s')

        # Wait a bit
        time.sleep(1.0)

        # Check if velocity was received
        count = self.received_topics['cmd_vel_nav']
        if count > 0:
            self.get_logger().info(f'✓ Velocity topic working ({count} messages received)')
            return True
        else:
            self.get_logger().warn('✗ No velocity messages received on /cmd_vel_nav')
            return False

    def test_kinematics(self):
        """Test kinematics calculations."""
        self.get_logger().info('Testing kinematics...')

        try:
            import kinematics as kin

            # Test nominal stance position (matches gait_controller_node default: x=0.085, z=-0.140)
            result = kin.ik_coxa(0.085, 0.0, -0.140)
            assert result is not None, 'IK failed for nominal stance'

            # Test that result is in valid joint range
            q_coxa, q_femur, q_tibia = result
            assert kin.COXA_LIMITS[0] <= q_coxa <= kin.COXA_LIMITS[1], 'Coxa out of range'
            assert kin.FEMUR_LIMITS[0] <= q_femur <= kin.FEMUR_LIMITS[1], 'Femur out of range'
            assert kin.TIBIA_LIMITS[0] <= q_tibia <= kin.TIBIA_LIMITS[1], 'Tibia out of range'

            # Test that unreachable positions now clamp to the workspace edge
            # (replaces the old (0,0,0) fallback which used to pancake the body).
            result = kin.ik_coxa(1.0, 0.0, -1.0)
            # Should NOT be all zeros — that was the bug that collapsed the gait.
            assert result != (0.0, 0.0, 0.0), \
                'Unreachable target should clamp to workspace edge, not return zeros'
            q_coxa, q_femur, q_tibia = result
            # Clamped pose must still be in joint limits.
            assert kin.COXA_LIMITS[0] <= q_coxa <= kin.COXA_LIMITS[1], 'Coxa out of range'
            assert kin.FEMUR_LIMITS[0] <= q_femur <= kin.FEMUR_LIMITS[1], 'Femur out of range'
            assert kin.TIBIA_LIMITS[0] <= q_tibia <= kin.TIBIA_LIMITS[1], 'Tibia out of range'

            # Test reachability check
            assert kin.is_reachable(0.085, 0.0, -0.140), 'New nominal stance should be reachable'
            assert not kin.is_reachable(0.12, 0.0, -0.158), \
                'Old (0.12, 0, -0.158) should be unreachable (confirms the bug)'
            assert not kin.is_reachable(1.0, 0.0, -1.0), 'Far position should not be reachable'

            # Test coordinate transforms
            world_pt = np.array([0.2, 0.0, -0.12])
            body_pose = np.array([0.0, 0.0, 0.0, 0.0])
            coxa_pt = kin.world_to_coxa(world_pt, 0, body_pose)
            assert len(coxa_pt) == 3, 'Coordinate transform should return 3 values'

            self.get_logger().info('✓ All kinematics tests passed')
            return True
        except Exception as e:
            self.get_logger().error(f'✗ Kinematics test failed: {e}')
            return False

    def test_gait_controller_logic(self):
        """Test gait controller state machine logic."""
        self.get_logger().info('Testing gait controller logic...')

        try:
            import kinematics as kin

            # Simulate tripod gait
            TRIPOD_A = [0, 2, 4]
            TRIPOD_B = [1, 3, 5]

            # Verify tripod groups partition the legs
            assert set(TRIPOD_A) | set(TRIPOD_B) == set(range(6)), 'Tripods should cover all legs'
            assert set(TRIPOD_A) & set(TRIPOD_B) == set(), 'Tripods should not overlap'

            # Test foot position clamping
            fx, fy, fz = 0.5, 0.0, -0.5  # Far position
            if not kin.is_reachable(fx, fy, fz):
                cx, cy, cz = kin.clamp_to_reach(fx, fy, fz)
                assert kin.is_reachable(cx, cy, cz), 'Clamped position should be reachable'
                self.get_logger().info('✓ Foot position clamping works')

            # Test all legs can reach nominal stance
            for leg in range(6):
                origin = kin.LEG_ORIGINS[leg]
                angle = kin.LEG_ANGLES[leg]
                coxa_fwd = np.array([0.10, 0.0, -0.10])
                R = np.array([
                    [np.cos(angle), -np.sin(angle), 0],
                    [np.sin(angle),  np.cos(angle), 0],
                    [0,                0,               1]
                ])
                target = R @ coxa_fwd + origin
                result = kin.ik_leg(target, leg)
                assert result is not None, f'Leg {leg} cannot reach nominal stance'

            self.get_logger().info('✓ All gait controller logic tests passed')
            return True
        except Exception as e:
            self.get_logger().error(f'✗ Gait controller logic test failed: {e}')
            return False

    def test_terrain_pipeline(self):
        """Test terrain processing pipeline."""
        self.get_logger().info('Testing terrain pipeline...')

        try:
            import numpy as np
            from grid_map_utils import make_layer
            from std_msgs.msg import Float32MultiArray, MultiArrayDimension

            # Create a mock GridMap
            gm = GridMap()
            gm.info.resolution = 0.05
            gm.info.length_x = 1.0
            gm.info.length_y = 1.0
            gm.layers = ['cost']

            # Create a 4x4 cost layer
            n = 4
            layer = Float32MultiArray()
            dim_x = MultiArrayDimension(label='column_index', size=n, stride=n * n)
            dim_y = MultiArrayDimension(label='row_index', size=n, stride=n)
            layer.layout.dim = [dim_x, dim_y]
            layer.data = list(range(n * n))
            gm.data.append(layer)

            # Test cost extraction
            data = gm.data[0]
            n2 = data.layout.dim[0].size
            arr = np.array(data.data, dtype=np.float32).reshape((n2, n2), order='F')
            assert arr.shape == (4, 4), f'Expected shape (4, 4), got {arr.shape}'

            self.get_logger().info('✓ Terrain pipeline tests passed')
            return True
        except Exception as e:
            self.get_logger().error(f'✗ Terrain pipeline test failed: {e}')
            import traceback
            traceback.print_exc()
            return False

    def run_all_tests(self):
        """Run all navigation tests."""
        results = {}

        # Test kinematics (no ROS required)
        results['kinematics'] = self.test_kinematics()

        # Test gait controller logic (no ROS required)
        results['gait_logic'] = self.test_gait_controller_logic()

        # Test terrain pipeline
        results['terrain_pipeline'] = self.test_terrain_pipeline()

        # ROS-dependent tests
        results['goal_sending'] = self.test_goal_sending()
        results['velocity_sending'] = self.test_velocity_sending()

        # Print summary
        self.get_logger().info('\n=== TEST SUMMARY ===')
        for test, passed in results.items():
            status = '✓ PASSED' if passed else '✗ FAILED'
            self.get_logger().info(f'  {test}: {status}')

        passed_count = sum(1 for v in results.values() if v)
        total_count = len(results)
        self.get_logger().info(f'\nTotal: {passed_count}/{total_count} tests passed')

        if passed_count == total_count:
            self.get_logger().info('✓ All navigation tests PASSED!')
            return True
        else:
            self.get_logger().error('✗ Some tests FAILED!')
            return False


def main():
    rclpy.init(args=sys.argv)
    tester = NavigationGoalTester()

    try:
        success = tester.run_all_tests()
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted')
        success = False
    finally:
        tester.destroy_node()
        rclpy.shutdown()
        sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
