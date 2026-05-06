#!/usr/bin/env python3
"""Time-parameterize OctoMap terrain plans for visualization and gait sync."""

import math
import threading

from geometry_msgs.msg import PoseStamped, Transform, Twist
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


class OctomapTrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('octomap_trajectory_planner')

        self.declare_parameter('plan_topic', '/octomap_plan')
        self.declare_parameter('grid_topic', '/terrain/traversability')
        self.declare_parameter('trajectory_topic', '/octomap_trajectory')
        self.declare_parameter('trajectory_path_topic', '/octomap_trajectory_path')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('joint_name', 'base_link')
        self.declare_parameter('sample_spacing', 0.10)
        self.declare_parameter('free_speed', 0.18)
        self.declare_parameter('climb_speed', 0.08)
        self.declare_parameter('slow_cost_threshold', 20)
        self.declare_parameter('lethal_threshold', 90)

        self.plan_topic = self.get_parameter('plan_topic').value
        self.grid_topic = self.get_parameter('grid_topic').value
        self.trajectory_topic = self.get_parameter('trajectory_topic').value
        self.trajectory_path_topic = self.get_parameter('trajectory_path_topic').value
        self.global_frame = self.get_parameter('global_frame').value
        self.joint_name = self.get_parameter('joint_name').value
        self.sample_spacing = max(0.01, float(self.get_parameter('sample_spacing').value))
        self.free_speed = max(0.01, float(self.get_parameter('free_speed').value))
        self.climb_speed = max(0.01, float(self.get_parameter('climb_speed').value))
        self.slow_cost_threshold = int(
            self.get_parameter('slow_cost_threshold').value
        )
        self.lethal_threshold = int(self.get_parameter('lethal_threshold').value)

        self.lock = threading.Lock()
        self.grid_msg = None
        self.grid = None

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.grid_sub = self.create_subscription(
            OccupancyGrid,
            self.grid_topic,
            self.grid_callback,
            latched_qos,
        )
        self.plan_sub = self.create_subscription(
            Path,
            self.plan_topic,
            self.plan_callback,
            latched_qos,
        )
        self.trajectory_pub = self.create_publisher(
            MultiDOFJointTrajectory,
            self.trajectory_topic,
            latched_qos,
        )
        self.path_pub = self.create_publisher(
            Path,
            self.trajectory_path_topic,
            latched_qos,
        )

        self.get_logger().info('OctoMap trajectory planner initialized')
        self.get_logger().info(f'  Plan topic: {self.plan_topic}')
        self.get_logger().info(f'  Trajectory topic: {self.trajectory_topic}')

    def grid_callback(self, msg):
        data = np.array(msg.data, dtype=np.int16).reshape(
            int(msg.info.height),
            int(msg.info.width),
        )
        with self.lock:
            self.grid_msg = msg
            self.grid = data

    def plan_callback(self, msg):
        samples = self.sample_plan(msg)
        if not samples:
            self.get_logger().warn('Received an empty OctoMap plan; no trajectory.')
            return

        with self.lock:
            grid_msg = self.grid_msg
            grid = None if self.grid is None else self.grid.copy()

        costs = [self.lookup_cost(grid_msg, grid, x, y) for x, y, _, _ in samples]
        speeds = [self.speed_for_cost(cost) for cost in costs]
        lethal_samples = sum(
            1 for cost in costs if cost is not None and cost >= self.lethal_threshold
        )
        if lethal_samples:
            self.get_logger().warn(
                f'Trajectory samples include {lethal_samples} lethal terrain cells.'
            )

        frame_id = msg.header.frame_id or self.global_frame
        stamp = self.get_clock().now().to_msg()
        trajectory = MultiDOFJointTrajectory()
        trajectory.header.stamp = stamp
        trajectory.header.frame_id = frame_id
        trajectory.joint_names = [self.joint_name]

        trajectory_path = Path()
        trajectory_path.header = trajectory.header

        elapsed = 0.0
        for index, sample in enumerate(samples):
            x, y, z, yaw = sample
            if index > 0:
                prev_x, prev_y, _, _ = samples[index - 1]
                distance = math.hypot(x - prev_x, y - prev_y)
                segment_speed = min(speeds[index - 1], speeds[index])
                elapsed += distance / max(segment_speed, 0.01)

            point = self.trajectory_point(x, y, z, yaw, speeds[index], elapsed)
            if index == len(samples) - 1:
                point.velocities = [Twist()]
            trajectory.points.append(point)
            trajectory_path.poses.append(self.path_pose(frame_id, stamp, x, y, z, yaw))

        self.trajectory_pub.publish(trajectory)
        self.path_pub.publish(trajectory_path)
        self.get_logger().info(
            f'Published OctoMap trajectory with {len(trajectory.points)} samples, '
            f'{elapsed:.1f}s duration.'
        )

    def sample_plan(self, path_msg):
        if not path_msg.poses:
            return []

        points = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
                self.yaw_from_pose(pose),
            )
            for pose in path_msg.poses
        ]
        if len(points) == 1:
            return [points[0]]

        samples = []
        last_yaw = points[0][3]
        for index in range(len(points) - 1):
            start = points[index]
            end = points[index + 1]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            dz = end[2] - start[2]
            distance = math.hypot(dx, dy)
            if distance < 1e-6:
                continue
            yaw = math.atan2(dy, dx)
            last_yaw = yaw
            steps = max(1, int(math.ceil(distance / self.sample_spacing)))
            for step in range(steps):
                ratio = step / steps
                samples.append(
                    (
                        start[0] + dx * ratio,
                        start[1] + dy * ratio,
                        start[2] + dz * ratio,
                        yaw,
                    )
                )

        final = points[-1]
        samples.append((final[0], final[1], final[2], last_yaw))
        return samples

    def lookup_cost(self, grid_msg, grid, x, y):
        if grid_msg is None or grid is None:
            return 0

        cell = self.world_to_cell(grid_msg, x, y)
        if cell is None:
            return -1

        row, col = cell
        return int(grid[row, col])

    @staticmethod
    def world_to_cell(grid_msg, x, y):
        resolution = float(grid_msg.info.resolution)
        origin = grid_msg.info.origin.position
        col = int(math.floor((x - origin.x) / resolution))
        row = int(math.floor((y - origin.y) / resolution))
        if row < 0 or col < 0:
            return None
        if row >= grid_msg.info.height or col >= grid_msg.info.width:
            return None
        return row, col

    def speed_for_cost(self, cost):
        if cost is None or cost < self.slow_cost_threshold:
            return self.free_speed
        return self.climb_speed

    @staticmethod
    def yaw_from_pose(pose):
        q = pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def yaw_to_quaternion(yaw):
        half_yaw = yaw * 0.5
        return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)

    def trajectory_point(self, x, y, z, yaw, speed, elapsed):
        point = MultiDOFJointTrajectoryPoint()

        transform = Transform()
        transform.translation.x = x
        transform.translation.y = y
        transform.translation.z = z
        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
        transform.rotation.x = qx
        transform.rotation.y = qy
        transform.rotation.z = qz
        transform.rotation.w = qw
        point.transforms = [transform]

        velocity = Twist()
        velocity.linear.x = speed * math.cos(yaw)
        velocity.linear.y = speed * math.sin(yaw)
        velocity.linear.z = 0.0
        point.velocities = [velocity]

        sec = int(max(0.0, elapsed))
        nanosec = int(round((elapsed - sec) * 1_000_000_000))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        return point

    def path_pose(self, frame_id, stamp, x, y, z, yaw):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = OctomapTrajectoryPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
