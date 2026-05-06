#!/usr/bin/env python3
"""Plan terrain-aware waypoint routes from the OctoMap traversability grid."""

import heapq
import math
import threading

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros


class OctomapGoalPlanner(Node):
    def __init__(self):
        super().__init__('octomap_goal_planner')

        self.declare_parameter('grid_topic', '/terrain/traversability')
        self.declare_parameter('goal_topic', '/octomap_goal')
        self.declare_parameter('plan_topic', '/octomap_plan')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'spooder/base_footprint')
        self.declare_parameter('navigate_action', '/navigate_through_poses')
        self.declare_parameter('send_to_nav2', True)
        self.declare_parameter('lethal_threshold', 90)
        self.declare_parameter('cost_weight', 3.0)
        self.declare_parameter('waypoint_spacing', 0.45)
        self.declare_parameter('goal_search_radius_cells', 12)
        self.declare_parameter('max_expansions', 120000)

        self.grid_topic = self.get_parameter('grid_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.plan_topic = self.get_parameter('plan_topic').value
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.navigate_action = self.get_parameter('navigate_action').value
        self.send_to_nav2 = bool(self.get_parameter('send_to_nav2').value)
        self.lethal_threshold = int(self.get_parameter('lethal_threshold').value)
        self.cost_weight = float(self.get_parameter('cost_weight').value)
        self.waypoint_spacing = float(self.get_parameter('waypoint_spacing').value)
        self.goal_search_radius_cells = int(
            self.get_parameter('goal_search_radius_cells').value
        )
        self.max_expansions = int(self.get_parameter('max_expansions').value)

        self.lock = threading.Lock()
        self.grid_msg = None
        self.grid = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.grid_sub = self.create_subscription(
            OccupancyGrid,
            self.grid_topic,
            self.grid_callback,
            map_qos,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10,
        )
        self.plan_pub = self.create_publisher(Path, self.plan_topic, map_qos)
        self.nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            self.navigate_action,
        )

        self.get_logger().info('OctoMap goal planner initialized')
        self.get_logger().info(f'  Goal topic: {self.goal_topic}')
        self.get_logger().info(f'  Plan topic: {self.plan_topic}')

    def grid_callback(self, msg):
        data = np.array(msg.data, dtype=np.int16).reshape(
            int(msg.info.height),
            int(msg.info.width),
        )
        with self.lock:
            self.grid_msg = msg
            self.grid = data

    def goal_callback(self, goal_msg):
        with self.lock:
            grid_msg = self.grid_msg
            grid = None if self.grid is None else self.grid.copy()

        if grid_msg is None or grid is None:
            self.get_logger().warn('No /terrain/traversability grid yet; cannot plan.')
            return

        robot_xy = self.lookup_robot_xy()
        if robot_xy is None:
            return

        goal_xy = self.pose_xy_in_global_frame(goal_msg)
        if goal_xy is None:
            return

        start = self.world_to_cell(grid_msg, *robot_xy)
        goal = self.world_to_cell(grid_msg, *goal_xy)
        if start is None or goal is None:
            self.get_logger().warn('Start or goal is outside the terrain grid.')
            return

        start = self.nearest_traversable(grid, start)
        goal = self.nearest_traversable(grid, goal)
        if start is None or goal is None:
            self.get_logger().warn('No traversable start/goal cell found near request.')
            return

        cells = self.astar(grid, start, goal)
        if not cells:
            self.get_logger().warn('OctoMap terrain planner could not find a route.')
            return

        path = self.cells_to_path(grid_msg, cells)
        self.plan_pub.publish(path)
        self.get_logger().info(
            f'Published OctoMap terrain plan with {len(path.poses)} poses.'
        )

        if self.send_to_nav2:
            self.send_waypoints(path)

    def lookup_robot_xy(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as exc:
            self.get_logger().warn(
                f'Waiting for {self.global_frame}->{self.base_frame}: {exc}',
                throttle_duration_sec=5.0,
            )
            return None

        translation = transform.transform.translation
        return float(translation.x), float(translation.y)

    def pose_xy_in_global_frame(self, pose_msg):
        frame_id = pose_msg.header.frame_id or self.global_frame
        x = float(pose_msg.pose.position.x)
        y = float(pose_msg.pose.position.y)
        if frame_id == self.global_frame:
            return x, y

        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                frame_id,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as exc:
            self.get_logger().warn(
                f'Cannot transform goal from {frame_id} to {self.global_frame}: {exc}'
            )
            return None

        transformed = self.transform_xy(x, y, transform)
        return transformed[0], transformed[1]

    @staticmethod
    def transform_xy(x, y, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            translation.x + x * cos_yaw - y * sin_yaw,
            translation.y + x * sin_yaw + y * cos_yaw,
        )

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

    def nearest_traversable(self, grid, cell):
        if self.is_traversable(grid, cell):
            return cell

        row, col = cell
        for radius in range(1, self.goal_search_radius_cells + 1):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if max(abs(dr), abs(dc)) != radius:
                        continue
                    candidate = row + dr, col + dc
                    if self.is_traversable(grid, candidate):
                        return candidate
        return None

    def is_traversable(self, grid, cell):
        row, col = cell
        if row < 0 or col < 0 or row >= grid.shape[0] or col >= grid.shape[1]:
            return False
        value = int(grid[row, col])
        return value >= 0 and value < self.lethal_threshold

    def astar(self, grid, start, goal):
        frontier = []
        heapq.heappush(frontier, (0.0, start))
        came_from = {start: None}
        cost_so_far = {start: 0.0}
        expansions = 0

        while frontier and expansions < self.max_expansions:
            _, current = heapq.heappop(frontier)
            expansions += 1

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor, step_cost in self.neighbors(grid, current):
                terrain_cost = max(0, int(grid[neighbor[0], neighbor[1]])) / 100.0
                new_cost = (
                    cost_so_far[current] +
                    step_cost * (1.0 + terrain_cost * self.cost_weight)
                )
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current
        return []

    def neighbors(self, grid, cell):
        row, col = cell
        for dr, dc, step_cost in (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ):
            neighbor = row + dr, col + dc
            if self.is_traversable(grid, neighbor):
                yield neighbor, step_cost

    @staticmethod
    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def reconstruct_path(came_from, current):
        path = [current]
        while came_from[current] is not None:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def cells_to_path(self, grid_msg, cells):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.global_frame

        resolution = float(grid_msg.info.resolution)
        stride = max(1, int(round(self.waypoint_spacing / resolution)))
        selected = cells[::stride]
        if selected[-1] != cells[-1]:
            selected.append(cells[-1])

        for index, cell in enumerate(selected):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = self.cell_center(
                grid_msg,
                cell,
            )
            pose.pose.position.z = 0.0

            next_cell = selected[min(index + 1, len(selected) - 1)]
            next_x, next_y = self.cell_center(grid_msg, next_cell)
            yaw = math.atan2(
                next_y - pose.pose.position.y,
                next_x - pose.pose.position.x,
            )
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            path.poses.append(pose)
        return path

    @staticmethod
    def cell_center(grid_msg, cell):
        row, col = cell
        resolution = float(grid_msg.info.resolution)
        origin = grid_msg.info.origin.position
        return (
            origin.x + (col + 0.5) * resolution,
            origin.y + (row + 0.5) * resolution,
        )

    def send_waypoints(self, path):
        if len(path.poses) == 0:
            return

        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Nav2 NavigateThroughPoses action is not available.')
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = path.poses[1:] if len(path.poses) > 1 else path.poses
        goal.behavior_tree = ''
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Failed to send OctoMap waypoints: {exc}')
            return
        if not goal_handle.accepted:
            self.get_logger().warn('OctoMap waypoint goal rejected by Nav2.')
            return
        self.get_logger().info('OctoMap waypoint goal accepted by Nav2.')


def main(args=None):
    rclpy.init(args=args)
    node = OctomapGoalPlanner()
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
