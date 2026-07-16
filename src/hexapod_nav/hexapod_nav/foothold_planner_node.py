#!/usr/bin/env python3
"""
foothold_planner_node.py

For each leg:
  1. Query terrain costmap for reachable zone
  2. Select lowest-cost reachable cell as foothold target
  3. Publish target
  4. On every subsequent costmap update, re-check committed target cost
  5. If cost delta > threshold, replan mid-swing (publish new target)

This implements the dynamic replanning loop that is the core research contribution.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PointStamped
from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from . import kinematics as kin


NUM_LEGS = 6

# Swing phase enum
STANCE = 0
SWING  = 1


class FootholdPlannerNode(Node):
    def __init__(self):
        super().__init__('foothold_planner_node')

        # Parameters
        self.declare_parameter('replan_cost_threshold', 0.3)  # Cost delta to trigger replan
        self.declare_parameter('nominal_stance_height', -0.12)  # z of foot in body frame (m)
        self.declare_parameter('aep_forward_offset', 0.05)   # Anterior Extreme Position offset (m)
        self.declare_parameter('pep_backward_offset', 0.05)  # Posterior Extreme Position offset (m)

        self.replan_thresh = self.get_parameter('replan_cost_threshold').value
        self.stance_z      = self.get_parameter('nominal_stance_height').value
        self.aep_offset    = self.get_parameter('aep_forward_offset').value
        self.pep_offset    = self.get_parameter('pep_backward_offset').value

        # State
        self.leg_phase      = [STANCE] * NUM_LEGS
        self.committed_targets = [None] * NUM_LEGS   # (ix, iy, cost_at_commit)
        self.costmap = None
        self.costmap_meta = None
        self.body_pose = np.array([0.0, 0.0, 0.12, 0.0])  # x, y, z, yaw

        # Publishers — one per leg
        self.target_pubs = [
            self.create_publisher(PointStamped, f'/leg_{i}/foothold_target', 10)
            for i in range(NUM_LEGS)
        ]
        self.replan_pubs = [
            self.create_publisher(PointStamped, f'/leg_{i}/foothold_replan', 10)
            for i in range(NUM_LEGS)
        ]

        # Subscriptions
        self.create_subscription(GridMap, '/terrain_costmap', self.costmap_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info('Foothold planner node started')

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = 2 * np.arctan2(q.z, q.w)
        self.body_pose = np.array([p.x, p.y, p.z, yaw])

    def costmap_callback(self, msg: GridMap):
        """Runs every time terrain costmap updates."""
        self.costmap = self._extract_cost_array(msg)
        self.costmap_meta = msg.info

        # For every leg currently in SWING, check if committed target cost has changed
        for leg_idx in range(NUM_LEGS):
            if self.leg_phase[leg_idx] == SWING and self.committed_targets[leg_idx] is not None:
                ix, iy, committed_cost = self.committed_targets[leg_idx]
                current_cost = self._get_cost_at_cell(ix, iy)

                if current_cost == np.inf or (current_cost - committed_cost) > self.replan_thresh:
                    self.get_logger().info(
                        f'Leg {leg_idx}: cost delta {current_cost - committed_cost:.2f} > threshold. Replanning.'
                    )
                    self._select_and_publish_foothold(leg_idx, replan=True)

    def begin_swing(self, leg_idx: int):
        """Called by gait controller when leg enters swing phase."""
        self.leg_phase[leg_idx] = SWING
        self._select_and_publish_foothold(leg_idx, replan=False)

    def end_swing(self, leg_idx: int):
        """Called by gait controller when leg touches down."""
        self.leg_phase[leg_idx] = STANCE
        self.committed_targets[leg_idx] = None

    def _select_and_publish_foothold(self, leg_idx: int, replan: bool):
        if self.costmap is None or self.costmap_meta is None:
            return

        meta = self.costmap_meta
        res  = meta.resolution
        cx   = meta.pose.position.x
        cy   = meta.pose.position.y
        n    = self.costmap.shape[0]

        # Build world-frame positions for all grid cells
        xs = cx - meta.length_x/2 + (np.arange(n) + 0.5) * res
        ys = cy - meta.length_y/2 + (np.arange(n) + 0.5) * res
        xx, yy = np.meshgrid(xs, ys, indexing='ij')
        zz = np.zeros_like(xx)  # floor heights — ideally read from terrain grid_map

        candidates = np.stack([xx.flatten(), yy.flatten(), zz.flatten()], axis=1)

        # Mask to reachable cells
        reachable = kin.get_reachable_zone(leg_idx, self.body_pose, candidates)

        # AEP/PEP workspace further restricts candidates
        # (foothold must be ahead of PEP and behind AEP in leg frame)
        # — implement per your gait geometry

        cost_flat = self.costmap.flatten()
        cost_flat[~reachable] = np.inf

        best_idx = np.argmin(cost_flat)
        if cost_flat[best_idx] == np.inf:
            self.get_logger().warn(f'Leg {leg_idx}: no valid foothold found')
            return

        best_ix = best_idx // n
        best_iy = best_idx % n
        best_cost = cost_flat[best_idx]

        self.committed_targets[leg_idx] = (best_ix, best_iy, best_cost)

        # Publish target
        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = 'map'
        pt.point.x = candidates[best_idx, 0]
        pt.point.y = candidates[best_idx, 1]
        pt.point.z = candidates[best_idx, 2]

        pub = self.replan_pubs[leg_idx] if replan else self.target_pubs[leg_idx]
        pub.publish(pt)

    def _get_cost_at_cell(self, ix: int, iy: int) -> float:
        if self.costmap is None:
            return np.inf
        n = self.costmap.shape[0]
        if 0 <= ix < n and 0 <= iy < n:
            return float(self.costmap[ix, iy])
        return np.inf

    def _extract_cost_array(self, msg: GridMap) -> np.ndarray:
        data = msg.data[0]
        n = data.layout.dim[0].size
        return np.array(data.data, dtype=np.float32).reshape((n, n), order='F')


def main(args=None):
    rclpy.init(args=args)
    node = FootholdPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()