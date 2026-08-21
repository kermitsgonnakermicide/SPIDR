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
from geometry_msgs.msg import PointStamped, Point
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from . import kinematics as kin


NUM_LEGS = 6

# Swing phase enum
STANCE = 0
SWING  = 1

# Distinct colors per leg (RGBA 0-1)
LEG_COLORS = [
    (1.0, 0.2, 0.2, 0.95),  # RF red
    (1.0, 0.6, 0.1, 0.95),  # RM orange
    (1.0, 1.0, 0.2, 0.95),  # RR yellow
    (0.2, 1.0, 0.3, 0.95),  # LF green
    (0.2, 0.6, 1.0, 0.95),  # LM blue
    (0.7, 0.3, 1.0, 0.95),  # LR purple
]
LEG_NAMES = ['RF', 'RM', 'RR', 'LF', 'LM', 'LR']


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
        self.terrain_grid = None
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
        # Visualization of dynamic foothold placement (targets + lines to body)
        self.marker_pub = self.create_publisher(MarkerArray, '/foothold_markers', 10)
        self._target_points = [None] * NUM_LEGS  # last published world Point
        self._target_is_replan = [False] * NUM_LEGS

        # Subscriptions
        self.create_subscription(GridMap, '/terrain_costmap', self.costmap_callback, 10)
        self.create_subscription(GridMap, '/terrain_grid_map', self.terrain_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(UInt8MultiArray, '/leg_phase', self.phase_callback, 10)

        self.create_timer(0.1, self._publish_markers)
        self.get_logger().info('Foothold planner node started')

    def terrain_callback(self, msg: GridMap):
        """Cache terrain grid map for floor height lookups."""
        self.terrain_grid = msg

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = 2 * np.arctan2(q.z, q.w)
        self.body_pose = np.array([p.x, p.y, p.z, yaw])

    def phase_callback(self, msg: UInt8MultiArray):
        """Receive leg phase from gait controller. Detect transitions."""
        if len(msg.data) != NUM_LEGS:
            return
        for leg_idx in range(NUM_LEGS):
            new_phase = msg.data[leg_idx]
            old_phase = self.leg_phase[leg_idx]
            if old_phase == STANCE and new_phase == SWING:
                self.begin_swing(leg_idx)
            elif old_phase == SWING and new_phase == STANCE:
                self.end_swing(leg_idx)

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
        self._target_points[leg_idx] = None
        self._target_is_replan[leg_idx] = False

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

        # Read floor heights from terrain grid_map if available
        zz = np.full_like(xx, self.stance_z)
        if self.terrain_grid is not None:
            floor_data = self._extract_terrain_layer(self.terrain_grid, 'floor')
            if floor_data is not None and floor_data.shape == xx.shape:
                zz = np.nan_to_num(floor_data, nan=self.stance_z)

        candidates = np.stack([xx.flatten(), yy.flatten(), zz.flatten()], axis=1)

        # Mask to reachable cells
        reachable = kin.get_reachable_zone(leg_idx, self.body_pose, candidates)

        # AEP/PEP workspace restriction: transform each candidate to coxa frame
        # and reject if forward distance is outside [PEP, AEP] range
        aep_mask = self._aep_pep_filter(candidates, leg_idx)
        reachable &= aep_mask

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

        self._target_points[leg_idx] = pt.point
        self._target_is_replan[leg_idx] = replan

        pub = self.replan_pubs[leg_idx] if replan else self.target_pubs[leg_idx]
        pub.publish(pt)
        self._publish_markers()

    def _publish_markers(self):
        """RViz MarkerArray: spheres at foothold targets + lines from body."""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        bx, by, bz, _ = self.body_pose

        # Delete-all first so stale markers disappear when targets clear
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = stamp
        clear.ns = 'footholds'
        clear.id = 0
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for i in range(NUM_LEGS):
            pt = self._target_points[i]
            if pt is None:
                continue
            r, g, b, a = LEG_COLORS[i]
            # Mid-swing replan → white ring cue via higher scale / brighter
            scale = 0.035 if self._target_is_replan[i] else 0.025

            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = stamp
            sphere.ns = 'footholds'
            sphere.id = i + 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = pt.x
            sphere.pose.position.y = pt.y
            sphere.pose.position.z = pt.z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = scale
            sphere.color = ColorRGBA(r=r, g=g, b=b, a=a)
            sphere.lifetime.sec = 0
            ma.markers.append(sphere)

            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp = stamp
            line.ns = 'foothold_rays'
            line.id = i + 1
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.008
            line.color = ColorRGBA(r=r, g=g, b=b, a=0.6)
            line.points = [
                Point(x=bx, y=by, z=bz),
                Point(x=pt.x, y=pt.y, z=pt.z),
            ]
            ma.markers.append(line)

            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = stamp
            text.ns = 'foothold_labels'
            text.id = i + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pt.x
            text.pose.position.y = pt.y
            text.pose.position.z = pt.z + 0.04
            text.pose.orientation.w = 1.0
            text.scale.z = 0.03
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            tag = 'REPLAN' if self._target_is_replan[i] else 'TGT'
            text.text = f'{LEG_NAMES[i]} {tag}'
            ma.markers.append(text)

        self.marker_pub.publish(ma)

    def _aep_pep_filter(self, candidates: np.ndarray, leg_idx: int) -> np.ndarray:
        """
        Filter candidates by AEP/PEP workspace bounds in the leg's coxa frame.

        AEP (Anterior Extreme Position): forward limit of step workspace
        PEP (Posterior Extreme Position): backward limit of step workspace
        Candidates must have their coxa-frame x-coordinate between PEP and AEP.
        """
        mask = np.ones(len(candidates), dtype=bool)
        bx, by, bz, byaw = self.body_pose

        cos_y = np.cos(-byaw)
        sin_y = np.sin(-byaw)
        origin = kin.LEG_ORIGINS[leg_idx]
        angle = kin.LEG_ANGLES[leg_idx]
        cos_a = np.cos(-angle)
        sin_a = np.sin(-angle)

        for i, wp in enumerate(candidates):
            # World → body frame
            dx = wp[0] - bx
            dy = wp[1] - by
            body_x = dx * cos_y - dy * sin_y
            body_y = dx * sin_y + dy * cos_y
            body_z = wp[2]

            # Body → leg frame
            tx = body_x - origin[0]
            ty = body_y - origin[1]
            tz = body_z - origin[2]
            coxa_x = tx * cos_a - ty * sin_a

            # coxa_x is forward distance in leg frame
            # PEP: minimum forward distance, AEP: maximum forward distance
            pep_limit = kin.COXA_LEN + self.pep_offset
            aep_limit = kin.COXA_LEN + kin.FEMUR_LEN + kin.TIBIA_LEN - self.aep_offset

            if coxa_x < pep_limit or coxa_x > aep_limit:
                mask[i] = False

        return mask

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

    def _extract_terrain_layer(self, msg: GridMap, layer_name: str) -> np.ndarray:
        """Extract a named layer from a GridMap message."""
        try:
            idx = msg.layers.index(layer_name)
        except ValueError:
            return None
        data = msg.data[idx]
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