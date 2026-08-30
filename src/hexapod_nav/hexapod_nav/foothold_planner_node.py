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
        self.body_pose = np.array([p.x, p.y, 0.154, yaw])

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

        # GridMap convention: index 0 = positive corner (getPositionFromIndex
        # uses -index transform), so xs[0] must be the positive end.
        half_x = meta.length_x / 2
        half_y = meta.length_y / 2
        xs = cx + half_x - (np.arange(n) + 0.5) * res
        ys = cy + half_y - (np.arange(n) + 0.5) * res
        xx, yy = np.meshgrid(xs, ys, indexing='ij')

        # Read floor heights from terrain grid_map if available
        zz = np.full_like(xx, self.stance_z)
        if self.terrain_grid is not None:
            floor_data = self._extract_terrain_layer(self.terrain_grid, 'floor')
            if floor_data is not None and floor_data.shape == xx.shape:
                zz = np.nan_to_num(floor_data, nan=self.stance_z)

        candidates = np.stack([xx.flatten(), yy.flatten(), zz.flatten()], axis=1)

        cost_flat = self.costmap.flatten()
        n_total = len(cost_flat)
        n_finite_cost = int(np.sum(np.isfinite(cost_flat)))
        n_nan_floor = int(np.sum(np.isnan(self.costmap)))

        # Mask to reachable cells
        reachable = kin.get_reachable_zone(leg_idx, self.body_pose, candidates)

        # AEP/PEP workspace restriction
        aep_mask = self._aep_pep_filter(candidates, leg_idx)

        reachable_and_aep = reachable & aep_mask
        n_pass = int(np.sum(reachable_and_aep))

        cost_flat[~reachable_and_aep] = np.inf

        best_idx = np.argmin(cost_flat)
        if cost_flat[best_idx] == np.inf:
            fallback = self._default_stance_world(leg_idx)
            if fallback is not None:
                pt = PointStamped()
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.header.frame_id = 'map'
                pt.point.x = float(fallback[0])
                pt.point.y = float(fallback[1])
                pt.point.z = float(fallback[2])
                self._target_points[leg_idx] = pt.point
                self._target_is_replan[leg_idx] = replan
                pub = self.replan_pubs[leg_idx] if replan else self.target_pubs[leg_idx]
                pub.publish(pt)
                self._publish_markers()
                self.get_logger().warn(
                    f'Leg {leg_idx}: no valid foothold, using default stance fallback | '
                    f'costmap: {n_finite_cost}/{n_total} finite, '
                    f'reachable+aep: {n_pass}/{n_total}')
            else:
                self.get_logger().warn(
                    f'Leg {leg_idx}: no valid foothold, no fallback | '
                    f'costmap: {n_finite_cost}/{n_total} finite, '
                    f'reachable+aep: {n_pass}/{n_total}, '
                    f'body_z={self.body_pose[2]:.3f}, '
                    f'costmap_origin=({cx:.2f},{cy:.2f}), '
                    f'robot=({self.body_pose[0]:.2f},{self.body_pose[1]:.2f})')
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
        """RViz MarkerArray: swing trajectory arcs, landing zones, current foot + target spheres."""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        bx, by, bz, _ = self.body_pose

        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = stamp
        clear.ns = 'footholds'
        clear.id = 0
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        swing_height = 0.06

        for i in range(NUM_LEGS):
            pt = self._target_points[i]
            if pt is None:
                continue
            r, g, b, a = LEG_COLORS[i]
            is_replan = self._target_is_replan[i]

            stance = self._default_stance_world(i)
            if stance is None:
                stance = np.array([bx, by, bz - 0.12])

            # --- Current foot position (small dim sphere) ---
            cur = Marker()
            cur.header.frame_id = 'map'
            cur.header.stamp = stamp
            cur.ns = 'current_feet'
            cur.id = i
            cur.type = Marker.SPHERE
            cur.action = Marker.ADD
            cur.pose.position.x = float(stance[0])
            cur.pose.position.y = float(stance[1])
            cur.pose.position.z = float(stance[2])
            cur.pose.orientation.w = 1.0
            cur.scale.x = cur.scale.y = cur.scale.z = 0.018
            cur.color = ColorRGBA(r=r, g=g, b=b, a=0.35)
            ma.markers.append(cur)

            # --- Target sphere (larger, bright) ---
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = stamp
            sphere.ns = 'foothold_targets'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = pt.x
            sphere.pose.position.y = pt.y
            sphere.pose.position.z = pt.z
            sphere.pose.orientation.w = 1.0
            s = 0.040 if is_replan else 0.028
            sphere.scale.x = sphere.scale.y = sphere.scale.z = s
            sphere.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
            ma.markers.append(sphere)

            # --- Landing zone disc (flat ring on ground) ---
            disc = Marker()
            disc.header.frame_id = 'map'
            disc.header.stamp = stamp
            disc.ns = 'landing_zones'
            disc.id = i
            disc.type = Marker.CYLINDER
            disc.action = Marker.ADD
            disc.pose.position.x = pt.x
            disc.pose.position.y = pt.y
            disc.pose.position.z = pt.z - 0.003
            disc.pose.orientation.w = 1.0
            disc.scale.x = disc.scale.y = 0.06
            disc.scale.z = 0.003
            disc.color = ColorRGBA(r=r, g=g, b=b, a=0.25)
            ma.markers.append(disc)

            # --- Swing trajectory arc (cubic Bezier from current to target) ---
            arc = Marker()
            arc.header.frame_id = 'map'
            arc.header.stamp = stamp
            arc.ns = 'swing_arcs'
            arc.id = i
            arc.type = Marker.LINE_STRIP
            arc.action = Marker.ADD
            arc.scale.x = 0.005
            arc.color = ColorRGBA(r=r, g=g, b=b, a=0.55 if not is_replan else 0.75)

            p0 = stance
            p3 = np.array([pt.x, pt.y, pt.z])
            p1 = p0 + np.array([0.0, 0.0, swing_height])
            p2 = p3 + np.array([0.0, 0.0, swing_height])

            n_arc = 16
            for j in range(n_arc + 1):
                t = j / n_arc
                t1 = 1.0 - t
                x = t1**3 * p0[0] + 3 * t1**2 * t * p1[0] + 3 * t1 * t**2 * p2[0] + t**3 * p3[0]
                y = t1**3 * p0[1] + 3 * t1**2 * t * p1[1] + 3 * t1 * t**2 * p2[1] + t**3 * p3[1]
                z = t1**3 * p0[2] + 3 * t1**2 * t * p1[2] + 3 * t1 * t**2 * p2[2] + t**3 * p3[2]
                arc.points.append(Point(x=float(x), y=float(y), z=float(z)))
            ma.markers.append(arc)

            # --- Vertical drop line from arc peak to ground ---
            peak_t = 0.5
            t1 = 0.5
            peak_x = t1**3 * p0[0] + 3 * t1**2 * 0.5 * p1[0] + 3 * t1 * 0.25 * p2[0] + 0.125 * p3[0]
            peak_y = t1**3 * p0[1] + 3 * t1**2 * 0.5 * p1[1] + 3 * t1 * 0.25 * p2[1] + 0.125 * p3[1]
            peak_z = t1**3 * p0[2] + 3 * t1**2 * 0.5 * p1[2] + 3 * t1 * 0.25 * p2[2] + 0.125 * p3[2]

            drop = Marker()
            drop.header.frame_id = 'map'
            drop.header.stamp = stamp
            drop.ns = 'swing_peaks'
            drop.id = i
            drop.type = Marker.SPHERE
            drop.action = Marker.ADD
            drop.pose.position.x = float(peak_x)
            drop.pose.position.y = float(peak_y)
            drop.pose.position.z = float(peak_z)
            drop.pose.orientation.w = 1.0
            drop.scale.x = drop.scale.y = drop.scale.z = 0.012
            drop.color = ColorRGBA(r=min(r + 0.3, 1.0), g=min(g + 0.3, 1.0),
                                   b=min(b + 0.3, 1.0), a=0.7)
            ma.markers.append(drop)

            # --- Text label ---
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = stamp
            text.ns = 'foothold_labels'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pt.x
            text.pose.position.y = pt.y
            text.pose.position.z = pt.z + 0.05
            text.pose.orientation.w = 1.0
            text.scale.z = 0.025
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            tag = 'RPLN' if is_replan else 'TGT'
            text.text = f'{LEG_NAMES[i]} {tag}'
            ma.markers.append(text)

        self.marker_pub.publish(ma)

    def _default_stance_world(self, leg_idx: int):
        """Return the default standing position for this leg in world frame."""
        stance_x = 0.12
        stance_z = -0.11
        default_coxa = np.array([stance_x, 0.0, stance_z])
        try:
            return kin.coxa_to_world(default_coxa, leg_idx, self.body_pose)
        except Exception:
            return None

    def _aep_pep_filter(self, candidates: np.ndarray, leg_idx: int) -> np.ndarray:
        """
        Filter candidates by AEP/PEP workspace bounds in the leg's coxa frame.

        AEP (Anterior Extreme Position): forward limit of step workspace
        PEP (Posterior Extreme Position): backward limit of step workspace
        Candidates must have their coxa-frame x-coordinate between PEP and AEP.
        """
        bx, by, bz, byaw = self.body_pose
        origin = kin.LEG_ORIGINS[leg_idx]
        angle = kin.LEG_ANGLES[leg_idx]

        cos_y = np.cos(-byaw)
        sin_y = np.sin(-byaw)
        cos_a = np.cos(-angle)
        sin_a = np.sin(-angle)

        dx = candidates[:, 0] - bx
        dy = candidates[:, 1] - by
        body_x = dx * cos_y - dy * sin_y
        body_y = dx * sin_y + dy * cos_y

        tx = body_x - origin[0]
        ty = body_y - origin[1]
        coxa_x = tx * cos_a - ty * sin_a

        pep_limit = kin.COXA_LEN + self.pep_offset
        aep_limit = kin.COXA_LEN + kin.FEMUR_LEN + kin.TIBIA_LEN - self.aep_offset

        return (coxa_x >= pep_limit) & (coxa_x <= aep_limit)

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