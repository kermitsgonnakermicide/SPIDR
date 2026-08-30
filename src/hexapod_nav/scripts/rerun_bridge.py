#!/usr/bin/env python3
"""rerun_bridge.py - Full RViz replacement using Rerun.io (v2 with error handling)"""
import math, os, subprocess, traceback, numpy as np
import rclpy
from rclpy.node import Node
import rerun as rr
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, PolygonStamped
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float64MultiArray, UInt8MultiArray
from visualization_msgs.msg import MarkerArray
from tf2_msgs.msg import TFMessage
import sensor_msgs_py.point_cloud2 as pc2

URDF_PATH = '/tmp/spooder_rerun.urdf'


def _ensure_urdf():
    if os.path.exists(URDF_PATH):
        return URDF_PATH
    candidates = [
        os.path.join(os.path.expanduser('~'), 'spooder_ws', 'src',
                     'spooder_description', 'urdf', 'spooder.xacro'),
        '/tmp/spooder.urdf',
    ]
    for src in candidates:
        if not os.path.exists(src):
            continue
        if src.endswith('.xacro'):
            try:
                subprocess.run(['xacro', src, '-o', URDF_PATH],
                               timeout=10, capture_output=True)
                if os.path.exists(URDF_PATH):
                    return URDF_PATH
            except Exception:
                pass
        else:
            return src
    return None


def _stamp(msg):
    s = msg.header.stamp
    rr.set_time('ros', duration=s.sec + s.nanosec * 1e-9)


def _parse_pc2(msg):
    pts = []
    for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
        if math.isfinite(p[0]) and math.isfinite(p[1]) and math.isfinite(p[2]):
            pts.append([p[0], p[1], p[2]])
    return np.array(pts, dtype=np.float32) if pts else None


def _occgrid_img(msg):
    w, h = msg.info.width, msg.info.height
    data = np.array(msg.data, dtype=np.int8).reshape((h, w))
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[data == -1] = [128, 128, 128]
    img[(data >= 0) & (data <= 50)] = [200, 200, 200]
    img[data > 50] = [40, 40, 40]
    return img


def _gridmap_layer(grid_msg, layer_name):
    try:
        idx = grid_msg.layers.index(layer_name)
    except ValueError:
        return None
    dm = grid_msg.data[idx]
    n = dm.layout.dim[0].size
    return np.array(dm.data, dtype=np.float32).reshape((n, n), order='F')


def _heightmap_img(arr):
    arr = np.nan_to_num(arr, nan=0.0)
    vmin, vmax = float(np.min(arr)), float(np.max(arr))
    norm = (arr - vmin) / (vmax - vmin) if vmax - vmin > 1e-6 else np.zeros_like(arr)
    img = np.zeros((*arr.shape, 3), dtype=np.uint8)
    img[:, :, 0] = (norm * 255).astype(np.uint8)
    img[:, :, 1] = ((1.0 - norm) * 200).astype(np.uint8)
    img[:, :, 2] = 100
    return img


def _costmap_img(arr):
    arr = np.nan_to_num(arr, nan=0.0)
    vmin, vmax = float(np.min(arr)), float(np.max(arr))
    norm = (arr - vmin) / (vmax - vmin) if vmax - vmin > 1e-6 else np.zeros_like(arr)
    img = np.zeros((*arr.shape, 3), dtype=np.uint8)
    img[:, :, 0] = (255 - norm * 200).astype(np.uint8)
    img[:, :, 1] = (norm * 255).astype(np.uint8)
    img[:, :, 2] = 128
    return img

class RerunBridge(Node):
    def __init__(self):
        super().__init__('rerun_bridge')
        rr.init('spooder', spawn=True)
        rr.log('world', rr.ViewCoordinates.FLU, static=True)
        self.seq = 0
        self.topic_counts = {}
        urdf = _ensure_urdf()
        if urdf:
            rr.log_file_from_path(urdf, static=True, entity_path_prefix='')
            self.get_logger().info(f'URDF loaded: {urdf}')
        else:
            self.get_logger().warn('Could not find spooder URDF')
        self.create_subscription(TFMessage, '/tf', self._wrap('tf_cb', self.tf_cb), 500)
        self.create_subscription(LaserScan, '/scan', self._wrap('scan_cb', self.scan_cb), 10)
        self.create_subscription(PointCloud2, '/camera/points', self._wrap('pc_cloud_cb', self.pc_cloud_cb), 10)
        self.create_subscription(PointCloud2, '/octomap_point_cloud_centers', self._wrap('pc_octo_cb', self.pc_octo_cb), 10)
        self.create_subscription(Odometry, '/odom', self._wrap('odom_cb', self.odom_cb), 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._wrap('odom_filt_cb', self.odom_filt_cb), 10)
        self.create_subscription(Imu, '/imu', self._wrap('imu_cb', self.imu_cb), 50)
        self.create_subscription(Twist, '/cmd_vel', self._wrap('cmd_vel_cb', self.cmd_vel_cb), 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self._wrap('cmd_vel_nav_cb', self.cmd_vel_nav_cb), 10)
        self.create_subscription(Path, '/plan', self._wrap('plan_cb', self.plan_cb), 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._wrap('gcost_cb', self.gcost_cb), 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._wrap('lcost_cb', self.lcost_cb), 10)
        self.create_subscription(OccupancyGrid, '/projected_map', self._wrap('proj_map_cb', self.proj_map_cb), 10)
        self.create_subscription(OccupancyGrid, '/map', self._wrap('slam_map_cb', self.slam_map_cb), 10)
        self.create_subscription(GridMap, '/terrain_grid_map', self._wrap('terrain_grid_cb', self.terrain_grid_cb), 10)
        self.create_subscription(GridMap, '/terrain_costmap', self._wrap('terrain_cost_cb', self.terrain_cost_cb), 10)
        self.create_subscription(UInt8MultiArray, '/leg_phase', self._wrap('leg_phase_cb', self.leg_phase_cb), 10)
        self.create_subscription(Float64MultiArray, '/spooder_controller/commands', self._wrap('joint_cmds_cb', self.joint_cmds_cb), 10)
        self.create_subscription(MarkerArray, '/foothold_markers', self._wrap('foothold_arr_cb', self.foothold_arr_cb), 10)
        self.create_subscription(MarkerArray, '/gait_foot_markers', self._wrap('gait_foot_cb', self.gait_foot_cb), 10)
        self.create_subscription(MarkerArray, '/occupied_cells_vis_array', self._wrap('occupied_cb', self.occupied_cb), 10)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint', self._wrap('footprint_cb', self.footprint_cb), 10)
        self.create_subscription(PolygonStamped, '/global_costmap/published_footprint', self._wrap('gfootprint_cb', self.gfootprint_cb), 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._wrap('goal_cb', self.goal_cb), 10)
        for i in range(6):
            self.create_subscription(PointStamped, f'/leg_{i}/foothold_target',
                self._wrap(f'foothold_t{i}', lambda m, idx=i: self.foothold_pt_cb(m, idx, 'target')), 10)
            self.create_subscription(PointStamped, f'/leg_{i}/foothold_replan',
                self._wrap(f'foothold_r{i}', lambda m, idx=i: self.foothold_pt_cb(m, idx, 'replan')), 10)
        self.create_timer(5.0, self._status_report)
        self.get_logger().info('Rerun bridge v2 ready (error-handled)')

    def _wrap(self, name, fn):
        def wrapper(msg):
            self.topic_counts[name] = self.topic_counts.get(name, 0) + 1
            try:
                fn(msg)
            except Exception as e:
                self.get_logger().error(f'{name} failed: {e}\n{traceback.format_exc()}')
        return wrapper

    def _bump(self):
        self.seq += 1
        rr.set_time('tick', sequence=self.seq)

    def _status_report(self):
        active = [f'{k}({v})' for k, v in sorted(self.topic_counts.items()) if v > 0]
        self.get_logger().info(f'Topics active: {len(active)} | {", ".join(active[:15])}')

    def tf_cb(self, msg):
        self._bump()
        for tf in msg.transforms:
            t, q = tf.transform.translation, tf.transform.rotation
            rr.log(f'{tf.header.frame_id}/{tf.child_frame_id}',
                   rr.Transform3D(translation=[t.x, t.y, t.z],
                                  rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w])))

    def scan_cb(self, msg):
        self._bump(); _stamp(msg)
        n = len(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, n)
        r = np.array(msg.ranges, dtype=np.float32)
        mask = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        if np.any(mask):
            xs = r[mask] * np.cos(angles[mask])
            ys = r[mask] * np.sin(angles[mask])
            pts = np.column_stack([xs, ys, np.zeros(len(xs))])
            rr.log('world/laser_scan', rr.Points3D(positions=pts, colors=[255,80,80], radii=[0.008]))

    def pc_cloud_cb(self, msg):
        self._bump(); _stamp(msg)
        pts = _parse_pc2(msg)
        if pts is not None and len(pts) > 0:
            rr.log('world/camera_points', rr.Points3D(positions=pts, colors=[100,200,255], radii=[0.005]))

    def pc_octo_cb(self, msg):
        self._bump(); _stamp(msg)
        pts = _parse_pc2(msg)
        if pts is not None and len(pts) > 0:
            step = max(1, len(pts) // 8000)
            rr.log('world/octomap_pts', rr.Points3D(positions=pts[::step], colors=[255,80,80], radii=[0.008]))

    def _log_odom(self, msg, entity, color):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = 2 * math.atan2(o.z, o.w)
        rr.log(entity, rr.Arrows3D(origins=[[p.x, p.y, p.z]],
               vectors=[[0.15*math.cos(yaw), 0.15*math.sin(yaw), 0]],
               colors=[color], radii=[0.008]))
        v = msg.twist.twist
        prefix = entity.split('/')[-1]
        rr.log(f'odom/{prefix}_vx', rr.Scalars(v.linear.x))
        rr.log(f'odom/{prefix}_vy', rr.Scalars(v.linear.y))
        rr.log(f'odom/{prefix}_vz', rr.Scalars(v.angular.z))

    def odom_cb(self, msg):
        self._bump(); _stamp(msg); self._log_odom(msg, 'world/odom_raw', [255,85,255])

    def odom_filt_cb(self, msg):
        self._bump(); _stamp(msg); self._log_odom(msg, 'world/odom_filtered', [0,255,170])

    def imu_cb(self, msg):
        self._bump(); _stamp(msg)
        rr.log('imu/gyro_x', rr.Scalars(msg.angular_velocity.x))
        rr.log('imu/gyro_y', rr.Scalars(msg.angular_velocity.y))
        rr.log('imu/gyro_z', rr.Scalars(msg.angular_velocity.z))
        rr.log('imu/accel_x', rr.Scalars(msg.linear_acceleration.x))
        rr.log('imu/accel_y', rr.Scalars(msg.linear_acceleration.y))
        rr.log('imu/accel_z', rr.Scalars(msg.linear_acceleration.z))

    def cmd_vel_cb(self, msg):
        self._bump()
        rr.log('vel/cmd_x', rr.Scalars(msg.linear.x))
        rr.log('vel/cmd_y', rr.Scalars(msg.linear.y))
        rr.log('vel/cmd_yaw', rr.Scalars(msg.angular.z))
        rr.log('world/cmd_vel_arrow', rr.Arrows3D(origins=[[0,0,0.05]],
               vectors=[[msg.linear.x, msg.linear.y, 0]], colors=[0,220,255], radii=[0.012]))

    def cmd_vel_nav_cb(self, msg):
        self._bump()
        rr.log('vel/nav_x', rr.Scalars(msg.linear.x))
        rr.log('vel/nav_yaw', rr.Scalars(msg.angular.z))

    def plan_cb(self, msg):
        self._bump(); _stamp(msg)
        if msg.poses:
            pts = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
            rr.log('world/nav_plan', rr.LineStrips3D([pts], colors=[0,200,255], radii=[0.012]))

    def _log_occgrid(self, msg, entity):
        self._bump(); _stamp(msg)
        img = _occgrid_img(msg)
        rr.log(entity, rr.GridMap(data=img.tobytes(),
               format=rr.components.ImageFormat(width=img.shape[1], height=img.shape[0],
                                                color_model='RGB', channel_datatype='U8'),
               cell_size=rr.datatypes.Float32(msg.info.resolution),
               translation=[msg.info.origin.position.x, msg.info.origin.position.y, 0.0]))

    def gcost_cb(self, msg): self._log_occgrid(msg, 'map/global_costmap')
    def lcost_cb(self, msg): self._log_occgrid(msg, 'map/local_costmap')
    def proj_map_cb(self, msg): self._log_occgrid(msg, 'map/projected')
    def slam_map_cb(self, msg): self._log_occgrid(msg, 'map/slam')

    def terrain_grid_cb(self, msg):
        self._bump(); _stamp(msg)
        ox = msg.info.pose.position.x - msg.info.length_x / 2
        oy = msg.info.pose.position.y - msg.info.length_y / 2
        for layer, entity in [('floor', 'map/terrain_floor'), ('clearance', 'map/terrain_clearance')]:
            arr = _gridmap_layer(msg, layer)
            if arr is not None:
                arr = arr[::-1, ::-1]
                img = _heightmap_img(arr.T)
                rr.log(entity, rr.GridMap(data=img.tobytes(),
                       format=rr.components.ImageFormat(width=img.shape[1], height=img.shape[0],
                                                        color_model='RGB', channel_datatype='U8'),
                       cell_size=rr.datatypes.Float32(msg.info.resolution),
                       translation=[ox, oy, 0.0]))

    def terrain_cost_cb(self, msg):
        self._bump(); _stamp(msg)
        ox = msg.info.pose.position.x - msg.info.length_x / 2
        oy = msg.info.pose.position.y - msg.info.length_y / 2
        arr = _gridmap_layer(msg, 'cost')
        if arr is not None:
            arr = arr[::-1, ::-1]
            img = _costmap_img(arr.T)
            rr.log('map/terrain_cost', rr.GridMap(data=img.tobytes(),
                   format=rr.components.ImageFormat(width=img.shape[1], height=img.shape[0],
                                                    color_model='RGB', channel_datatype='U8'),
                   cell_size=rr.datatypes.Float32(msg.info.resolution),
                   translation=[ox, oy, 0.0]))

    def leg_phase_cb(self, msg):
        self._bump()
        for i, phase in enumerate(msg.data):
            rr.log(f'gait/leg_{i}_phase', rr.Scalars(float(phase)))

    def joint_cmds_cb(self, msg):
        self._bump()
        names = ['rf_coxa','rf_femur','rf_tibia','rm_coxa','rm_femur','rm_tibia',
                 'rr_coxa','rr_femur','rr_tibia','lf_coxa','lf_femur','lf_tibia',
                 'lm_coxa','lm_femur','lm_tibia','lr_coxa','lr_femur','lr_tibia']
        for i, val in enumerate(msg.data):
            if i < len(names):
                rr.log(f'joints/{names[i]}', rr.Scalars(val))

    def foothold_arr_cb(self, msg):
        self._bump()
        all_pts, all_colors = [], []
        for m in msg.markers:
            if m.action != 0:
                continue
            for pt in m.points:
                all_pts.append([pt.x, pt.y, pt.z])
                c = [m.color.r*255, m.color.g*255, m.color.b*255] if m.color.a > 0 else [0,255,100]
                all_colors.append(c)
        if all_pts:
            rr.log('map/foothold_targets', rr.Points3D(positions=all_pts, colors=all_colors, radii=[0.012]))

    def gait_foot_cb(self, msg):
        self._bump()
        all_pts, all_colors = [], []
        for m in msg.markers:
            if m.action != 0:
                continue
            all_pts.append([m.pose.position.x, m.pose.position.y, m.pose.position.z])
            c = [m.color.r*255, m.color.g*255, m.color.b*255] if m.color.a > 0 else [255,255,255]
            all_colors.append(c)
        if all_pts:
            rr.log('map/gait_feet', rr.Points3D(positions=all_pts, colors=all_colors, radii=[0.015]))

    def occupied_cb(self, msg):
        self._bump()
        all_pts = []
        for m in msg.markers:
            if m.action != 0:
                continue
            if m.type == 8 and m.points:
                for pt in m.points:
                    all_pts.append([pt.x, pt.y, pt.z])
            elif m.pose and (m.pose.position.x != 0 or m.pose.position.y != 0):
                if m.scale.x > 0:
                    all_pts.append([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        if all_pts:
            step = max(1, len(all_pts) // 5000)
            rr.log('map/octomap_cells', rr.Points3D(positions=all_pts[::step], colors=[200,80,80], radii=[0.015]))

    def footprint_cb(self, msg):
        self._bump(); _stamp(msg)
        pts = [[pt.x, pt.y, 0.0] for pt in msg.polygon.points]
        if pts:
            pts.append(pts[0])
            rr.log('map/footprint_local', rr.LineStrips3D([pts], colors=[0,200,255], radii=[0.008]))

    def gfootprint_cb(self, msg):
        self._bump(); _stamp(msg)
        pts = [[pt.x, pt.y, 0.0] for pt in msg.polygon.points]
        if pts:
            pts.append(pts[0])
            rr.log('map/footprint_global', rr.LineStrips3D([pts], colors=[200,200,0], radii=[0.008]))

    def goal_cb(self, msg):
        self._bump(); _stamp(msg)
        p = msg.pose.position
        o = msg.pose.orientation
        yaw = 2 * math.atan2(o.z, o.w)
        rr.log('world/goal_pose', rr.Arrows3D(origins=[[p.x, p.y, p.z]],
               vectors=[[0.2*math.cos(yaw), 0.2*math.sin(yaw), 0]],
               colors=[255,50,50], radii=[0.015]))

    def foothold_pt_cb(self, msg, leg_idx, kind):
        self._bump(); _stamp(msg)
        p = msg.point
        names = ['rf','rm','rr','lf','lm','lr']
        color = [0,255,100] if kind == 'target' else [255,200,0]
        rr.log(f'world/foothold_{kind}_{names[leg_idx]}',
               rr.Points3D(positions=[[p.x, p.y, p.z]], colors=[color], radii=[0.015]))

def main(args=None):
    rclpy.init(args=args)
    node = RerunBridge()
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
