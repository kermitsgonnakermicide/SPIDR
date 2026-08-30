#!/usr/bin/env python3
"""
robot_state_viz.py

Publishes RViz markers showing the robot's current physical state:
  - Heading arrow on the ground plane
  - Elevation text above the robot
  - Tilt disc (flat circle that rolls/pitches with the robot body)
  - Ground projection shadow

Subscribes to /odom (or EKF-filtered odometry) to get the full 3D pose.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class RobotStateVizNode(Node):
    def __init__(self):
        super().__init__('robot_state_viz')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('base_frame', 'spooder/base_footprint')
        self.declare_parameter('wheelbase', 0.22)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.wheelbase = self.get_parameter('wheelbase').value

        self.sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_cb, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/robot_state_markers', 10)

        self.timer = self.create_timer(0.1, self._publish)

        self.pose = None
        self.get_logger().info('Robot state viz started')

    def odom_cb(self, msg: Odometry):
        self.pose = msg.pose.pose

    def _publish(self):
        if self.pose is None:
            return

        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        p = self.pose.position
        q = self.pose.orientation

        # Extract euler angles
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x**2 + q.y**2)
        roll = math.atan2(sinr, cosr)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y**2 + q.z**2)
        yaw = math.atan2(siny, cosy)

        # ── 1. Heading arrow on ground plane ──
        arrow_len = self.wheelbase * 1.2
        arrow = Marker()
        arrow.header.frame_id = 'map'
        arrow.header.stamp = stamp
        arrow.ns = 'robot_state'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = arrow_len
        arrow.scale.y = 0.015
        arrow.scale.z = 0.008
        arrow.pose.position.x = p.x
        arrow.pose.position.y = p.y
        arrow.pose.position.z = 0.005
        arrow.pose.orientation.w = math.cos(yaw / 2.0)
        arrow.pose.orientation.z = math.sin(yaw / 2.0)
        arrow.color = ColorRGBA(r=0.1, g=0.7, b=1.0, a=0.85)
        ma.markers.append(arrow)

        # ── 2. Ground projection shadow (flat ring at z=0) ──
        shadow = Marker()
        shadow.header.frame_id = 'map'
        shadow.header.stamp = stamp
        shadow.ns = 'robot_state'
        shadow.id = 1
        shadow.type = Marker.CYLINDER
        shadow.action = Marker.ADD
        shadow.pose.position.x = p.x
        shadow.pose.position.y = p.y
        shadow.pose.position.z = 0.003
        shadow.pose.orientation.w = 1.0
        shadow.scale.x = self.wheelbase * 1.8
        shadow.scale.y = self.wheelbase * 1.8
        shadow.scale.z = 0.003
        shadow.color = ColorRGBA(r=0.1, g=0.7, b=1.0, a=0.15)
        ma.markers.append(shadow)

        # ── 3. Tilt disc (rolls/pitches with the robot body) ──
        disc = Marker()
        disc.header.frame_id = 'map'
        disc.header.stamp = stamp
        disc.ns = 'robot_state'
        disc.id = 2
        disc.type = Marker.CYLINDER
        disc.action = Marker.ADD
        disc.pose.position.x = p.x
        disc.pose.position.y = p.y
        disc.pose.position.z = p.z
        # Apply roll, pitch, yaw as quaternion
        cr = math.cos(roll / 2.0)
        sr = math.sin(roll / 2.0)
        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)
        cy = math.cos(yaw / 2.0)
        sy = math.sin(yaw / 2.0)
        disc.pose.orientation.w = cr * cp * cy + sr * sp * sy
        disc.pose.orientation.x = sr * cp * cy - cr * sp * sy
        disc.pose.orientation.y = cr * sp * cy + sr * cp * sy
        disc.pose.orientation.z = cr * cp * sy - sr * sp * cy
        disc.scale.x = self.wheelbase * 1.6
        disc.scale.y = self.wheelbase * 1.4
        disc.scale.z = 0.008
        disc.color = ColorRGBA(r=0.2, g=1.0, b=0.4, a=0.25)
        ma.markers.append(disc)

        # ── 4. Vertical altitude line (ground to body) ──
        alt = Marker()
        alt.header.frame_id = 'map'
        alt.header.stamp = stamp
        alt.ns = 'robot_state'
        alt.id = 3
        alt.type = Marker.LINE_STRIP
        alt.action = Marker.ADD
        alt.scale.x = 0.004
        alt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.5)
        alt.points = [
            Point(x=p.x, y=p.y, z=0.0),
            Point(x=p.x, y=p.y, z=p.z),
        ]
        ma.markers.append(alt)

        # ── 5. Elevation text ──
        elev = Marker()
        elev.header.frame_id = 'map'
        elev.header.stamp = stamp
        elev.ns = 'robot_state'
        elev.id = 4
        elev.type = Marker.TEXT_VIEW_FACING
        elev.action = Marker.ADD
        elev.pose.position.x = p.x + 0.08
        elev.pose.position.y = p.y
        elev.pose.position.z = max(p.z, 0.0) + 0.06
        elev.pose.orientation.w = 1.0
        elev.scale.z = 0.03
        elev.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        elev.text = f'h={p.z:.2f}m  r={math.degrees(roll):+.1f}  p={math.degrees(pitch):+.1f}  y={math.degrees(yaw):+.1f}'
        ma.markers.append(elev)

        # ── 6. Roll/pitch indicator bars on the tilt disc ──
        # Roll bar (along Y axis, tilts with roll)
        roll_bar = Marker()
        roll_bar.header.frame_id = 'map'
        roll_bar.header.stamp = stamp
        roll_bar.ns = 'robot_state'
        roll_bar.id = 5
        roll_bar.type = Marker.CUBE
        roll_bar.action = Marker.ADD
        half = self.wheelbase * 0.6
        roll_bar.pose.position.x = p.x
        roll_bar.pose.position.y = p.y
        roll_bar.pose.position.z = p.z + 0.01
        roll_bar.pose.orientation.w = math.cos(yaw / 2.0)
        roll_bar.pose.orientation.z = math.sin(yaw / 2.0)
        roll_bar.scale.x = 0.008
        roll_bar.scale.y = half * 2.0
        roll_bar.scale.z = 0.003
        roll_bar.color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=0.6)
        ma.markers.append(roll_bar)

        # Pitch bar (along X axis, tilts with pitch)
        pitch_bar = Marker()
        pitch_bar.header.frame_id = 'map'
        pitch_bar.header.stamp = stamp
        pitch_bar.ns = 'robot_state'
        pitch_bar.id = 6
        pitch_bar.type = Marker.CUBE
        pitch_bar.action = Marker.ADD
        pitch_bar.pose.position.x = p.x
        pitch_bar.pose.position.y = p.y
        pitch_bar.pose.position.z = p.z + 0.012
        pitch_bar.pose.orientation.w = math.cos(yaw / 2.0)
        pitch_bar.pose.orientation.z = math.sin(yaw / 2.0)
        pitch_bar.scale.x = half * 2.0
        pitch_bar.scale.y = 0.008
        pitch_bar.scale.z = 0.003
        pitch_bar.color = ColorRGBA(r=0.3, g=1.0, b=0.3, a=0.6)
        ma.markers.append(pitch_bar)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateVizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
