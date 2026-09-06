#!/usr/bin/env python3
"""
Sim-aware static_transform_publisher replacement.

Rosbags / simulated time: tf2_ros's CLI static_transform_publisher publishes
its latched /tf_static message ONCE in the constructor with
``node.get_clock().now()``. When ``use_sim_time`` is True and the Gazebo
/clock bridge has not yet fired, rclcpp returns ROS time zero. The result is
a static transform latched at stamp=0,0, which then shows up as a "future"
message relative to sim-time and floods tf2 buffers with "Detected jump back
in time" warnings once the clock catches up.

This script waits until the simulated clock is non-zero (or times out)
before publishing the static transform, then sends one latched message
stamped at that sim time and spins.

Usage:
    sim_static_tf.py --frame-id map --child-frame-id odom --x 0 --y 0 --z 0

Notes:
    * This script honours --use-sim-time/--no-use-sim-time; default is True
      via the ROS parameter override below.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def _parse_header(value: str) -> Tuple[float, float, float]:
    return tuple(float(v) for v in value.split(','))  # type: ignore[return-value]


def _quaternion_from_euler(roll: float, pitch: float, yaw: float):
    # Standard XYZ->quat using tf2's conventions would be nicer, but plain
    # numpy/transforms is not a hard dep here. Replicate the same yaw-pitch-roll
    # used by tf2_ros static_transform_publisher.
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def _wait_for_sim_clock(node: Node, timeout: float) -> bool:
    """Block until at least one /clock message has been received."""
    received = {'flag': False}

    qos = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

    def _on_clock(_msg: Clock) -> None:
        received['flag'] = True

    node.create_subscription(Clock, '/clock', _on_clock, qos)
    deadline = time.time() + timeout
    while time.time() < deadline and not received['flag']:
        rclpy.spin_once(node, timeout_sec=0.1)
    return received['flag']


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--frame-id', required=True)
    parser.add_argument('--child-frame-id', required=True)
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=0.0)
    parser.add_argument('--roll', type=float, default=0.0)
    parser.add_argument('--pitch', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument(
        '--wait-timeout', type=float, default=15.0,
        help='Seconds to wait for the sim clock before publishing.',
    )
    parser.add_argument(
        '--use-sim-time', dest='use_sim_time',
        action='store_true', default=True,
    )
    parser.add_argument(
        '--no-use-sim-time', dest='use_sim_time',
        action='store_false',
    )

    # Allow ROS-style flags to override --use-sim-time/--no-use-sim-time.
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    args = parser.parse_args(argv)

    rclpy.init()
    try:
        node = rclpy.create_node(
            'sim_static_tf',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL,
                                         args.use_sim_time),
            ],
        )
    except Exception as e:
        print(f'Failed to create node: {e}', file=sys.stderr)
        return 2

    if args.use_sim_time:
        node.get_logger().info(
            f'Waiting up to {args.wait_timeout:.1f}s for /clock to come up...')
        ok = _wait_for_sim_clock(node, args.wait_timeout)
        if not ok:
            node.get_logger().warn(
                'Timed out waiting for /clock; publishing at stamp=now anyway.')

    qx, qy, qz, qw = _quaternion_from_euler(args.roll, args.pitch, args.yaw)
    broadcaster = StaticTransformBroadcaster(node)

    from geometry_msgs.msg import TransformStamped  # noqa: imported lazily
    msg = TransformStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = args.frame_id
    msg.child_frame_id = args.child_frame_id
    msg.transform.translation.x = float(args.x)
    msg.transform.translation.y = float(args.y)
    msg.transform.translation.z = float(args.z)
    msg.transform.rotation.x = qx
    msg.transform.rotation.y = qy
    msg.transform.rotation.z = qz
    msg.transform.rotation.w = qw
    broadcaster.sendTransform(msg)

    node.get_logger().info(
        f'Published latched {args.frame_id} -> {args.child_frame_id} at '
        f'stamp {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
