"""OAK-D Lite IMU republisher.

The OAK-D Lite on the physical robot sits upside down and what its
firmware considers "front" is actually the back of the robot. Both
of these happen at the *mount*, not inside the camera, so the
depthai_ros_driver-issued topic ``/oak_d/imu`` already carries badly-
rotated linear_acceleration, angular_velocity and orientation
vectors relative to the rest of the robot.

This node:

  1. Subscribes ``/oak_d/imu``.
  2. Applies a parameterised rotation (``imu_correction_rpy_deg``) to
     the IMU data so it lands in the robot frame.
  3. Republishes as ``/imu/data`` with the IMU frame defined in the
     URDF (``oak_d_imu_link``).

The rotation is applied as a left-multiplication by the rotation
matrix derived from the RPY parameter. Gravity is assumed positive
Z when level, which matches the conventions used in
``spooder_description/urdf/imu.xacro``.

Why this lives in spooder_robot and not spooder_perception
---------------------------------------------------------
The IMU is a *body* sensor. As soon as the OAK-D driver emits
``/oak_d/imu``, the data needs to be in robot frame before any
consumer on the robot (gait controller, EKF, body-levelling) sees
it. spooder_perception is the sim pipeline's perception world; the
physical-robot frame is more naturally handled alongside the rest of
the robot controls.
"""
from __future__ import annotations

import math
import threading

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger


_ROLL_PITCH_YAW_EULER = "rpy"   # we apply R then P then Y in body frame


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """3-3 rotation matrix from RPY (XYZ extrinsic) angles in radians."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ], dtype=float)


def _quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=float)


def _matrix_to_quat(m: np.ndarray):
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    else:
        # Pick the largest diagonal to keep numerical stability.
        if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = 2.0 * math.sqrt(max(1.0 + m[0, 0] - m[1, 1] - m[2, 2], 0.0))
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = 2.0 * math.sqrt(max(1.0 + m[1, 1] - m[0, 0] - m[2, 2], 0.0))
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(max(1.0 + m[2, 2] - m[0, 0] - m[1, 1], 0.0))
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
    return x, y, z, w


def _apply_rotation(msg: Imu, R: np.ndarray, frame_id: str) -> Imu:
    out = Imu()
    out.header = msg.header
    out.header.frame_id = frame_id

    def rotate_vec(v):
        a = np.array([v.x, v.y, v.z])
        b = R @ a
        out_v = type(v)()
        out_v.x = float(b[0])
        out_v.y = float(b[1])
        out_v.z = float(b[2])
        return out_v

    out.orientation_covariance = msg.orientation_covariance
    out.angular_velocity_covariance = msg.angular_velocity_covariance
    out.linear_acceleration_covariance = msg.linear_acceleration_covariance

    if msg.orientation_covariance[0] >= 0.0:
        # Driver gives an absolute orientation quaternion. Bake the mount
        # correction into it: R_total = R_corr @ R_measured, then turn back
        # into a quaternion with the same conversion utilities above.
        R_meas = _quat_to_matrix(msg.orientation)
        R_total = R @ R_meas
        x, y, z, w = _matrix_to_quat(R_total)
    else:
        # Covariance [-1] on first element means orientation is missing;
        # leave it as the zero quat so downstream nodes don't accidentally
        # trust a fabricated orientation.
        x = y = z = 0.0
        w = 1.0

    out.orientation.x = x
    out.orientation.y = y
    out.orientation.z = z
    out.orientation.w = w

    out.angular_velocity = rotate_vec(msg.angular_velocity)
    out.linear_acceleration = rotate_vec(msg.linear_acceleration)
    return out


class ImuRepublisher(Node):
    def __init__(self):
        super().__init__("spooder_imu_node")
        self._lock = threading.Lock()

        self.declare_parameter("input_topic", "/oak_d/imu")
        self.declare_parameter("output_topic", "/imu/data")
        self.declare_parameter("output_frame_id", "imu_link")
        # 180-deg pitch + 180-deg yaw is the working configuration that
        # matches "OAK-D upside down + its front looks to the back".
        # Override at boot if you change the mount.
        self.declare_parameter(
            "imu_correction_rpy_deg",
            [0.0, 0.0, 180.0],
        )
        self.declare_parameter(
            "imu_upside_down",
            True,
        )

        # Subscribe at higher QoS so we don't drop IMU samples in a busy
        # pipeline -- controller is BEST_EFFORT + KEEP_LAST(10).
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self.create_subscription(
            Imu, self.get_parameter("input_topic").value,
            self._cb, sub_qos,
        )

        # Publish at slightly more conservative QoS to make inter-host
        # discovery safer. EKF / gait will adjust their own profile.
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._pub = self.create_publisher(
            Imu, self.get_parameter("output_topic").value, pub_qos,
        )
        self._svc = self.create_service(Trigger, "~/reset_correction", self._reset)

        self._R = self._build_R(self.get_parameter("imu_correction_rpy_deg").value,
                                bool(self.get_parameter("imu_upside_down").value))
        self.get_logger().info(
            f"IMU republisher up: {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value} ({self.get_parameter('output_frame_id').value})",
        )

    def _build_R(self, rpy_deg, upside_down):
        rpy = [math.radians(rpy_deg[0]), math.radians(rpy_deg[1]), math.radians(rpy_deg[2])]
        if upside_down:
            # Rotating 180 about the camera's +X axis flips Z and Y, taking
            # "upside down" into account. Combined with the user's yaw
            # trim, this covers both stated issues.
            rpy[0] += math.pi
        with self._lock:
            self._R = _rpy_to_matrix(*rpy)
        return self._R

    def _cb(self, msg: Imu):
        with self._lock:
            R = self._R
        out = _apply_rotation(msg, R, self.get_parameter("output_frame_id").value)
        self._pub.publish(out)

    def _reset(self, _req, response):
        self._build_R(
            self.get_parameter("imu_correction_rpy_deg").value,
            bool(self.get_parameter("imu_upside_down").value),
        )
        response.success = True
        response.message = "IMU correction rebuilt from parameters."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
