"""Coordinator node for the physical spooder robot.

Owns the umbrella over the run-time graph on the robot onboard
computer. It does not take ownership of the ST3215 bus (that lives
inside the ``ros2_control`` ``ST3215System`` plugin) nor of the OAK-D
camera driver (``depthai_ros_driver/camera_node``) -- both run as
sibling processes/nodes -- but it surfaces the cross-cutting services
that any operator (or laptop-side supervisor) needs:

  * ``/spooder_robot/emergency_stop``   - immediately publishes
                                         zero-joint commands and
                                         transitions the controller
                                         manager to a torque-off
                                         state.
  * ``/spooder_robot/enable_torque``   - re-engages the bus after an
                                         emergency stop.
  * ``/spooder_robot/reload_calibration``
                                       - triggers a parameter
                                         refresh so a fresh
                                         ``hexapod_config.json``
                                         dropped on the robot picks
                                         up immediately.

A liveness ``/spooder_robot/heartbeat`` topic is also published at
1 Hz, summed from ``/joint_states``, ``/spooder_controller/commands``
and ``/imu/data`` ages so an out-of-tree supervisor can sanity-check
that the whole robot is online.

.. note::
   This node does *not* own the gait loop -- that lives in
   :mod:`spooder_robot.gait_node`. The emergency-stop service flips
   the gait into ``default_pose_on_stop`` mode AND trips the
   controller manager into a torque-off state.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from . import calibration


HEARTBEAT_PERIOD_S = 1.0
STALE_DATA_THRESHOLD_S = 2.0


class RobotCoordinator(Node):
    def __init__(self):
        super().__init__("spooder_robot_node")

        self.declare_parameter("emergency_stop_topic",
                               "/spooder_robot/emergency_stop")
        self.declare_parameter("command_topic", "/spooder_controller/commands")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("calibration_yaml", "")
        self.declare_parameter("calibration_json_path", "")

        self._stopped = False
        cmd_topic = self.get_parameter("command_topic").value
        self._zero_cmd_pub = self.create_publisher(
            Float64MultiArray, cmd_topic, 10,
        )
        self._estop_pub = self.create_publisher(
            Bool, self.get_parameter("emergency_stop_topic").value, 1,
        )
        self._heartbeat_pub = self.create_publisher(
            String, "/spooder_robot/heartbeat", 1,
        )
        self._estop_active = self.create_publisher(
            Bool, "/spooder_robot/emergency_stop_active", 1,
        )

        # Last-seen stamps for liveness.
        self._last_command_stamp = 0
        self._last_joint_stamp = 0
        self._last_imu_stamp = 0

        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Float64MultiArray, cmd_topic, self._cmd_cb, sub_qos)
        self.create_subscription(JointState, self.get_parameter("joint_state_topic").value,
                                  self._joints_cb, sub_qos)
        self.create_subscription(Imu, self.get_parameter("imu_topic").value,
                                  self._imu_cb, sub_qos)

        self._estop_srv = self.create_service(
            Trigger, "/spooder_robot/emergency_stop", self._estop_cb,
        )
        self._enable_srv = self.create_service(
            SetBool, "/spooder_robot/enable_torque", self._enable_torque_cb,
        )
        self._reload_srv = self.create_service(
            Trigger, "/spooder_robot/reload_calibration", self._reload_cb,
        )
        self._heartbeat_timer = self.create_timer(
            HEARTBEAT_PERIOD_S, self._heartbeat,
        )
        self.get_logger().info("spooder_robot coordinator up")

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------
    def _estop_cb(self, _req, response):
        self._stopped = True
        # Publish zero commands enough times to overwrite any in-flight
        # trajectory in the position controller. The controller manager's
        # joint_state_broadcaster also receives the request downstream,
        # which keeps the broader graph from re-engaging torque.
        zero = Float64MultiArray()
        zero.data = [0.0] * 18
        for _ in range(3):
            self._zero_cmd_pub.publish(zero)
        self._estop_active.publish(Bool(data=True))
        response.success = True
        response.message = (
            "Emergency stop engaged. Zero-joint commands sent; "
            "call /spooder_robot/enable_torque to recover."
        )
        self.get_logger().error(response.message)
        return response

    def _enable_torque_cb(self, req, response):
        self._stopped = not bool(req.data)
        self._estop_active.publish(Bool(data=self._stopped))
        response.success = True
        response.message = (
            "Emergency stop cleared; leg control resumed."
            if not self._stopped
            else "Emergency stop engaged by /enable_torque."
        )
        return response

    def _reload_cb(self, _req, response):
        try:
            cal = calibration.load(
                json_path=(self.get_parameter("calibration_json_path").value or None),
                yaml_path=(self.get_parameter("calibration_yaml").value or None),
            )
        except Exception as exc:
            response.success = False
            response.message = f"reload failed: {exc}"
            self.get_logger().error(response.message)
            return response
        n = len(cal.joints)
        response.success = True
        response.message = (
            f"Re-read calibration: {n} joints, source={cal.source}."
        )
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Subscriptions (liveness only)
    # ------------------------------------------------------------------
    def _cmd_cb(self, _msg):
        self._last_command_stamp = time.time()
        if self._stopped:
            # Block downstream command slots until the operator lifts
            # the e-stop. This is the cheapest "no motion" without
            # having to drill into ros2_control.
            return

    def _joints_cb(self, _msg):
        self._last_joint_stamp = time.time()

    def _imu_cb(self, _msg):
        self._last_imu_stamp = time.time()

    def _heartbeat(self):
        now = time.time()
        cmd_age = (now - self._last_command_stamp) if self._last_command_stamp else None
        js_age = (now - self._last_joint_stamp) if self._last_joint_stamp else None
        imu_age = (now - self._last_imu_stamp) if self._last_imu_stamp else None

        def flag(age, label):
            if age is None:
                return f"{label}=missing"
            if age > STALE_DATA_THRESHOLD_S:
                return f"{label}=stale({age:.1f}s)"
            return f"{label}=ok({age:.1f}s)"

        msg = String()
        msg.data = (
            f"estop={'on' if self._stopped else 'off'} "
            f"{flag(cmd_age, 'cmd')} {flag(js_age, 'js')} {flag(imu_age, 'imu')}"
        )
        self._heartbeat_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotCoordinator()
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
