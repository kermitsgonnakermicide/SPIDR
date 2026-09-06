"""Tripod gait controller for the physical robot.

This replaces the sine-wave joint angle generator from the Tk control
app, which dragged feet at the swing extremes and veered over time
because it had no body-velocity compensation in the foot frame.

Compared with the Tk ``GaitEngine.leg_target()``:

  * Foot *target positions* are planned in body frame every cycle,
    then converted to per-leg joint angles via IK. So the foot path is
    smooth arc in 3D space, not a single-joint sine.
  * Stance-leg foot velocity in body frame equals ``-cmd_vel`` (foot
    stays planted on moving body). Yaw rotation is folded in so a
    turning body actually pivots the stance legs around it instead of
    sweeping them straight back.
  * Per-leg swing targets honour optional ``/leg_N/foothold_target``
    overrides published by the laptop-side foothold planner when it's
    running; otherwise the planner falls back to a symmetric open-loop
    step the size of the body's predicted step over the swing window.
  * IMU-driven femur trim: each leg's femur gets a small additive bias
    proportional to body roll + pitch, so the gait stays roughly level
    even when stepping on a slope or transiently off-balance.
  * Joint values leaving the IK solver are first clamped to the
    per-joint soft limits from the calibration YAML, then shifted by
    the per-joint offset and (maybe) inverted before being sent to the
    ST3215 bus.

The node *does not* own the bus; it publishes a 18-element
``Float64MultiArray`` to ``/spooder_controller/commands``. The
``position_controllers/JointGroupPositionController`` consumed at the
``ros2_control``-level serialises that one Float64MultiArray per
joint into SYNC_WRITE on the ST3215 bus.

.. note::
   This is deliberately a *separate* node from
   ``hexapod_nav/gait_controller_node.py`` because the two have
   different intents -- the sim-side node is hooked up to Nav2 and a
   full foothold planner; this one is the "fail-safe" gait that runs
   on the physical robot without any laptop, accepting only
   ``/cmd_vel`` and per-leg swing overrides. Operators should pick
   one or the other as the canonical producer of
   ``/spooder_controller/commands`` -- never run both at once.
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, UInt8MultiArray
from geometry_msgs.msg import Point, PointStamped, Twist

from . import kinematics as kin


NUM_LEGS = 6
JOINT_ORDER = (
    "rf_coxa_joint", "rf_femur_joint", "rf_tibia_joint",
    "rm_coxa_joint", "rm_femur_joint", "rm_tibia_joint",
    "rr_coxa_joint", "rr_femur_joint", "rr_tibia_joint",
    "lf_coxa_joint", "lf_femur_joint", "lf_tibia_joint",
    "lm_coxa_joint", "lm_femur_joint", "lm_tibia_joint",
    "lr_coxa_joint", "lr_femur_joint", "lr_tibia_joint",
)

# Tripod groupings -- rf+lr+lm vs rm+lf+rr so adjacent legs move with
# the same phase. Mirrors the convention in hexapod_nav/gait_controller_node.py
# so the planner (if it's also running) sees the same leg_phase bits.
TRIPOD_A = (0, 2, 4)   # rf, rr, lm
TRIPOD_B = (1, 3, 5)   # rm, lf, lr

STANCE = 0
SWING = 1


def _bezier_swing(p0: np.ndarray, p3: np.ndarray, height_m: float, t: float) -> np.ndarray:
    """Cubic Bezier swing trajectory in body frame (3-D, m).

    Control points lift ``p0`` and ``p3`` straight up by ``height_m``,
    so the trace leaves and lands vertically -- far less "dragging the
    foot" than a parabola in xy only.
    """
    p1 = p0.copy(); p1[2] += height_m
    p2 = p3.copy(); p2[2] += height_m
    one_minus_t = 1.0 - t
    return ((one_minus_t ** 3) * p0
            + 3 * (one_minus_t ** 2) * t * p1
            + 3 * one_minus_t * (t ** 2) * p2
            + (t ** 3) * p3)


def _rotate_xy(v: np.ndarray, dtheta: float) -> np.ndarray:
    """Rotate a 2-D column (x,y) by dtheta in the body-XY plane."""
    c, s = math.cos(dtheta), math.sin(dtheta)
    x, y = v[0], v[1]
    return np.array([c * x - s * y, s * x + c * y, v[2]])


@dataclass
class FootState:
    """Per-leg body-frame foot position plus swing bookkeeping."""
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    swing_t: float = 1.0             # 0..1, 1 = landing/sticking
    swing_target: np.ndarray = field(default_factory=lambda: np.zeros(3))
    in_swing: bool = False
    phase: int = STANCE


@dataclass
class ImuReading:
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    stamp: float = 0.0


class GaitControllerNode(Node):
    def __init__(self):
        super().__init__("spooder_gait_node")

        # ---------------- parameters ----------------
        self.declare_parameter("calibration_yaml", "")
        self.declare_parameter("calibration_json_path", "")
        # Defaults match the physical robot's measured leg lengths
        # (femur+tibia = 0.145 m). See config/gait_params.yaml for why.
        self.declare_parameter("default_foot_forward", 0.10)
        self.declare_parameter("default_foot_z", -0.10)
        self.declare_parameter("cycle_period_s", 1.6)
        self.declare_parameter("swing_duration_s", 0.5)
        self.declare_parameter("step_height_m", 0.05)
        self.declare_parameter("max_step_length_m", 0.10)
        self.declare_parameter("max_yaw_rate_rad_s", 0.6)
        self.declare_parameter("cmd_vel_timeout_s", 0.6)
        # Femur-trim gains expressed as fractions of the body tilt:
        # the resulting joint correction is ``gain * tilt_rad`` so the
        # values are dimensionless ratios (rad of trim / rad of tilt).
        # The Tk control app's "3 steps / degree" comes out to about
        # 0.27 in this unit, so 0.1 is a conservative starting point.
        self.declare_parameter("imu_roll_gain", 0.1)
        self.declare_parameter("imu_pitch_gain", 0.1)
        self.declare_parameter("imu_max_trim_rad", 0.20)
        self.declare_parameter("imu_flip_roll", False)
        self.declare_parameter("imu_flip_pitch", False)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("use_laptop_overrides", True)
        self.declare_parameter("default_pose_on_stop", True)

        # ---------------- state ----------------
        self._calib = self._load_calibration()
        self._lock = threading.Lock()
        self._cmd_vel = np.zeros(3)
        self._cmd_vel_stamp = self._clock_now()
        self._imu = ImuReading()
        # Per-leg body-frame foot positions. Initialise at the resting
        # pose the calibration says is "home" so first cycle doesn't
        # jump.
        self._feet = [FootState() for _ in range(NUM_LEGS)]
        for f in self._feet:
            f.pos[:] = np.array([
                self.get_parameter("default_foot_forward").value,
                0.0,
                self.get_parameter("default_foot_z").value,
            ])
        self._active_tripod = 0   # 0 or 1
        self._pending_tripod = 0  # index of the next set to start swing

        # Per-leg override from laptop (None = use default step).
        self._overrides: list[Optional[PointStamped]] = [None] * NUM_LEGS
        # Bitmask of which override to "consume" on next reset.
        self._override_stamp = [-1.0] * NUM_LEGS

        # ---------------- IO ----------------
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        for i in range(NUM_LEGS):
            self.create_subscription(
                PointStamped,
                f"/leg_{i}/foothold_target",
                lambda msg, leg=i: self._override_cb(leg, msg),
                sub_qos,
            )
        self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_cb,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=10),
        )
        self.create_subscription(
            Imu, "/imu/data", self._imu_cb,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=10),
        )
        # Joint state is optional -- we use it only for diagnostics of
        # which leg phases are landing where the planner expects.
        self.create_subscription(
            JointState, "/joint_states", self._joints_cb,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=2),
        )
        self._cmd_pub = self.create_publisher(
            Float64MultiArray, "/spooder_controller/commands", 10,
        )
        self._phase_pub = self.create_publisher(
            UInt8MultiArray, "/spooder_robot/leg_phase", 10,
        )
        self._cmd_zero_pub = self.create_publisher(
            Float64MultiArray, "/spooder_controller/commands", 10,
        )

        self._timer = self.create_timer(
            1.0 / self.get_parameter("control_rate_hz").value,
            self._tick,
        )
        self.get_logger().info("spooder gait controller up")

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------
    def _load_calibration(self):
        from . import calibration
        json_path = self.get_parameter("calibration_json_path").value or None
        yaml_path = self.get_parameter("calibration_yaml").value or None
        try:
            cal = calibration.load(json_path=json_path, yaml_path=yaml_path)
        except Exception as exc:
            self.get_logger().warn(
                f"calibration load failed ({exc}); using URDF defaults"
            )
            cal = calibration.URDF_DEFAULTS
        if cal.warnings:
            for w in cal.warnings:
                self.get_logger().warn(f"calibration: {w}")
        return cal

    # ------------------------------------------------------------------
    # Subscription handlers
    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self._cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
            self._cmd_vel_stamp = self._clock_now()

    def _imu_cb(self, msg: Imu):
        # Roll/pitch from orientation quaternion -- much more stable than
        # linear_acceleration while the body is in motion. We assume the
        # IMU republisher has already aligned the IMU's frame with the
        # robot's body frame, so this is just a quaternion-to-euler on
        # the published orientation. Yaw is ignored (magnetometer-less
        # IMU drifts; offset-correction comes from elsewhere).
        q = msg.orientation
        n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if n < 1e-6 or msg.orientation_covariance[0] < 0.0:
            return  # invalid / missing orientation; keep last reading
        x, y, z, w = q.x / n, q.y / n, q.z / n, q.w / n
        # ZYX intrinsic roll/pitch (yaw discarded)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        if self.get_parameter("imu_flip_roll").value:
            roll_deg = -roll_deg
        if self.get_parameter("imu_flip_pitch").value:
            pitch_deg = -pitch_deg
        self._imu = ImuReading(
            roll_rad=math.radians(roll_deg),
            pitch_rad=math.radians(pitch_deg),
            stamp=self._clock_now(),
        )

    def _joints_cb(self, _msg):
        # Reserved for future "are we where the planner thinks we are?"
        # diagnostics; nothing to do here in the open-loop gait.
        return None

    def _override_cb(self, leg: int, msg: PointStamped):
        if not self.get_parameter("use_laptop_overrides").value:
            return
        self._overrides[leg] = msg
        self._override_stamp[leg] = self._clock_now()

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def _tick(self):
        dt = 1.0 / max(1.0, self.get_parameter("control_rate_hz").value)
        now = self._clock_now()
        with self._lock:
            vx_b, vy_b, yaw_rate = self._cmd_vel
            age = now - self._cmd_vel_stamp
            if age > self.get_parameter("cmd_vel_timeout_s").value:
                vx_b = vy_b = yaw_rate = 0.0
            # Apply parameter caps so a misbehaving planner can't slam
            # the body past its physical limit.
            yaw_rate = max(
                -self.get_parameter("max_yaw_rate_rad_s").value,
                min(self.get_parameter("max_yaw_rate_rad_s").value, yaw_rate),
            )
            imu_trim = self._imu_trim()

        # Movement policy: if the body isn't commanded to move and the
        # user wants home-on-stop, freeze in the standing pose.
        if (vx_b == 0.0 and vy_b == 0.0 and yaw_rate == 0.0
                and self.get_parameter("default_pose_on_stop").value):
            self._park_and_publish()
            return

        # Advance all legs.
        cycle_period = self.get_parameter("cycle_period_s").value
        swing_dur = self.get_parameter("swing_duration_s").value
        step_height = self.get_parameter("step_height_m").value
        max_step = self.get_parameter("max_step_length_m").value
        step_vx = max(-max_step, min(max_step, vx_b * swing_dur))
        step_vy = max(-max_step, min(max_step, vy_b * swing_dur))

        # 1) Stance legs push the body, so the feet track backwards in body frame.
        for leg in range(NUM_LEGS):
            foot = self._feet[leg]
            if not foot.in_swing:
                foot.pos[0] -= vx_b * dt
                foot.pos[1] -= vy_b * dt
                dtheta = yaw_rate * dt
                foot.pos[:] = _rotate_xy(foot.pos, dtheta)

        # 2) Tripod-state machine. When the *currently swinging* tripod
        #    finishes, commit the swing targets and start the next.
        swinging_legs = TRIPOD_A if self._active_tripod == 0 else TRIPOD_B
        current_tripod_done = all(
            self._feet[leg].swing_t >= 1.0 for leg in swinging_legs
        )
        if current_tripod_done:
            self._active_tripod = 1 - self._active_tripod
            swinging_legs = TRIPOD_A if self._active_tripod == 0 else TRIPOD_B
            # Start the new tripod each leg from its current pose.
            for leg in swinging_legs:
                foot = self._feet[leg]
                target = self._choose_swing_target(leg, step_vx, step_vy, yaw_rate, swing_dur)
                foot.swing_target = target
                foot.swing_t = 0.0
                foot.in_swing = True

        # 3) Advance active swinging legs along the Bezier arc.
        for leg in swinging_legs:
            foot = self._feet[leg]
            foot.swing_t = min(foot.swing_t + dt / swing_dur, 1.0)
            self._feet[leg].pos = _bezier_swing(
                foot.pos, foot.swing_target, step_height, foot.swing_t
            )
            if foot.swing_t >= 1.0:
                foot.in_swing = False
                # Foot lands exactly at the swing target.
                self._feet[leg].pos[:] = self._feet[leg].swing_target

        # 4) Solve IK + write commands.
        joint_targets = self._solve_all(imu_trim)
        self._publish_commands(joint_targets)

    def _park_and_publish(self):
        # Smoothly relax to the standing pose over a few cycles to
        # avoid a small "settle twitch" when /cmd_vel goes idle.
        stand = np.array([
            self.get_parameter("default_foot_forward").value,
            0.0,
            self.get_parameter("default_foot_z").value,
        ])
        # Update forward in body frame on every tick so when the gate
        # wakes back up the legs are at their default x/z already.
        for foot in self._feet:
            if not np.allclose(foot.pos, stand, atol=1e-3):
                foot.pos[0] += (stand[0] - foot.pos[0]) * 0.15
                foot.pos[2] += (stand[2] - foot.pos[2]) * 0.15
                foot.pos[1] += (0.0 - foot.pos[1]) * 0.15
            foot.in_swing = False
            foot.swing_t = 1.0
        joint_targets = self._solve_all(np.zeros(3))
        self._publish_commands(joint_targets)
        # Publish phase too so diagnostics see "all stance".
        self._publish_phase()

    # ------------------------------------------------------------------
    # Swing target selection: default step vs. laptop override.
    # ------------------------------------------------------------------
    def _choose_swing_target(self, leg: int, step_max_x: float,
                             step_max_y: float, yaw_rate: float,
                             swing_dur: float) -> np.ndarray:
        ov = self._overrides[leg]
        if ov is not None:
            self._overrides[leg] = None
            # Override is given in *body* frame per the convention in
            # hexapod_nav/gait_controller_node. If a future planner
            # publishes world-frame points instead, we'll need a body
            # pose here -- left as a TODO.
            target = np.array([ov.point.x, ov.point.y, ov.point.z])
            return self._clamp_step(leg, target, max_step_along=0.20)

        # Default open-loop step: lift+place forward by what the body
        # will move through over this swing window, leaving the foot
        # one body-velocity*swing-duration ahead in body frame.
        # That's the predicted foot landing position in body frame at
        # the end of the swing.
        cur = self._feet[leg].pos.copy()
        dtheta = yaw_rate * swing_dur
        # Foot should NOT carry the body's velocity over the swing --
        # so the *target* in body frame is the predicted foot landing
        # in body frame after yaw rotation, but with no translation.
        # Empirically we use 0.85x swing_dur compensation so the foot
        # doesn't overshoot when phase transitions jitter slightly.
        target_x = cur[0] + 0.85 * step_max_x
        target_y = cur[1] + 0.85 * step_max_y
        target_z = self.get_parameter("default_foot_z").value
        target = _rotate_xy(np.array([target_x, target_y, target_z]), dtheta)
        return target

    def _clamp_step(self, leg: int, target: np.ndarray,
                    max_step_along: float) -> np.ndarray:
        """Keep the swing target within a sensible envelope per leg so
        discretionary planner overrides can't push the IK solver out
        of its reachable workspace.

        Two-layer clamp:
          1. Per-leg step-length cap (XY distance from current pose)
             keeps fidelity with the planner's intent -- "shorten, don't
             redirect".
          2. After the step clamp, run the absolute target through
             :func:`kin.clamp_to_reach` so the IK solver never sees an
             unreachable point (which would otherwise silently revert
             the leg to its centre pose -- the bug that turned the
             sim's "walk" into "stand still").
        """
        cur = self._feet[leg].pos.copy()
        delta = target - cur
        D = math.hypot(delta[0], delta[1])
        if D > max_step_along:
            scale = max_step_along / D
            delta *= scale
            target = cur + delta
        cx, cy, cz = kin.clamp_to_reach(target[0], target[1], target[2])
        return np.array([cx, cy, cz])

    # ------------------------------------------------------------------
    # IMU trim
    # ------------------------------------------------------------------
    def _imu_trim(self) -> np.ndarray:
        """6-vector of per-leg femur trim, applied *after* IK so it
        doesn't break foot-target convergence."""
        roll = self._imu.roll_rad
        pitch = self._imu.pitch_rad
        g_roll = float(self.get_parameter("imu_roll_gain").value)
        g_pitch = float(self.get_parameter("imu_pitch_gain").value)
        max_trim = self.get_parameter("imu_max_trim_rad").value
        # Per-leg lever-arm signs. Mirror hexapod_nav/gait_controller
        # defaults so the planner-side behavior matches.
        LEG_ROLL_SIGN = (1, 1, 1, -1, -1, -1)
        LEG_PITCH_SIGN = (1, 0, -1, 1, 0, -1)
        out = np.zeros(NUM_LEGS)
        for i in range(NUM_LEGS):
            t = (LEG_ROLL_SIGN[i] * g_roll * roll
                 + LEG_PITCH_SIGN[i] * g_pitch * pitch)
            out[i] = max(-max_trim, min(max_trim, t))
        return out

    # ------------------------------------------------------------------
    # IK + commands
    # ------------------------------------------------------------------
    def _solve_all(self, femur_trim: np.ndarray) -> list:
        joint_targets = []
        for leg in range(NUM_LEGS):
            # Clamp the current leg's foot position to a reachable
            # workspace *before* handing it off to IK. Without this,
            # an out-of-reach foot (e.g. landing on a slope that
            # exceeds the leg's reach) makes ik_body return None and we
            # would have to fall back to a bogus centre pose -- which
            # is exactly the bug that turned the simulator's "walk"
            # into "stand still".
            foot = self._feet[leg].pos
            cx, cy, cz = kin.clamp_to_reach(float(foot[0]),
                                            float(foot[1]),
                                            float(foot[2]))
            ang = kin.ik_body(leg, (cx, cy, cz))
            if ang is None:
                # clamp_to_reach should have guaranteed reachability;
                # if we still get None (e.g. coxa angle exceeded
                # COXA_LIMITS), use the standing-pose default angles
                # rather than silently sending the centre pose.
                ox, oy, oz = kin.LEG_ORIGINS[leg]
                stand = np.array([
                    self.get_parameter("default_foot_forward").value,
                    0.0,
                    self.get_parameter("default_foot_z").value,
                ])
                fb = kin.ik_body(leg, tuple(stand.tolist()))
                ang = fb if fb is not None else kin.JointAngles(0.0, 0.0, 0.0)
            q1, q2, q3 = ang.q_coxa, ang.q_femur, ang.q_tibia
            # IMU body-leveling: shift femur of each leg a little.
            q2 += float(femur_trim[leg])

            for j_idx, q in enumerate((q1, q2, q3)):
                joint_name = JOINT_ORDER[leg * 3 + j_idx]
                q = self._calib.clamp(joint_name, q)
                bus_q = self._calib.apply(joint_name, q)
                joint_targets.append(bus_q)
        return joint_targets

    def _publish_commands(self, joint_targets: list):
        msg = Float64MultiArray()
        msg.data = list(float(x) for x in joint_targets)
        self._cmd_pub.publish(msg)
        self._publish_phase()

    def _publish_phase(self):
        msg = UInt8MultiArray()
        msg.data = [SWING if f.in_swing else STANCE for f in self._feet]
        self._phase_pub.publish(msg)

    # ------------------------------------------------------------------
    def _clock_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = GaitControllerNode()
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
