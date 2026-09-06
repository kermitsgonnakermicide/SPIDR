"""3-DOF per-leg inverse / forward kinematics for the spooder hexapod.

Physical-robot defaults come from the user's measured servo-to-servo
dimensions: coxa 41.6 mm, femur 53 mm, tibia 91.6 mm (tibia measured
centre-of-rotation-of-tibia-servo straight down to the foot). These
are shorter than the URDF (0.043 / 0.060 / 0.104), so call
:func:`set_link_lengths` from the boot script if you want to run the
sim OR record a more accurate tibia measurement.

Why this differs from the URDF -- the URDF was authored before the
physical hardware was assembled. The vent in
``spooder_description/urdf/spooder.xacro`` is too generous on the
femur/tibia: at any foot target the sim-side ``ik_coxa`` falls back to
(0, 0, 0). The real robot's tibia servo CoR-to-foot is 91.6 mm, not
104 mm, and the femur is 53 mm, not 60 mm. If you boot this code in
simulation without overriding the link lengths, every step the gait
will instantly revert to its "centre" pose and the robot won't
actually walk.

.. note::
   With these defaults, femur + tibia = 0.145 m. That caps the
   reachable vertical drop at 0.145 m and leaves only enough horizontal
   room to land the foot ~10 cm out from the coxa. Tune ``default_foot_z``
   in ``gait_params.yaml`` to *-0.10* (not -0.12) for the physical rig.

Frame conventions
-----------------
Coxa frame (per leg, before yaw)::

    +X  = radial out from the body along the leg
    +Y  = lateral, perpendicular to the radial direction
    +Z  = up (the coxa yaw axis)

Joint order/coordinate::

    q1 = coxa yaw  (rotation about the leg's local +Z)
    q2 = femur pitch (lifts the femur in the (X, Z) plane of the
                       coxa frame; q2 = 0 means femur lying along +X)
    q3 = tibia pitch relative to femur (q3 = 0 means tibia collinear
                                       with femur → leg fully straight)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

# Defaults match the physical servos the user measured. Call
# set_link_lengths() to override (e.g. when running the sim).
COXA_LEN: float = 0.0416
FEMUR_LEN: float = 0.053
TIBIA_LEN: float = 0.0920

# URDF joint limits (rad) -- wider than physical so we don't slice
# against the URDF here. The per-joint soft limits from the
# calibration YAML are enforced in the gait controller instead.
COXA_LIMITS: Tuple[float, float] = (-0.7, 0.7)
FEMUR_LIMITS: Tuple[float, float] = (-1.5, 1.5)
TIBIA_LIMITS: Tuple[float, float] = (-2.5, 0.5)

# Per-leg mount, copied verbatim from spooder.xacro <xacro:leg ...> instantiations.
# Indexing: 0=rf, 1=rm, 2=rr, 3=lf, 4=lm, 5=lr.
LEG_ORIGINS = [
    (0.0835, -0.063, 0.0),   # 0: rf
    (0.0,    -0.063, 0.0),   # 1: rm
    (-0.0835, -0.063, 0.0),  # 2: rr
    (0.0835,  0.063, 0.0),   # 3: lf
    (0.0,     0.063, 0.0),   # 4: lm
    (-0.0835, 0.063, 0.0),   # 5: lr
]
LEG_ANGLES = [
    -0.7853,   # 0: rf  (-45 deg)
    -1.5708,   # 1: rm  (-90 deg)
    -2.3561,   # 2: rr  (-135 deg)
     0.7853,   # 3: lf  (+45 deg)
     1.5708,   # 4: lm  (+90 deg)
     2.3561,   # 5: lr  (+135 deg)
]
LEG_NAMES = ("RF", "RM", "RR", "LF", "LM", "LR")


def set_link_lengths(*, coxa: Optional[float] = None,
                     femur: Optional[float] = None,
                     tibia: Optional[float] = None) -> None:
    """Override the link lengths at runtime. Used by the calibration
    loader so measurements recorded from the physical servos can be
    fed back into IK without editing source."""
    global COXA_LEN, FEMUR_LEN, TIBIA_LEN
    if coxa is not None:
        COXA_LEN = float(coxa)
    if femur is not None:
        FEMUR_LEN = float(femur)
    if tibia is not None:
        TIBIA_LEN = float(tibia)


@dataclass
class JointAngles:
    q_coxa: float
    q_femur: float
    q_tibia: float

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.q_coxa, self.q_femur, self.q_tibia)


# ---------------------------------------------------------------------------
# Per-leg IK
# ---------------------------------------------------------------------------

def _law_of_cosines_ik(d_horiz: float, d_z: float) -> Optional[Tuple[float, float]]:
    """Core 2-DOF solver given target height *z* and post-coxa horizontal *r*.

    Returns ``(q_femur, q_tibia)`` or ``None`` if the target is out of
    the femur+tibia reach.
    """
    L = math.hypot(d_horiz, d_z)
    if L > FEMUR_LEN + TIBIA_LEN or L < abs(FEMUR_LEN - TIBIA_LEN):
        return None
    cos_alpha = (FEMUR_LEN * FEMUR_LEN + L * L - TIBIA_LEN * TIBIA_LEN) / (2 * FEMUR_LEN * L)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.acos(cos_alpha)
    beta = math.atan2(d_z, d_horiz)
    q_femur = beta + alpha
    cos_gamma = (FEMUR_LEN * FEMUR_LEN + TIBIA_LEN * TIBIA_LEN - L * L) / (2 * FEMUR_LEN * TIBIA_LEN)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))
    gamma = math.acos(cos_gamma)
    q_tibia = gamma - math.pi
    return q_femur, q_tibia


def ik_coxa(x: float, y: float, z: float) -> Optional[JointAngles]:
    """Foot target in coxa frame -> joint angles."""
    q_coxa = math.atan2(y, x)
    if not (COXA_LIMITS[0] <= q_coxa <= COXA_LIMITS[1]):
        return None
    horiz = math.hypot(x, y) - COXA_LEN
    inner = _law_of_cosines_ik(horiz, z)
    if inner is None:
        return None
    q_femur, q_tibia = inner
    return JointAngles(q_coxa, q_femur, q_tibia)


def is_reachable(x: float, y: float, z: float) -> bool:
    horiz = math.hypot(x, y) - COXA_LEN
    L = math.hypot(horiz, z)
    return abs(FEMUR_LEN - TIBIA_LEN) <= L <= FEMUR_LEN + TIBIA_LEN


def clamp_to_reach(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """If the target is unreachable, scale (x, y) back so it just fits."""
    if is_reachable(x, y, z):
        return x, y, z
    r_xy = math.hypot(x, y)
    if r_xy < 1e-6:
        return x, y, z
    h_max = math.sqrt(max((FEMUR_LEN + TIBIA_LEN) ** 2 - z * z, 0.0))
    scale = (h_max + COXA_LEN) / r_xy * 0.99
    return x * scale, y * scale, z


# ---------------------------------------------------------------------------
# Same math packaged for body-frame input (used by the IMU swing overrides).
# ---------------------------------------------------------------------------

def ik_body(leg: int, target_xyz: Tuple[float, float, float]) -> Optional[JointAngles]:
    """Foot position in body frame for leg index *leg*."""
    if not 0 <= leg < len(LEG_ORIGINS):
        return None
    ox, oy, oz = LEG_ORIGINS[leg]
    a = LEG_ANGLES[leg]
    tx = target_xyz[0] - ox
    ty = target_xyz[1] - oy
    tz = target_xyz[2] - oz
    cos_a, sin_a = math.cos(-a), math.sin(-a)
    cx = tx * cos_a - ty * sin_a
    cy = tx * sin_a + ty * cos_a
    cz = tz
    return ik_coxa(cx, cy, cz)


def foot_pos_body(leg: int, q_coxa: float, q_femur: float, q_tibia: float) -> Tuple[float, float, float]:
    """Forward kinematics: joint angles -> foot position in the body frame.

    Mirror of :func:`ik_body`. Math: in the coxa frame (origin at the
    coxa joint, ``+X`` radial, ``+Z`` up) the leg is a 2-link arm in
    the ``(X, Z)`` plane; q_coxa then sweeps the radial direction around
    the coxa's ``+Z``. Adding :data:`LEG_ORIGINS[leg]` and rotating by
    :data:`LEG_ANGLES[leg]` lands the result in body frame.

    q_tibia is the *interior knee angle delta*: ``q_tibia = 0`` means
    the tibia is collinear with the femur (knee fully straight, leg at
    max reach); ``q_tibia = -pi`` means the knee is fully folded.
    """
    if not 0 <= leg < len(LEG_ORIGINS):
        raise IndexError(f"leg {leg} out of range")
    ox, oy, oz = LEG_ORIGINS[leg]
    a = LEG_ANGLES[leg]

    # Radial extent of femur + tibia in the coxa frame's (radial, z) plane.
    cs_f, sn_f = math.cos(q_femur), math.sin(q_femur)
    cs_t, sn_t = math.cos(q_femur + q_tibia), math.sin(q_femur + q_tibia)
    radial = COXA_LEN + FEMUR_LEN * cs_f + TIBIA_LEN * cs_t
    z = FEMUR_LEN * sn_f + TIBIA_LEN * sn_t

    # Sweep radial by q_coxa in the coxa frame's (X, Y) plane.
    cs_c, sn_c = math.cos(q_coxa), math.sin(q_coxa)
    coxa_x = radial * cs_c
    coxa_y = radial * sn_c
    coxa_z = z

    # Coxa frame --> body frame: rotate by LEG_ANGLE about Z, then offset.
    cs_l, sn_l = math.cos(a), math.sin(a)
    bx = coxa_x * cs_l - coxa_y * sn_l + ox
    by = coxa_x * sn_l + coxa_y * cs_l + oy
    bz = coxa_z + oz
    return bx, by, bz
