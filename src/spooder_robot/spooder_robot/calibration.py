"""Per-joint calibration loader.

Three input formats are supported, in order of precedence:

1. JSON next to the cached YAML (``calibration_json_path``), parsed into
   the same per-joint dict structure used by the Tk control app. The
   Tk app saves a file like::

       {"RF_coxa": {"servo_id": 0, "invert": false,
                    "min": 1792, "max": 2304, "center": 2049}, ...}

   ``min``, ``max`` and ``center`` are 12-bit-step counts (0..4095).
   For ST3215 / Feetech servos, an encoder value of 2048 sits at
   "joint angle = 0 rad" (mid-rotation), so any deviation of *center*
   from 2048 becomes the per-joint ``offset_rad``:

       offset_rad = -(center - 2048) * 2*pi / 4096

   The Jacobian that turns this into the *bus* value is
   ``rad_to_position`` inside ``spooder_hardware``.

2. YAML ``robot_calibration.yaml`` delivered with this package, where
   the user has already converted their at-rest measurements into
   radian offsets.

3. Hard-coded defaults embedded in this file (``URDF_DEFAULTS``) if
   neither a JSON file nor a YAML is found, so the gait never crashes
   on first boot with no input.

The loader returns a :class:`RobotCalibration` with two maps:

  * ``joints`` : name -> :class:`JointCalib`
  * ``servo_ids`` : id -> joint name (reverse lookup for diagnostics)

Both are read-only after construction; the gait controller treats the
returned object as the immutable source of truth until the user runs
``reload_calibration`` service.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional


# 12-bit-step <-> radian conversions. Centered on 2048 because that's the
# mid-rotation encoder value the Feetech ST3215 line ships at.
_TK_CENTER_STEPS = 2048
_STEPS_PER_REV = 4096


def _steps_to_offset_rad(center_steps: int) -> float:
    """Offset in radians to subtract from a commanded joint angle so
    the user-measured at-rest encoder position maps to "0 rad"."""
    return -(center_steps - _TK_CENTER_STEPS) * (2 * math.pi / _STEPS_PER_REV)


def _steps_to_soft_limit_rad(low_steps: int, high_steps: int) -> float:
    """Symmetric soft limit scale around the centered pose (rad)."""
    span = max(high_steps - _TK_CENTER_STEPS, _TK_CENTER_STEPS - low_steps)
    # Convert to "radians from the centered pose" -- tighter of the two
    # sides would be more honest, but the user expects the +/- number on
    # the Frontend so we use the half-span.
    return span * (2 * math.pi / _STEPS_PER_REV) * 0.95


# Joint order matches the URDF; the loader uses this list to validate.
EXPECTED_JOINTS = (
    "rf_coxa_joint", "rf_femur_joint", "rf_tibia_joint",
    "rm_coxa_joint", "rm_femur_joint", "rm_tibia_joint",
    "rr_coxa_joint", "rr_femur_joint", "rr_tibia_joint",
    "lf_coxa_joint", "lf_femur_joint", "lf_tibia_joint",
    "lm_coxa_joint", "lm_femur_joint", "lm_tibia_joint",
    "lr_coxa_joint", "lr_femur_joint", "lr_tibia_joint",
)


@dataclass
class JointCalib:
    name: str
    servo_id: int
    offset_rad: float = 0.0
    soft_limit_rad: float = math.pi      # permissive default
    invert: bool = False


@dataclass
class RobotCalibration:
    joints: Dict[str, JointCalib] = field(default_factory=dict)
    servo_to_joint: Dict[int, str] = field(default_factory=dict)
    source: str = "defaults"
    warnings: list = field(default_factory=list)

    def apply(self, joint_name: str, joint_angle_rad: float) -> float:
        """Convert an IK-result joint angle to a bus-level joint angle.

        The hardware interface speaks radians-with-zero-at-servo-center,
        so we subtract the per-joint offset (which encodes the physically
        measured at-rest encoder vs IK home). The invert flag flips the
        *joint angle* before the offset is applied, because the mirror
        happened at the servo, not in IK:

            bus = invert ? -(angle - offset) : (angle - offset)

        Equivalently in code::
        """
        j = self.joints[joint_name]
        if j.invert:
            return -(joint_angle_rad - j.offset_rad)
        return joint_angle_rad - j.offset_rad

    def clamp(self, joint_name: str, joint_angle_rad: float) -> float:
        """Soft-limit the *IK-result* angle (the gait should never have
        to think about bus-level angles)."""
        j = self.joints[joint_name]
        lim = abs(j.soft_limit_rad)
        return max(-lim, min(lim, joint_angle_rad))


# ---------------------------------------------------------------------------
# JSON loader (Tk app format)
# ---------------------------------------------------------------------------
# Tk app joint keys are two-letter leg + joint (e.g. "rf_coxa") whereas
# the URDF uses three words with a "_joint" suffix. Map between them.
_TK_JOINT_TO_URDF = {}
for prefix in ("rf", "rm", "rr", "lf", "lm", "lr"):
    for joint in ("coxa", "femur", "tibia"):
        _TK_JOINT_TO_URDF[f"{prefix}_{joint}"] = f"{prefix}_{joint}_joint"


def load_from_json(path: str) -> RobotCalibration:
    if not os.path.exists(path):
        raise FileNotFoundError(f"hexapod_config.json not found at {path}")
    with open(path, "r") as f:
        data = json.load(f)
    out = RobotCalibration(source=f"json:{path}")
    for tk_key, urdf_key in _TK_JOINT_TO_URDF.items():
        cfg = data.get(tk_key)
        if cfg is None:
            out.warnings.append(f"missing {tk_key} in {path}")
            continue
        servo_id = int(cfg.get("servo_id", 0))
        center = int(cfg.get("center", _TK_CENTER_STEPS))
        soft_low = int(cfg.get("min", _TK_CENTER_STEPS - 256))
        soft_high = int(cfg.get("max", _TK_CENTER_STEPS + 256))
        out.joints[urdf_key] = JointCalib(
            name=urdf_key,
            servo_id=servo_id,
            offset_rad=_steps_to_offset_rad(center),
            soft_limit_rad=_steps_to_soft_limit_rad(soft_low, soft_high),
            invert=bool(cfg.get("invert", False)),
        )
        out.servo_to_joint[servo_id] = urdf_key
    return out


# ---------------------------------------------------------------------------
# YAML loader (this package's format)
# ---------------------------------------------------------------------------

def load_from_yaml(path: str) -> RobotCalibration:
    if not os.path.exists(path):
        raise FileNotFoundError(f"calibration YAML not found at {path}")
    # Lazy import -- YAML is only needed when we actually load it.
    import yaml
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    try:
        joints_section = data["spooder_robot"]["ros__parameters"]["joints"]
    except (KeyError, TypeError) as e:
        raise ValueError(f"unexpected calibration YAML structure: {e}") from e
    out = RobotCalibration(source=f"yaml:{path}")
    for name, cfg in joints_section.items():
        if name not in EXPECTED_JOINTS:
            out.warnings.append(f"unknown joint {name} in {path}")
            continue
        out.joints[name] = JointCalib(
            name=name,
            servo_id=int(cfg["servo_id"]),
            offset_rad=float(cfg.get("offset_rad", 0.0)),
            soft_limit_rad=float(cfg.get("soft_limit_rad", math.pi)),
            invert=bool(cfg.get("invert", False)),
        )
        out.servo_to_joint[out.joints[name].servo_id] = name
    return out


# ---------------------------------------------------------------------------
# Hard-coded URDF defaults (last resort).
# ---------------------------------------------------------------------------

URDF_DEFAULTS = RobotCalibration(source="urdf_defaults", warnings=[])
for idx, name in enumerate(EXPECTED_JOINTS):
    # 1-based servo IDs match spooder_hardware/config/hardware.yaml
    servo_id = idx + 1
    side_left = name.startswith(("lf", "lm", "lr"))
    joint_part = name.split("_")[1]
    soft = {
        "coxa": 0.7,
        "femur": 1.5,
        "tibia": 2.5,
    }[joint_part]
    URDF_DEFAULTS.joints[name] = JointCalib(
        name=name,
        servo_id=servo_id,
        offset_rad=0.0,
        soft_limit_rad=soft,
        # Tk-app convention: left-side coxas mounted mirrored.
        invert=side_left and joint_part in ("coxa", "tibia"),
    )
    URDF_DEFAULTS.servo_to_joint[servo_id] = name


# ---------------------------------------------------------------------------
# Top-level resolver: prefer JSON > YAML > defaults.
# ---------------------------------------------------------------------------

def load(json_path: Optional[str] = None,
         yaml_path: Optional[str] = None) -> RobotCalibration:
    """Pick the richest available input.

    Raises :class:`FileNotFoundError` only if *json_path* and *yaml_path*
    are both given and both are missing -- falling back to the URDF
    hard-coded baseline is treated as success so the robot never refuses
    to boot just because calibration files weren't installed yet.
    """
    if json_path:
        try:
            return load_from_json(json_path)
        except FileNotFoundError as e:
            if yaml_path is None:
                # Try the YAML next anyway before giving up.
                pass
            else:
                raise

    if yaml_path:
        try:
            cal = load_from_yaml(yaml_path)
            if cal.joints:
                return cal
        except FileNotFoundError:
            pass

    return URDF_DEFAULTS
