"""
kinematics.py

3-DOF inverse kinematics for hexapod legs.
All dimensions match the Diddler URDF exactly.

Two IK interfaces:
  - ik_coxa(x, y, z): coxa-frame IK (matches ik_solver.py, used by gait controller)
  - ik_leg(target_body, leg_index): body-frame IK (used by foothold planner)

Leg frame: X = forward along coxa, Z = up
Joint order: coxa (yaw), femur (pitch), tibia (pitch)
"""

import math
import numpy as np

# Diddler link lengths (from URDF)
COXA_LEN = 0.043
FEMUR_LEN = 0.060
TIBIA_LEN = 0.104

# Joint limits (radians) — match URDF
COXA_LIMITS = (-0.7, 0.7)
FEMUR_LIMITS = (-1.5, 1.5)
TIBIA_LIMITS = (-2.5, 0.5)

# Leg mount positions relative to base_link center (x, y, z)
# From URDF xacro leg instantiations
LEG_ORIGINS = [
    np.array([ 0.0835, -0.063, 0.0]),   # 0: rf (front-right)
    np.array([ 0.0,    -0.063, 0.0]),   # 1: rm (mid-right)
    np.array([-0.0835, -0.063, 0.0]),   # 2: rr (rear-right)
    np.array([ 0.0835,  0.063, 0.0]),   # 3: lf (front-left)
    np.array([ 0.0,     0.063, 0.0]),   # 4: lm (mid-left)
    np.array([-0.0835,  0.063, 0.0]),   # 5: lr (rear-left)
]

# Leg mount yaw angles (from URDF yaw parameter)
LEG_ANGLES = [
    -0.7853,   # rf: -45 deg
    -1.5708,   # rm: -90 deg
    -2.3561,   # rr: -135 deg
     0.7853,   # lf: +45 deg
     1.5708,   # lm: +90 deg
     2.3561,   # lr: +135 deg
]


def _law_of_cosines_ik(d_xy, d_z):
    """
    Core 2-DOF IK (femur + tibia) given horizontal distance and height.
    Matches ik_solver.py convention exactly.

    Returns (q_femur, q_tibia) or None if unreachable.
    """
    L = math.sqrt(d_xy**2 + d_z**2)
    if L > FEMUR_LEN + TIBIA_LEN or L < abs(FEMUR_LEN - TIBIA_LEN):
        return None

    cos_alpha = (FEMUR_LEN**2 + L**2 - TIBIA_LEN**2) / (2 * FEMUR_LEN * L)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.acos(cos_alpha)

    beta = math.atan2(d_z, d_xy)
    q_femur = beta + alpha

    cos_gamma = (FEMUR_LEN**2 + TIBIA_LEN**2 - L**2) / (2 * FEMUR_LEN * TIBIA_LEN)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))
    gamma = math.acos(cos_gamma)
    q_tibia = gamma - math.pi

    return (q_femur, q_tibia)


def ik_coxa(x, y, z):
    """
    IK for coxa-frame coordinates. Matches ik_solver.py exactly.
    Used by gait controller for standing/walking.

    Args:
        x, y, z: foot position in the coxa frame (m)
    Returns:
        (q_coxa, q_femur, q_tibia) or (0, 0, 0) if unreachable
    """
    q_coxa = math.atan2(y, x)
    if not (COXA_LIMITS[0] <= q_coxa <= COXA_LIMITS[1]):
        return (0.0, 0.0, 0.0)

    horizontal_dist = math.sqrt(x**2 + y**2) - COXA_LEN
    result = _law_of_cosines_ik(horizontal_dist, z)
    if result is None:
        return (0.0, 0.0, 0.0)

    q_femur, q_tibia = result
    return (q_coxa, q_femur, q_tibia)


def ik_leg(target_body, leg_index):
    """
    IK for body-frame coordinates. Used by foothold planner for reachability.

    Args:
        target_body: (x, y, z) foot position in body frame
        leg_index: 0-5 leg index
    Returns:
        (q_coxa, q_femur, q_tibia) or None if unreachable
    """
    origin = LEG_ORIGINS[leg_index]
    angle = LEG_ANGLES[leg_index]

    # Transform to leg frame
    t = target_body - origin
    cos_a = math.cos(-angle)
    sin_a = math.sin(-angle)
    tx = t[0] * cos_a - t[1] * sin_a
    ty = t[0] * sin_a + t[1] * cos_a
    tz = t[2]

    # Coxa yaw
    q_coxa = math.atan2(ty, tx)
    if not (COXA_LIMITS[0] <= q_coxa <= COXA_LIMITS[1]):
        return None

    # Femur + tibia
    horizontal_dist = math.sqrt(tx**2 + ty**2) - COXA_LEN
    result = _law_of_cosines_ik(horizontal_dist, tz)
    if result is None:
        return None

    q_femur, q_tibia = result
    return (q_coxa, q_femur, q_tibia)


def world_to_coxa(world_pt, leg_index, body_pose):
    """
    Convert a world-frame point to coxa-frame coordinates for IK.

    Args:
        world_pt: (x, y, z) in world frame
        leg_index: 0-5
        body_pose: (x, y, z, yaw) of body in world frame
    Returns:
        (cx, cy, cz) in coxa frame, suitable for ik_coxa()
    """
    bx, by, bz, byaw = body_pose

    # World → body frame
    dx = world_pt[0] - bx
    dy = world_pt[1] - by
    cos_y = math.cos(-byaw)
    sin_y = math.sin(-byaw)
    body_x = dx * cos_y - dy * sin_y
    body_y = dx * sin_y + dy * cos_y
    body_z = world_pt[2]

    # Body → leg frame (same transform as ik_leg)
    origin = LEG_ORIGINS[leg_index]
    angle = LEG_ANGLES[leg_index]
    t = np.array([body_x - origin[0], body_y - origin[1], body_z - origin[2]])
    cos_a = math.cos(-angle)
    sin_a = math.sin(-angle)
    tx = t[0] * cos_a - t[1] * sin_a
    ty = t[0] * sin_a + t[1] * cos_a
    tz = t[2]

    return (tx, ty, tz)


def coxa_to_world(coxa_pt, leg_index, body_pose):
    """Inverse of world_to_coxa: coxa-frame point → world frame."""
    bx, by, bz, byaw = body_pose
    origin = LEG_ORIGINS[leg_index]
    angle = LEG_ANGLES[leg_index]

    # Coxa → body (rotate by +leg angle, then add origin)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    cx, cy, cz = float(coxa_pt[0]), float(coxa_pt[1]), float(coxa_pt[2])
    body_x = cx * cos_a - cy * sin_a + origin[0]
    body_y = cx * sin_a + cy * cos_a + origin[1]
    body_z = cz + origin[2]

    # Body → world
    cos_y = math.cos(byaw)
    sin_y = math.sin(byaw)
    wx = body_x * cos_y - body_y * sin_y + bx
    wy = body_x * sin_y + body_y * cos_y + by
    wz = body_z + bz
    return np.array([wx, wy, wz])


def get_reachable_zone(leg_index, body_pose, grid_positions):
    """
    Given a list of (x, y, z) world positions, return boolean mask of reachable cells.

    body_pose: (x, y, z, yaw) of robot body in world frame
    grid_positions: (N, 3) array of candidate foothold positions
    """
    reachable = np.zeros(len(grid_positions), dtype=bool)

    for i, wp in enumerate(grid_positions):
        coxa_pt = world_to_coxa(wp, leg_index, body_pose)
        result = _law_of_cosines_ik(
            math.sqrt(coxa_pt[0]**2 + coxa_pt[1]**2) - COXA_LEN,
            coxa_pt[2])
        # Also check coxa yaw
        q_coxa = math.atan2(coxa_pt[1], coxa_pt[0])
        reachable[i] = (result is not None and
                        COXA_LIMITS[0] <= q_coxa <= COXA_LIMITS[1])

    return reachable
