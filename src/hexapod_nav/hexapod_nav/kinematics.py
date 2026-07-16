"""
kinematics.py

3-DOF inverse kinematics for hexapod legs.
Leg frame: X = forward along coxa, Z = up
Joint order: coxa (yaw), femur (pitch), tibia (pitch)

EDIT THESE VALUES to match your robot:
  coxa_len   = length of coxa link (m)
  femur_len  = length of femur link (m)
  tibia_len  = length of tibia link (m)
"""

import numpy as np


COXA_LEN  = 0.053   # metres — CHANGE THIS
FEMUR_LEN = 0.085   # metres — CHANGE THIS
TIBIA_LEN = 0.110   # metres — CHANGE THIS

# Joint limits (radians) — CHANGE TO MATCH YOUR SERVOS
COXA_LIMITS  = (-np.pi/2, np.pi/2)
FEMUR_LIMITS = (-np.pi/3, np.pi/2)
TIBIA_LIMITS = (-2*np.pi/3, 0.0)

# Leg mount positions relative to body centre (x, y, z) — CHANGE THIS
LEG_ORIGINS = [
    np.array([ 0.10,  0.08, 0.0]),   # Leg 0: front-right
    np.array([ 0.00,  0.10, 0.0]),   # Leg 1: mid-right
    np.array([-0.10,  0.08, 0.0]),   # Leg 2: rear-right
    np.array([-0.10, -0.08, 0.0]),   # Leg 3: rear-left
    np.array([ 0.00, -0.10, 0.0]),   # Leg 4: mid-left
    np.array([ 0.10, -0.08, 0.0]),   # Leg 5: front-left
]

# Leg mount angles (rotation of coxa frame relative to body frame)
LEG_ANGLES = [
    np.radians( 30),   # front-right
    np.radians(  0),   # mid-right
    np.radians(-30),   # rear-right
    np.radians(-150),  # rear-left
    np.radians(180),   # mid-left
    np.radians( 150),  # front-left
]


def ik_leg(target_body: np.ndarray, leg_index: int):
    """
    Compute joint angles for a leg to reach target_body (in body frame).

    Returns:
        (q_coxa, q_femur, q_tibia) in radians, or None if unreachable
    """
    origin = LEG_ORIGINS[leg_index]
    angle  = LEG_ANGLES[leg_index]

    # Transform target to leg frame
    t = target_body - origin
    R = np.array([[np.cos(-angle), -np.sin(-angle), 0],
                  [np.sin(-angle),  np.cos(-angle), 0],
                  [0,               0,              1]])
    t_leg = R @ t

    # Coxa rotation: yaw to point at target in XY
    q_coxa = np.arctan2(t_leg[1], t_leg[0])
    if not (COXA_LIMITS[0] <= q_coxa <= COXA_LIMITS[1]):
        return None

    # Remaining distance after coxa
    d_xy = np.sqrt(t_leg[0]**2 + t_leg[1]**2) - COXA_LEN
    d_z  = t_leg[2]
    dist = np.sqrt(d_xy**2 + d_z**2)

    # Check reachability
    if dist > FEMUR_LEN + TIBIA_LEN or dist < abs(FEMUR_LEN - TIBIA_LEN):
        return None

    # Law of cosines
    cos_tibia = (FEMUR_LEN**2 + TIBIA_LEN**2 - dist**2) / (2 * FEMUR_LEN * TIBIA_LEN)
    cos_tibia = np.clip(cos_tibia, -1, 1)
    q_tibia = np.pi - np.arccos(cos_tibia)  # negative convention

    cos_femur = (FEMUR_LEN**2 + dist**2 - TIBIA_LEN**2) / (2 * FEMUR_LEN * dist)
    cos_femur = np.clip(cos_femur, -1, 1)
    alpha = np.arccos(cos_femur)
    beta  = np.arctan2(-d_z, d_xy)
    q_femur = beta + alpha

    q_tibia = -q_tibia  # match servo convention

    # Check joint limits
    if not (FEMUR_LIMITS[0] <= q_femur <= FEMUR_LIMITS[1]):
        return None
    if not (TIBIA_LIMITS[0] <= q_tibia <= TIBIA_LIMITS[1]):
        return None

    return (q_coxa, q_femur, q_tibia)


def get_reachable_zone(leg_index: int, body_pose: np.ndarray,
                       grid_positions: np.ndarray) -> np.ndarray:
    """
    Given a list of (x, y, z) world positions, return boolean mask of reachable cells.

    body_pose: (x, y, z, yaw) of robot body in world frame
    grid_positions: (N, 3) array of candidate foothold positions
    """
    reachable = np.zeros(len(grid_positions), dtype=bool)
    bx, by, bz, byaw = body_pose

    R_body = np.array([[np.cos(byaw), -np.sin(byaw), 0],
                       [np.sin(byaw),  np.cos(byaw), 0],
                       [0,             0,            1]])

    for i, wp in enumerate(grid_positions):
        # World → body frame
        t = wp - np.array([bx, by, bz])
        t_body = R_body.T @ t

        result = ik_leg(t_body, leg_index)
        reachable[i] = result is not None

    return reachable