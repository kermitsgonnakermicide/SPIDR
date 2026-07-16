import numpy as np
import math


def _safe_div(a, b, fallback=0.0):
    return a / b if abs(b) > 1e-10 else fallback


# ---------------------------------------------------------------------------
# Family 1: 3D Geometric Hazards (on terrain surface)
# ---------------------------------------------------------------------------

def geometric_costs(voxel_map, candidates, default_z=-0.12):
    if len(candidates) == 0:
        return np.zeros((0, 5), dtype=np.float32)

    results = []
    for c in candidates:
        cx, cy, cz = float(c[0]), float(c[1]), float(c[2])
        h = voxel_map.height_at(cx, cy)

        if h is None:
            results.append([1.0, 0.0, 1.0, 1.0, 0.0])
            continue

        slope = voxel_map.slope_deg(cx, cy)
        _, roughness = voxel_map.surface_normal_and_roughness(cx, cy)
        step_h = voxel_map.step_height_at(cx, cy)
        edge_d = voxel_map.edge_distance(cx, cy)

        height_diff = abs(h - cz)

        results.append([
            _safe_div(min(slope, 60.0), 60.0),
            min(roughness, 1.0),
            _safe_div(min(step_h, 0.15), 0.15),
            max(0.0, 1.0 - _safe_div(edge_d, 0.15)),
            _safe_div(min(height_diff, 0.08), 0.08),
        ])
    return np.array(results, dtype=np.float32)


# ---------------------------------------------------------------------------
# Family 2: Stability Cost (support polygon, CoM margin)
# ---------------------------------------------------------------------------

def stability_costs(candidates, leg_origins_body, active_leg_mask):
    if len(candidates) == 0 or len(leg_origins_body) == 0:
        return np.zeros((len(candidates), 2), dtype=np.float32)

    stance_legs = [i for i, active in enumerate(active_leg_mask) if not active]
    if len(stance_legs) < 3:
        return np.zeros((len(candidates), 2), dtype=np.float32)

    stance_origins = np.array([leg_origins_body[i][:2] for i in stance_legs], dtype=np.float32)

    hull_center = stance_origins.mean(axis=0)

    results = []
    for c in candidates:
        cx, cy = float(c[0]), float(c[1])
        candidate_2d = np.array([cx, cy])

        dist_to_hull = np.linalg.norm(candidate_2d - hull_center)

        radii = []
        for o in stance_origins:
            radii.append(np.linalg.norm(candidate_2d - o))
        mean_radius = float(np.mean(radii)) if radii else 0.0
        radius_variance = float(np.std(radii)) if radii else 0.0

        results.append([
            min(dist_to_hull / 0.15, 1.0),
            min(radius_variance / 0.05, 1.0),
        ])
    return np.array(results, dtype=np.float32)


# ---------------------------------------------------------------------------
# Family 3: Kinematic Reachability (joint limit proximity)
# ---------------------------------------------------------------------------

def kinematic_costs(candidates, leg_origins_body, leg_params):
    results = []
    for idx, c in enumerate(candidates):
        origin = leg_origins_body[idx] if idx < len(leg_origins_body) else np.zeros(3)
        dx = float(c[0]) - float(origin[0])
        dy = float(c[1]) - float(origin[1])
        dz = float(c[2]) - float(origin[2])
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        results.append([_safe_div(abs(dist - 0.14), 0.06)])
    return np.array(results, dtype=np.float32)


# ---------------------------------------------------------------------------
# Family 4: Energy Cost (torque estimate, smoothness)
# ---------------------------------------------------------------------------

def energy_costs(candidates, leg_origins_body, prev_footholds, leg_idx):
    results = []
    for idx, c in enumerate(candidates):
        origin = leg_origins_body[idx] if idx < len(leg_origins_body) else np.zeros(3)
        dx = float(c[0]) - float(origin[0])
        dy = float(c[1]) - float(origin[1])
        dz = float(c[2]) - float(origin[2])
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        torque_estimate = abs(dz) * 2.0 + abs(dy) * 0.5

        smooth = 0.0
        if prev_footholds is not None and leg_idx < len(prev_footholds):
            px, py, pz = prev_footholds[leg_idx]
            smooth = math.sqrt(
                (float(c[0]) - px) ** 2 +
                (float(c[1]) - py) ** 2 +
                (float(c[2]) - pz) ** 2
            )

        results.append([
            _safe_div(min(torque_estimate, 2.0), 2.0),
            _safe_div(min(smooth, 0.05), 0.05),
        ])
    return np.array(results, dtype=np.float32)


# ---------------------------------------------------------------------------
# Combined cost with tunable weights
# ---------------------------------------------------------------------------

DEFAULT_WEIGHTS = {
    'geometric': np.array([2.0, 1.0, 1.5, 1.5, 1.0], dtype=np.float32),
    'stability': np.array([1.5, 1.0], dtype=np.float32),
    'kinematic': np.array([1.5], dtype=np.float32),
    'energy': np.array([0.5, 0.5], dtype=np.float32),
}


def compute_total_cost(
    voxel_map, candidates, leg_origins_body, active_leg_mask,
    prev_footholds=None, leg_idx=0, weights=None, default_z=-0.12,
):
    n = len(candidates)
    if n == 0:
        return np.array([], dtype=np.float32)

    if weights is None:
        w = DEFAULT_WEIGHTS
    else:
        w = weights

    g = geometric_costs(voxel_map, candidates, default_z)
    s = stability_costs(candidates, leg_origins_body, active_leg_mask)
    k = kinematic_costs(candidates, leg_origins_body, None)
    e = energy_costs(candidates, leg_origins_body, prev_footholds, leg_idx)

    cost = np.zeros(n, dtype=np.float32)
    if g.shape[0] == n:
        cost += g @ w['geometric']
    if s.shape[0] == n:
        cost += s @ w['stability']
    if k.shape[0] == n:
        cost += k @ w['kinematic']
    if e.shape[0] == n:
        cost += e @ w['energy']

    return cost
