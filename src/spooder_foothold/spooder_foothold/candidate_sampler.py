import numpy as np
import math


def sample_polar_grid(nominal_x, nominal_y, radial_steps=4, angular_steps=8,
                      min_radius=0.0, max_radius=0.06):
    candidates = []
    angles = np.linspace(0, 2 * math.pi, angular_steps, endpoint=False)
    if radial_steps == 1:
        radii = [0.0]
    else:
        radii = np.linspace(min_radius, max_radius, radial_steps)
    for r in radii:
        for theta in angles:
            cx = nominal_x + r * math.cos(theta)
            cy = nominal_y + r * math.sin(theta)
            candidates.append((cx, cy, r, theta))
    if radial_steps > 1 and min_radius == 0.0:
        cands = [(nominal_x, nominal_y, 0.0, 0.0)]
        for r, theta in [(cr, ct) for cr in radii[1:] for ct in angles]:
            cands.append((nominal_x + r * math.cos(theta),
                          nominal_y + r * math.sin(theta), r, theta))
        candidates = cands
    return candidates


def sample_spherical_grid(nominal_x, nominal_y, nominal_z,
                          radial_steps=3, angular_steps=8, max_radius=0.06,
                          z_range=0.03):
    candidates = []
    radii = np.linspace(0, max_radius, radial_steps)
    angles = np.linspace(0, 2 * math.pi, angular_steps, endpoint=False)
    z_offsets = np.linspace(-z_range, z_range, 3)
    for r in radii:
        for theta in angles:
            for dz in z_offsets:
                cx = nominal_x + r * math.cos(theta)
                cy = nominal_y + r * math.sin(theta)
                cz = nominal_z + dz
                candidates.append((cx, cy, cz))
    return np.array(candidates, dtype=np.float32)


def filter_by_reachability(candidates, leg_origin, max_reach=0.18, min_reach=0.04):
    filtered = []
    for c in candidates:
        dx = c[0] - leg_origin[0]
        dy = c[1] - leg_origin[1]
        dz = c[2] - leg_origin[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if min_reach <= dist <= max_reach:
            filtered.append(c)
    return np.array(filtered, dtype=np.float32) if filtered else candidates
