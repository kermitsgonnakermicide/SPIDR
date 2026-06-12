import numpy as np
import math


def extract_all(voxel_map, candidates):
    if len(candidates) == 0:
        return np.zeros((0, 6), dtype=np.float32)

    results = []
    for c in candidates:
        cx, cy, cz = c[0], c[1], c[2]
        h = voxel_map.height_at(cx, cy)
        if h is None:
            results.append([1.0, 0.0, 1.0, 1.0, 0.0, 0.0])
            continue

        slope = voxel_map.slope_deg(cx, cy)
        normal, roughness = voxel_map.surface_normal_and_roughness(cx, cy)
        step_h = voxel_map.step_height_at(cx, cy)
        edge_d = voxel_map.edge_distance(cx, cy)

        height_diff = abs(h - cz)
        results.append([
            slope / 45.0,
            roughness,
            step_h / 0.1,
            max(0.0, 1.0 - edge_d / 0.15),
            height_diff / 0.05,
            float(h),
        ])
    return np.array(results, dtype=np.float32)


def compute_cost(features, weights=None):
    if weights is None:
        weights = np.array([1.5, 1.0, 1.2, 1.0, 1.0], dtype=np.float32)
    if len(features) == 0:
        return np.array([], dtype=np.float32)
    weighted = features[:, :5] * weights
    return weighted.sum(axis=1)
