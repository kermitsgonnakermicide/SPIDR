import numpy as np
import math


class VoxelMap:
    def __init__(self, resolution=0.05, neighbor_radius=0.15):
        self.resolution = resolution
        self.neighbor_radius = neighbor_radius
        self.points = np.empty((0, 3), dtype=np.float32)
        self._height_grid = {}
        self._voxel_set = set()
        self._is_updated = False

    def update(self, points):
        if len(points) == 0:
            return
        self.points = points
        self._height_grid = {}
        self._voxel_set = set()
        inv_res = 1.0 / self.resolution
        for p in points:
            ix = int(math.floor(p[0] * inv_res))
            iy = int(math.floor(p[1] * inv_res))
            iz = int(math.floor(p[2] * inv_res))
            self._voxel_set.add((ix, iy, iz))
            key = (ix, iy)
            if key not in self._height_grid or p[2] > self._height_grid[key]:
                self._height_grid[key] = p[2]
        self._is_updated = True

    @property
    def is_updated(self):
        return self._is_updated

    def height_at(self, x, y):
        inv_res = 1.0 / self.resolution
        ix = int(math.floor(x * inv_res))
        iy = int(math.floor(y * inv_res))
        key = (ix, iy)
        if key in self._height_grid:
            return self._height_grid[key]
        candidates = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                k = (ix + dx, iy + dy)
                if k in self._height_grid:
                    candidates.append(self._height_grid[k])
        if candidates:
            return sum(candidates) / len(candidates)
        return None

    def surface_normal_and_roughness(self, x, y):
        inv_res = 1.0 / self.resolution
        cx = int(math.floor(x * inv_res))
        cy = int(math.floor(y * inv_res))
        radius_cells = max(1, int(self.neighbor_radius * inv_res))
        neighbors = []
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                k = (cx + dx, cy + dy)
                if k in self._height_grid:
                    wx = (k[0] + 0.5) * self.resolution
                    wy = (k[1] + 0.5) * self.resolution
                    wz = self._height_grid[k]
                    neighbors.append([wx, wy, wz])
        if len(neighbors) < 4:
            return np.array([0.0, 0.0, 1.0]), 0.0

        pts = np.array(neighbors, dtype=np.float32)
        mean = pts.mean(axis=0)
        centered = pts - mean
        cov = centered.T @ centered
        eigvals, eigvecs = np.linalg.eigh(cov)
        normal = eigvecs[:, 0]
        if normal[2] < 0:
            normal = -normal
        normal /= np.linalg.norm(normal)
        roughness = eigvals[0] / (eigvals.sum() + 1e-10)
        return normal, roughness

    def slope_deg(self, x, y):
        normal, _ = self.surface_normal_and_roughness(x, y)
        cos_angle = normal[2]
        cos_angle = max(-1.0, min(1.0, cos_angle))
        return math.degrees(math.acos(cos_angle))

    def step_height_at(self, x, y, search_radius=None):
        if search_radius is None:
            search_radius = self.resolution * 2
        inv_res = 1.0 / self.resolution
        cx = int(math.floor(x * inv_res))
        cy = int(math.floor(y * inv_res))
        center_h = self._height_grid.get((cx, cy))
        if center_h is None:
            return 0.0
        radius_cells = max(1, int(search_radius * inv_res))
        max_step = 0.0
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if dx == 0 and dy == 0:
                    continue
                k = (cx + dx, cy + dy)
                if k in self._height_grid:
                    step = abs(self._height_grid[k] - center_h)
                    if step > max_step:
                        max_step = step
        return max_step

    def edge_distance(self, x, y, search_radius=None):
        if search_radius is None:
            search_radius = self.resolution * 3
        inv_res = 1.0 / self.resolution
        cx = int(math.floor(x * inv_res))
        cy = int(math.floor(y * inv_res))
        has_center = (cx, cy) in self._height_grid
        if not has_center:
            return 0.0
        radius_cells = max(1, int(search_radius * inv_res))
        min_dist = search_radius
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if dx == 0 and dy == 0:
                    continue
                k = (cx + dx, cy + dy)
                if k not in self._height_grid:
                    dist = math.sqrt((dx * self.resolution) ** 2 + (dy * self.resolution) ** 2)
                    if dist < min_dist:
                        min_dist = dist
        return min_dist

    def is_surface_point(self, x, y, z, tolerance=None):
        if tolerance is None:
            tolerance = self.resolution * 0.6
        inv_res = 1.0 / self.resolution
        ix = int(math.floor(x * inv_res))
        iy = int(math.floor(y * inv_res))
        iz = int(math.floor(z * inv_res))
        return (ix, iy, iz) in self._voxel_set

    def project_to_surface(self, x, y):
        h = self.height_at(x, y)
        if h is not None:
            return h
        return None
