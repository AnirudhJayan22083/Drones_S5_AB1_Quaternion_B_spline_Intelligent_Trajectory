"""
Obstacle environment and analytical ray-casting sensing for Stage 6 (ArduPilot).

Provides obstacle geometry matching Stage 4 (MuJoCo) and Stage 5 (PyBullet),
together with an exact ray-surface distance query function that matches the
Stage 3 sensing contract:

    obstacle_distance_fn(position: np.ndarray, direction: np.ndarray) -> float
"""

import numpy as np
from stage3_sensing_environment import config as s3_config

# Obstacles matching models/quadrotor.xml and stage5_pybullet_sim
OBSTACLES = [
    {
        "name": "obstacle_1",
        "shape": "box",
        "half_extents": np.array([0.15, 0.15, 0.5], dtype=float),
        "position": np.array([1.5, 0.5, 1.0], dtype=float),
        "color": [0.85, 0.25, 0.2, 0.6],
    },
    {
        "name": "obstacle_2",
        "shape": "cylinder",
        "radius": 0.2,
        "height": 1.5,
        "position": np.array([-1.2, -1.0, 0.75], dtype=float),
        "color": [0.85, 0.25, 0.2, 0.6],
    },
]


def ray_box_intersect(ray_origin: np.ndarray, ray_dir: np.ndarray,
                      box_center: np.ndarray, half_extents: np.ndarray) -> float:
    """Exact ray-box slab intersection test."""
    min_b = box_center - half_extents
    max_b = box_center + half_extents
    t_min = -np.inf
    t_max = np.inf

    for i in range(3):
        if abs(ray_dir[i]) < 1e-9:
            if ray_origin[i] < min_b[i] or ray_origin[i] > max_b[i]:
                return np.inf
        else:
            inv_d = 1.0 / ray_dir[i]
            t1 = (min_b[i] - ray_origin[i]) * inv_d
            t2 = (max_b[i] - ray_origin[i]) * inv_d
            if t1 > t2:
                t1, t2 = t2, t1
            t_min = max(t_min, t1)
            t_max = min(t_max, t2)
            if t_min > t_max:
                return np.inf

    if t_max < 0.0:
        return np.inf
    return max(0.0, float(t_min))


def ray_cylinder_intersect(ray_origin: np.ndarray, ray_dir: np.ndarray,
                           cyl_center: np.ndarray, radius: float, height: float) -> float:
    """Exact ray-cylinder (Z-aligned) intersection test with flat end-caps."""
    ro = ray_origin - cyl_center
    rd = ray_dir
    z_min = -height / 2.0
    z_max = height / 2.0

    a = rd[0] ** 2 + rd[1] ** 2
    b = 2.0 * (ro[0] * rd[0] + ro[1] * rd[1])
    c = ro[0] ** 2 + ro[1] ** 2 - radius ** 2

    hits = []
    if a > 1e-9:
        disc = b ** 2 - 4.0 * a * c
        if disc >= 0.0:
            sq = np.sqrt(disc)
            for t in ((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)):
                if t >= 0.0:
                    z = ro[2] + t * rd[2]
                    if z_min <= z <= z_max:
                        hits.append(float(t))

    # Flat caps at z_min and z_max
    if abs(rd[2]) > 1e-9:
        for z_cap in (z_min, z_max):
            t_cap = (z_cap - ro[2]) / rd[2]
            if t_cap >= 0.0:
                x = ro[0] + t_cap * rd[0]
                y = ro[1] + t_cap * rd[1]
                if x ** 2 + y ** 2 <= radius ** 2:
                    hits.append(float(t_cap))

    return min(hits) if hits else np.inf


def distance_to_obstacle_surface(point: np.ndarray, obs: dict) -> float:
    """Exact Euclidean distance from point to the surface of an obstacle."""
    p = np.asarray(point, dtype=float)
    if obs["shape"] == "box":
        c = obs["position"]
        h = obs["half_extents"]
        d = np.maximum(np.abs(p - c) - h, 0.0)
        return float(np.linalg.norm(d))
    elif obs["shape"] == "cylinder":
        c = obs["position"]
        r = obs["radius"]
        h = obs["height"]
        d_xy = max(0.0, float(np.linalg.norm(p[:2] - c[:2]) - r))
        d_z = max(0.0, float(abs(p[2] - c[2]) - h / 2.0))
        return float(np.hypot(d_xy, d_z))
    return np.inf


def create_obstacle_distance_fn(
    obstacles=None,
    min_range: float = s3_config.SENSOR_MIN_RANGE,
    max_range: float = s3_config.SENSOR_MAX_RANGE,
    noise_sigma: float = s3_config.SENSOR_NOISE_SIGMA,
    enable_noise: bool = False,
    seed: int = None,
):
    """
    Creates an obstacle distance query function conforming to Stage 3 signature:
        obstacle_distance_fn(position: np.ndarray, direction: np.ndarray) -> float
    """
    obs_list = obstacles if obstacles is not None else OBSTACLES
    rng = np.random.default_rng(seed)

    def obstacle_distance_fn(position: np.ndarray, direction: np.ndarray) -> float:
        d = np.asarray(direction, dtype=float)
        norm = np.linalg.norm(d)
        if norm < 1e-8:
            return max_range
        d = d / norm
        p = np.asarray(position, dtype=float)

        min_hit = max_range
        for obs in obs_list:
            if obs["shape"] == "box":
                hit = ray_box_intersect(p, d, obs["position"], obs["half_extents"])
            else:
                hit = ray_cylinder_intersect(p, d, obs["position"], obs["radius"], obs["height"])

            if hit < min_hit:
                min_hit = hit

        clipped = float(np.clip(min_hit, min_range, max_range))
        if enable_noise and min_hit < max_range:
            clipped = float(np.clip(clipped + rng.normal(0.0, noise_sigma), min_range, max_range))
        return clipped

    return obstacle_distance_fn
