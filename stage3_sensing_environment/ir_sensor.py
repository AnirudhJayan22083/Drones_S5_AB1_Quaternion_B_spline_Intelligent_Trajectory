"""
IR sensor stub using MuJoCo ray-casting (mj_ray).

Simulates single-ray and multi-ray IR distance sensors mounted on a quadrotor
or at arbitrary positions, complete with range clipping, self-collision
exclusion (bodyexclude), and zero-mean Gaussian measurement noise.
"""

from typing import Callable, Optional, Sequence, Tuple, Union

import mujoco
import numpy as np

from . import config


class IRSensor:
    """
    IR distance sensor using MuJoCo's native ray-casting engine (`mj_ray`).
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: Optional[mujoco.MjData] = None,
        min_range: float = config.SENSOR_MIN_RANGE,
        max_range: float = config.SENSOR_MAX_RANGE,
        noise_sigma: float = config.SENSOR_NOISE_SIGMA,
        enable_noise: bool = config.ENABLE_NOISE,
        drone_body_name: str = config.DRONE_BODY_NAME,
        obstacle_prefix: Optional[str] = None,
        seed: Optional[int] = None,
    ):
        """
        model: Loaded mujoco.MjModel instance.
        data: Loaded mujoco.MjData instance. Created if None.
        min_range: Minimum sensing range in meters.
        max_range: Maximum sensing range in meters (returned on no-hit).
        noise_sigma: Standard deviation of Gaussian measurement noise (m).
        enable_noise: Whether measurement noise is added.
        drone_body_name: Name of drone body to exclude from ray casting.
        obstacle_prefix: If set, hits on bodies whose name doesn't start
            with this prefix (e.g. the ground plane, which mj_ray's
            bodyexclude can't filter -- that only takes one body) are
            treated as "no hit" instead of a real obstacle. None (default)
            reports every hit, matching raw mj_ray behavior -- used by
            ``create_obstacle_distance_fn`` below, whose entire purpose
            *is* obstacle avoidance, not general ray sensing.
        seed: Optional random seed for reproducible noise generation.
        """
        self.model = model
        self.data = data if data is not None else mujoco.MjData(model)
        self.min_range = min_range
        self.max_range = max_range
        self.noise_sigma = noise_sigma
        self.enable_noise = enable_noise
        self.obstacle_prefix = obstacle_prefix

        self.drone_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, drone_body_name
        )

        self.rng = np.random.default_rng(seed)

    def measure_ray(
        self,
        position: np.ndarray,
        direction: np.ndarray,
        add_noise: Optional[bool] = None,
    ) -> Tuple[float, int]:
        """
        Cast a single ray from ``position`` along unit vector ``direction``.

        Returns:
            (distance: float, geom_id_hit: int)
            If no obstacle is hit within max_range, geom_id_hit is -1.
        """
        pnt = np.asarray(position, dtype=np.float64)
        vec = np.asarray(direction, dtype=np.float64)

        norm = np.linalg.norm(vec)
        if norm < 1e-8:
            return self.max_range, -1
        vec = vec / norm

        geomid = np.zeros(1, dtype=np.int32)
        dist = mujoco.mj_ray(
            self.model,
            self.data,
            pnt,
            vec,
            None,  # geomgroup (None = all groups)
            1,     # flg_static = 1 (include static geoms)
            self.drone_body_id,  # bodyexclude (drone body)
            geomid,
        )

        hit_id = int(geomid[0]) if dist >= 0 else -1

        if hit_id >= 0 and self.obstacle_prefix is not None:
            body_id = self.model.geom_bodyid[hit_id]
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
            if not body_name.startswith(self.obstacle_prefix):
                dist = -1.0
                hit_id = -1

        if dist < 0 or dist > self.max_range:
            raw_dist = self.max_range
        else:
            raw_dist = dist

        clipped_dist = float(np.clip(raw_dist, self.min_range, self.max_range))

        should_add_noise = self.enable_noise if add_noise is None else add_noise
        if should_add_noise and self.noise_sigma > 0 and raw_dist < self.max_range:
            noise = float(self.rng.normal(0.0, self.noise_sigma))
            measured_dist = float(np.clip(clipped_dist + noise, self.min_range, self.max_range))
        else:
            measured_dist = clipped_dist

        return measured_dist, hit_id

    def get_obstacle_distance(
        self, position: np.ndarray, direction: np.ndarray, add_noise: Optional[bool] = None
    ) -> float:
        """
        Convenience wrapper returning distance only, adhering exactly to Stage 2's:
            obstacle_distance_fn(position, direction) -> float
        """
        dist, _ = self.measure_ray(position, direction, add_noise=add_noise)
        return dist

    def measure_rays(
        self,
        position: np.ndarray,
        directions: np.ndarray,
        add_noise: Optional[bool] = None,
    ) -> np.ndarray:
        """
        Cast multiple rays from ``position`` in directions specified by array (N, 3).

        Returns:
            distances: (N,) array of measured distances.
        """
        dirs = np.asarray(directions, dtype=float)
        distances = np.zeros(dirs.shape[0], dtype=float)
        for i in range(dirs.shape[0]):
            distances[i] = self.get_obstacle_distance(position, dirs[i], add_noise=add_noise)
        return distances


# --- Helper functions & Module-level API --------------------------------------

_DEFAULT_SENSOR_INSTANCE: Optional[IRSensor] = None


def get_default_sensor(model: mujoco.MjModel, data: Optional[mujoco.MjData] = None) -> IRSensor:
    """Get or create singleton default IRSensor instance."""
    global _DEFAULT_SENSOR_INSTANCE
    if _DEFAULT_SENSOR_INSTANCE is None or _DEFAULT_SENSOR_INSTANCE.model is not model:
        _DEFAULT_SENSOR_INSTANCE = IRSensor(model, data)
    elif data is not None:
        _DEFAULT_SENSOR_INSTANCE.data = data
    return _DEFAULT_SENSOR_INSTANCE


def create_obstacle_distance_fn(
    model: mujoco.MjModel,
    data: Optional[mujoco.MjData] = None,
    enable_noise: bool = config.ENABLE_NOISE,
    max_range: float = config.SENSOR_MAX_RANGE,
    noise_sigma: float = config.SENSOR_NOISE_SIGMA,
) -> Callable[[np.ndarray, np.ndarray], float]:
    """
    Factory creating a standalone `obstacle_distance_fn(position, direction) -> float`
    bound to the provided MuJoCo model and data.

    Matches Stage 2's expected signature:
        obstacle_distance_fn(position: np.ndarray, direction: np.ndarray) -> float

    Filters hits to bodies named with ``config.OBSTACLE_PREFIX`` (matching
    ``EnvironmentManager``'s own convention for "what counts as an obstacle"
    in environment.py) -- mj_ray's bodyexclude only accepts the drone body,
    so without this a downward-facing probe direction would report the
    ground plane itself as "the nearest obstacle" at a distance equal to the
    drone's altitude, which is exactly the kind of false positive Stage 2's
    planner needs *not* to see.
    """
    sensor = IRSensor(
        model=model,
        data=data,
        max_range=max_range,
        noise_sigma=noise_sigma,
        obstacle_prefix=config.OBSTACLE_PREFIX,
        enable_noise=enable_noise,
    )

    def obstacle_distance_fn(position: np.ndarray, direction: np.ndarray) -> float:
        return sensor.get_obstacle_distance(position, direction)

    return obstacle_distance_fn


def get_obstacle_distance(
    position: np.ndarray,
    direction: np.ndarray,
    model: Optional[mujoco.MjModel] = None,
    data: Optional[mujoco.MjData] = None,
) -> float:
    """
    Module-level function for obstacle distance ray queries.
    If model is provided, uses or creates an IRSensor instance.
    If model is None, falls back to config.SENSOR_MAX_RANGE.
    """
    if model is None:
        return config.SENSOR_MAX_RANGE

    sensor = get_default_sensor(model, data)
    return sensor.get_obstacle_distance(position, direction)
