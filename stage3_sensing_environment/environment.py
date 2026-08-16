"""
Environment management and obstacle query utilities for MuJoCo world models.

Provides tools to discover, inspect, and dynamically reposition static or
moving obstacle bodies in the MuJoCo simulation environment.
"""

from typing import Dict, List, Optional, Tuple, Union

import mujoco
import numpy as np

from . import config


class EnvironmentManager:
    """
    Manages obstacle bodies and environment queries in a MuJoCo model/data.
    """

    def __init__(self, model: mujoco.MjModel, data: Optional[mujoco.MjData] = None):
        """
        model: Loaded mujoco.MjModel.
        data: Optional mujoco.MjData. If provided, position queries use dynamic
              data state; otherwise initial model positions are used.
        """
        self.model = model
        self.data = data
        self.drone_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, config.DRONE_BODY_NAME)
        self._obstacle_cache: Dict[str, Dict] = {}
        self.refresh_obstacles()

    def refresh_obstacles(self) -> List[Dict]:
        """Scan the model worldbody for obstacle bodies and cache metadata."""
        self._obstacle_cache.clear()
        obstacles = []

        for body_id in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if not name:
                continue

            # Identify obstacle bodies by prefix or non-drone/world bodies
            if name.startswith(config.OBSTACLE_PREFIX):
                obs_info = self._inspect_body(body_id, name)
                self._obstacle_cache[name] = obs_info
                obstacles.append(obs_info)

        return obstacles

    def _inspect_body(self, body_id: int, name: str) -> Dict:
        """Extract position, geom details, and bounding radius for a body."""
        # Initial position from model.body_pos
        pos = np.array(self.model.body_pos[body_id], dtype=float)

        # Collect attached geoms to compute bounding sphere
        geom_ids = [
            g_id for g_id in range(self.model.ngeom)
            if self.model.geom_bodyid[g_id] == body_id
        ]

        max_extent = 0.0
        geom_info = []
        for g_id in geom_ids:
            g_type = self.model.geom_type[g_id]
            g_size = self.model.geom_size[g_id]
            g_pos = self.model.geom_pos[g_id]
            g_dist = np.linalg.norm(g_pos)

            # Compute bounding radius for geom type
            if g_type == mujoco.mjtGeom.mjGEOM_BOX:
                radius = np.linalg.norm(g_size[:3])
            elif g_type in (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
                radius = np.sqrt(g_size[0] ** 2 + g_size[1] ** 2) if g_type == mujoco.mjtGeom.mjGEOM_CYLINDER else np.linalg.norm(g_size[:2])
            else:
                radius = np.max(g_size)

            extent = g_dist + radius
            if extent > max_extent:
                max_extent = extent

            geom_info.append({
                "geom_id": g_id,
                "type": g_type,
                "size": g_size,
                "pos": g_pos,
                "bounding_radius": radius,
            })

        return {
            "name": name,
            "body_id": body_id,
            "pos": pos,
            "bounding_radius": max_extent if max_extent > 0 else 0.5,
            "geoms": geom_info,
        }

    def get_obstacles(self) -> List[Dict]:
        """Return cached obstacle metadata, updating positions from data if available."""
        obstacles = []
        for name, info in self._obstacle_cache.items():
            obs = info.copy()
            if self.data is not None:
                obs["pos"] = np.array(self.data.xpos[info["body_id"]], dtype=float)
            obstacles.append(obs)
        return obstacles

    def set_obstacle_position(self, name_or_id: Union[str, int], position: np.ndarray):
        """
        Dynamically update an obstacle's position in self.data (or model.body_pos if data is None).
        """
        if isinstance(name_or_id, str):
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name_or_id)
        else:
            body_id = name_or_id

        if body_id < 0 or body_id >= self.model.nbody:
            raise ValueError(f"Invalid body identifier: {name_or_id}")

        pos_arr = np.asarray(position, dtype=float)
        self.model.body_pos[body_id] = pos_arr

        if self.data is not None:
            # If body has a mocap joint or free joint, or update xpos directly before forward
            self.data.xpos[body_id] = pos_arr
            mujoco.mj_forward(self.model, self.data)

        if name_or_id in self._obstacle_cache:
            self._obstacle_cache[name_or_id]["pos"] = pos_arr
