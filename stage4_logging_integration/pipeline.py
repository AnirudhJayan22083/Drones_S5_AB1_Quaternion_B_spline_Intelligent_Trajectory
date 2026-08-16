"""
Shared pipeline construction for Stage 4's two entry points
(``live_viewer.py`` and ``run_full_demo.py``), so the wiring between
Stage 1/2/3 is defined exactly once instead of duplicated across both
scripts and drifting apart over time.
"""

import os

import mujoco
import numpy as np

from stage1_dynamics_attitude.attitude_controller import AttitudeController
from stage1_dynamics_attitude.mixer import MotorMixer
from stage1_dynamics_attitude import config as s1_config

from stage2_trajectory_planning import config as s2_config
from stage2_trajectory_planning.adaptive_planner import (
    AdaptiveBSplinePlanner,
    DEFAULT_PROBE_DIRECTIONS,
)
from stage2_trajectory_planning.position_controller import PositionController

from stage3_sensing_environment import create_obstacle_distance_fn

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(REPO_ROOT, "models", "quadrotor.xml")

# Same waypoint path Stage 2's own demo uses, chosen to pass close enough
# to obstacle_1 in models/quadrotor.xml that the adaptive planner has
# something to react to -- see stage2_trajectory_planning/run_trajectory_demo.py.
WAYPOINTS = np.array([
    [0.0, 0.0, 1.0],
    [0.8, 0.3, 1.2],
    [1.5, 0.5, 1.4],
    [2.2, 0.2, 1.2],
    [3.0, 0.0, 1.0],
])


def build_pipeline(model: mujoco.MjModel, data: mujoco.MjData):
    """
    Build the full Stage 1+2+3 pipeline against a live (model, data) pair,
    using Stage 3's REAL ray-cast sensor (not Stage 2's synthetic
    ray-sphere stand-in) -- this is the actual integration point Stage 4
    exists to exercise.

    Returns: (planner, traj, pos_ctrl, attitude_ctrl, mixer, obstacle_distance_fn)
    """
    obstacle_distance_fn = create_obstacle_distance_fn(model, data, enable_noise=False)

    planner = AdaptiveBSplinePlanner(
        WAYPOINTS,
        obstacle_distance_fn=obstacle_distance_fn,
        safe_distance=s2_config.SAFE_DISTANCE,
        gain=s2_config.PERTURB_GAIN,
        max_shift=s2_config.MAX_PERTURB_SHIFT,
        max_iters=s2_config.MAX_PLANNER_ITERS,
        degree=s2_config.BSPLINE_DEGREE,
    )
    traj = planner.plan()

    pos_ctrl = PositionController(
        kp=s2_config.POSITION_KP,
        kd=s2_config.POSITION_KD,
        mass=s2_config.MASS,
        gravity=s2_config.GRAVITY,
        max_tilt_deg=s2_config.MAX_TILT_DEG,
    )
    attitude_ctrl = AttitudeController(
        kp=s1_config.ATTITUDE_KP,
        kd=s1_config.ATTITUDE_KD,
        inertia_diag=s1_config.INERTIA_DIAG,
        feedforward=True,
    )
    mixer = MotorMixer(
        motor_positions=s1_config.MOTOR_POSITIONS,
        motor_spin=s1_config.MOTOR_SPIN,
        yaw_torque_coeff=s1_config.YAW_TORQUE_COEFF,
        thrust_min=s1_config.THRUST_MIN_PER_MOTOR,
        thrust_max=s1_config.THRUST_MAX_PER_MOTOR,
    )

    return planner, traj, pos_ctrl, attitude_ctrl, mixer, obstacle_distance_fn


def nearest_obstacle_distance(obstacle_distance_fn, point: np.ndarray) -> float:
    """
    Probe the same fixed direction set Stage 2's planner uses
    (``DEFAULT_PROBE_DIRECTIONS``) to get a real-time "nearest obstacle"
    reading for logging/plotting -- this mirrors
    ``AdaptiveBSplinePlanner._nearest_obstacle`` exactly, but as a free
    function so Stage 4 doesn't reach into Stage 2's private method.
    """
    best = np.inf
    for direction in DEFAULT_PROBE_DIRECTIONS:
        dist = obstacle_distance_fn(point, direction)
        if dist < best:
            best = dist
    return best


def reset_drone_state(data: mujoco.MjData):
    """Place the drone at the first waypoint, level, at rest."""
    data.qpos[0:3] = WAYPOINTS[0]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
