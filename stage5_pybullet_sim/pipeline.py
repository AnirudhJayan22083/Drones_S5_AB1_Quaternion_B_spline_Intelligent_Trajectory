"""
Stage 5 pipeline construction: identical control/planning stack to Stage 4
(``stage4_logging_integration.pipeline.build_pipeline``), wired to the
PyBullet backend instead of MuJoCo. Reuses the same WAYPOINTS route as
Stage 4 so the two are directly comparable.
"""

import numpy as np

from stage1_dynamics_attitude.attitude_controller import AttitudeController
from stage1_dynamics_attitude import config as s1_config

from stage2_trajectory_planning import config as s2_config
from stage2_trajectory_planning.adaptive_planner import ReactiveReplanner
from stage2_trajectory_planning.position_controller import PositionController

from stage4_logging_integration.pipeline import WAYPOINTS

from . import pybullet_backend as backend


def build_pipeline(client: int, obstacle_ids):
    """
    Build the Stage 1+2+3 control/planning stack against a live PyBullet
    ``client``, given the obstacle body ids from ``backend.build_world``.
    Mirrors ``stage4_logging_integration.pipeline.build_pipeline`` exactly,
    aside from swapping the MuJoCo-backed obstacle-distance function for
    PyBullet's ``rayTest``-based one.

    Returns: (planner, traj, pos_ctrl, attitude_ctrl, obstacle_distance_fn)
    -- no ``mixer`` (PyBullet's ``apply_motor_thrusts`` still needs
    ``MotorMixer.allocate``'s output, but forces are applied directly to
    the body rather than through MuJoCo actuators, so the mixer is built
    and used inside ``run_pybullet_demo.py`` alongside the others).
    """
    obstacle_distance_fn = backend.create_obstacle_distance_fn(client, obstacle_ids, enable_noise=False)

    planner = ReactiveReplanner(
        WAYPOINTS,
        obstacle_distance_fn=obstacle_distance_fn,
        seconds_per_segment=s2_config.SECONDS_PER_SEGMENT,
        detection_range=s2_config.DETECTION_RANGE,
        rearm_range=s2_config.REARM_RANGE,
        min_replan_gap=s2_config.MIN_REPLAN_GAP,
        safe_distance=s2_config.SAFE_DISTANCE,
        gain=s2_config.PERTURB_GAIN,
        max_shift=s2_config.MAX_PERTURB_SHIFT,
        max_iters=s2_config.MAX_PLANNER_ITERS,
        degree=s2_config.BSPLINE_DEGREE,
        replan_speed_factor=s2_config.REPLAN_SPEED_FACTOR,
    )
    traj = planner

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

    return planner, traj, pos_ctrl, attitude_ctrl, obstacle_distance_fn
