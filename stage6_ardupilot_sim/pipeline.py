"""
Pipeline construction for Stage 6 ArduPilot SITL / simulation demo.

Mirrors Stage 4 and Stage 5 pipeline configuration:
  - Exact same WAYPOINTS route (starting at [-5, 0, 1])
  - Stage 2 ReactiveReplanner with online obstacle detection and control-point perturbation
  - Ray-cast obstacle sensing conforming to Stage 3 contract
"""

import numpy as np

from stage2_trajectory_planning import config as s2_config
from stage2_trajectory_planning.adaptive_planner import (
    DEFAULT_PROBE_DIRECTIONS,
    ReactiveReplanner,
)
from .environment import OBSTACLES, create_obstacle_distance_fn

# Extends Stage 2's demo route with an approach waypoint starting well clear
# of both obstacles (>DETECTION_RANGE from each), matching Stage 4 & Stage 5.
WAYPOINTS = np.array([
    [-5.0, 0.0, 1.0],
    [0.0, 0.0, 1.0],
    [0.8, 0.3, 1.2],
    [1.5, 0.5, 1.4],
    [2.2, 0.2, 1.2],
    [3.0, 0.0, 1.0],
], dtype=float)


def nearest_obstacle_distance(obstacle_distance_fn, point: np.ndarray) -> float:
    """
    Probes standard probe directions to find the nearest sensed obstacle distance,
    matching Stage 4 & Stage 5 for logging and clearance evaluation.
    """
    best = np.inf
    for direction in DEFAULT_PROBE_DIRECTIONS:
        dist = obstacle_distance_fn(point, direction)
        if dist < best:
            best = dist
    return float(best)


def build_pipeline(enable_sensor_noise: bool = False):
    """
    Constructs the ReactiveReplanner and sensing function for Stage 6.

    Returns:
        planner: ReactiveReplanner instance
        traj: Active B-spline trajectory tracker
        obstacle_distance_fn: Ray-cast sensor query function
    """
    obstacle_distance_fn = create_obstacle_distance_fn(
        obstacles=OBSTACLES,
        enable_noise=enable_sensor_noise,
    )

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

    return planner, traj, obstacle_distance_fn
