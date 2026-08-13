"""
Stage 2 -- Trajectory & Adaptive Planning.

Public interface for other stages to import:

    from stage2_trajectory_planning import (
        BSplineTrajectory, AdaptiveBSplinePlanner, PositionController, config,
    )

Pipeline: waypoints -> AdaptiveBSplinePlanner (perturbs control points using
Stage 3's get_obstacle_distance) -> BSplineTrajectory (position/velocity/
acceleration/yaw vs. time) -> PositionController (per-step position/velocity
error -> thrust_des, q_des) -> Stage 1's AttitudeController.compute_torque
-> MotorMixer.allocate -> motor commands.
"""

from .bspline import BSpline, BSplineTrajectory
from .adaptive_planner import AdaptiveBSplinePlanner, default_obstacle_distance_fn
from .position_controller import PositionController
from . import config

__all__ = [
    "BSpline",
    "BSplineTrajectory",
    "AdaptiveBSplinePlanner",
    "default_obstacle_distance_fn",
    "PositionController",
    "config",
]
