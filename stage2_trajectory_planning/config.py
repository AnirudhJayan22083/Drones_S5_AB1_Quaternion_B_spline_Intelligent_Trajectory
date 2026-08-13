"""
Parameters for Stage 2 -- trajectory generation, position control, and
adaptive obstacle avoidance.

Reuses mass/gravity from Stage 1's config so the two stages never disagree
about the airframe. See stage1_dynamics_attitude/config.py for the physical
parameters this builds on.
"""

import numpy as np

from stage1_dynamics_attitude import config as stage1_config

MASS = stage1_config.MASS
GRAVITY = stage1_config.GRAVITY

# --- Position controller gains ------------------------------------------------
# Proportional (position error) and derivative (velocity error) gains, per
# world axis [x, y, z]. Z is stiffer than X/Y since altitude has no coupling
# through the attitude loop the way horizontal motion does (horizontal
# accel is produced by tilting, which costs one extra control lag).
POSITION_KP = np.array([3.0, 3.0, 6.0])
POSITION_KD = np.array([2.5, 2.5, 4.0])

# Maximum commanded tilt away from vertical (deg). Caps how aggressively the
# position controller can lean the airframe over to chase large position
# errors, so it doesn't request attitudes the Stage-1 loop can't recover
# gracefully from.
MAX_TILT_DEG = 35.0

# --- B-spline trajectory ------------------------------------------------------
BSPLINE_DEGREE = 3  # cubic
SECONDS_PER_SEGMENT = 2.0  # default time budget per waypoint-to-waypoint leg

# --- Adaptive control-point perturbation --------------------------------------
# A control point closer to an obstacle than SAFE_DISTANCE gets pushed away
# from it; the push magnitude grows linearly with the intrusion depth,
# scaled by PERTURB_GAIN and capped at MAX_PERTURB_SHIFT so a single close
# reading can't fling a control point across the map.
SAFE_DISTANCE = 0.5  # m
PERTURB_GAIN = 1.0
MAX_PERTURB_SHIFT = 0.6  # m per iteration
MAX_PLANNER_ITERS = 5

# Fallback sensor range used by the placeholder obstacle-distance stub (see
# adaptive_planner.default_obstacle_distance_fn) when no real distance
# function -- i.e. Stage 3's get_obstacle_distance -- has been wired in yet.
STUB_SENSOR_RANGE = 10.0
