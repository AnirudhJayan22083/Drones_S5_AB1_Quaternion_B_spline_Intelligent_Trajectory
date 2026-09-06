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
# Obstacle distance readings (from Stage 3's ray-cast sensor) are measured
# from a single point -- the control point / flown position -- to the
# obstacle surface. That's the drone's *center*, not its physical extent, so
# a point-mass margin alone lets the airframe (arms + rotors) clip an
# obstacle it technically "cleared". DRONE_RADIUS folds that physical
# footprint into the margin so SAFE_DISTANCE is clearance for the actual
# vehicle body, not just its center point.
DRONE_RADIUS = stage1_config.ARM_LENGTH + 0.08  # m, arm length + rotor/prop allowance

# A control point closer to an obstacle than SAFE_DISTANCE gets pushed away
# from it; the push magnitude grows linearly with the intrusion depth,
# scaled by PERTURB_GAIN and capped at MAX_PERTURB_SHIFT so a single close
# reading can't fling a control point across the map.
SAFE_DISTANCE = 0.5 + DRONE_RADIUS  # m, obstacle-surface to vehicle-body clearance
PERTURB_GAIN = 1.0
MAX_PERTURB_SHIFT = 0.6  # m per iteration
MAX_PLANNER_ITERS = 8

# How close (m, from the drone's live position to the obstacle surface) an
# obstacle has to be before ReactiveReplanner treats it as "detected" and
# replans the remaining route -- instead of flying the naive, obstacle-blind
# curve for the whole flight.
#
# Defined directly off DRONE_RADIUS (not SAFE_DISTANCE) because what
# actually matters physically is body-surface-to-obstacle-surface
# clearance at the moment of detection, not the sensor's raw
# center-to-obstacle reading. TRIGGER_BODY_CLEARANCE below is that
# clearance. It is NOT a free "closer feels more realistic" knob --
# full-physics closed-loop simulation (real Stage 3 ray sensor, Stage 1/2
# control loops, MuJoCo dynamics) shows a hard collision cliff just below
# 0.24m: at 0.22m body clearance the drone still clips the obstacle by
# ~4cm, and it gets worse fast the closer the trigger gets (5cm trigger ->
# ~11.5cm penetration) simply because there isn't enough distance left, at
# this vehicle's flight speed, for a replan + control response to change
# the flight path before momentum carries it into the obstacle. 0.30m
# clears that cliff with a verified ~13cm of spare body clearance -- close
# enough to read as "reacting right at the obstacle" rather than meters
# out, without being fragile to small perturbations. See adaptive_planner.py.
TRIGGER_BODY_CLEARANCE = 0.30  # m, verified collision-free via simulation sweep
DETECTION_RANGE = DRONE_RADIUS + TRIGGER_BODY_CLEARANCE

# ReactiveReplanner re-arms (becomes eligible to trigger another replan)
# only once distance climbs back above REARM_RANGE, and never fires two
# replans closer together than MIN_REPLAN_GAP seconds -- both guard against
# chattering: the "nearest obstacle" reading is a min over a fixed set of
# probe directions, not a true continuous distance, so it can jump back
# above a single shared threshold for a step or two while the drone is
# still genuinely close (e.g. mid-turn), which would otherwise re-trigger a
# fresh replan before the previous one had a chance to actually clear the
# obstacle.
REARM_RANGE = DETECTION_RANGE + 0.1
MIN_REPLAN_GAP = 1.0  # s

# Reacting this close to an obstacle (see TRIGGER_BODY_CLEARANCE above)
# means the avoidance curve bends sharply right where the drone already is
# -- flown at the same cruise speed as the calm, unobstructed legs, that
# sharp bend demands a sudden burst of lateral acceleration, which is what
# reads as a hard jerk to the side. A real pilot backs off the throttle for
# a last-second dodge instead of powering through it at cruise speed, so
# ReactiveReplanner gives the freshly-replanned segment more time to cover
# its (often longer, detour-including) path -- only ever slower than the
# nominal schedule, never faster. 0.4 = the replanned segment cruises at
# 40% of the flight's normal speed.
REPLAN_SPEED_FACTOR = 0.4

# Fallback sensor range used by the placeholder obstacle-distance stub (see
# adaptive_planner.default_obstacle_distance_fn) when no real distance
# function -- i.e. Stage 3's get_obstacle_distance -- has been wired in yet.
STUB_SENSOR_RANGE = 10.0
