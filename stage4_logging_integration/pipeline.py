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
from stage1_dynamics_attitude.attitude_estimator import AttitudeEstimator
from stage1_dynamics_attitude.mixer import MotorMixer
from stage1_dynamics_attitude import config as s1_config

from stage2_trajectory_planning import config as s2_config
from stage2_trajectory_planning.adaptive_planner import (
    DEFAULT_PROBE_DIRECTIONS,
    ReactiveReplanner,
)
from stage2_trajectory_planning.bspline import BSplineTrajectory
from stage2_trajectory_planning.position_controller import PositionController

from stage3_sensing_environment import create_obstacle_distance_fn
from stage3_sensing_environment.imu_sensor import IMUSensor

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(REPO_ROOT, "models", "quadrotor.xml")

# Extends Stage 2's own demo route (see
# stage2_trajectory_planning/run_trajectory_demo.py) with one extra approach
# waypoint, starting clear of *both* obstacles in models/quadrotor.xml
# (>DETECTION_RANGE from each -- see the distance check in
# stage2_trajectory_planning/config.py's DETECTION_RANGE comment). The
# original route's start point is only ~1.0m from obstacle_1's surface and
# ~0.8m from obstacle_2's, both closer than ReactiveReplanner's
# DETECTION_RANGE, so it would detect and react on literally the first sim
# step -- correct, but with no visible stretch of "blind" flight along the
# naive curve first.
WAYPOINTS = np.array([
    [-5.0, 0.0, 1.0],
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

    The planner is a ``ReactiveReplanner``: it starts by flying the naive
    curve straight through the raw waypoints -- no pre-flight obstacle scan,
    so the drone doesn't know an obstacle exists at takeoff -- and only
    replans the remaining route once the live sensor detects something
    within ``s2_config.DETECTION_RANGE`` of the drone's current position
    (call ``planner.maybe_replan(t, pos)`` once per sim step). ``planner``
    and ``traj`` are the same object -- both names are returned for
    call-site clarity (one reads as "the plan", the other as "the thing
    being tracked").

    Returns: (planner, traj, pos_ctrl, attitude_ctrl, mixer, obstacle_distance_fn)
    """
    obstacle_distance_fn = create_obstacle_distance_fn(model, data, enable_noise=False)

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
    mixer = MotorMixer(
        motor_positions=s1_config.MOTOR_POSITIONS,
        motor_spin=s1_config.MOTOR_SPIN,
        yaw_torque_coeff=s1_config.YAW_TORQUE_COEFF,
        thrust_min=s1_config.THRUST_MIN_PER_MOTOR,
        thrust_max=s1_config.THRUST_MAX_PER_MOTOR,
    )

    return planner, traj, pos_ctrl, attitude_ctrl, mixer, obstacle_distance_fn


def build_imu_and_estimator(model: mujoco.MjModel, data: mujoco.MjData):
    """
    Build the Stage 3 IMU (gyro + accelerometer, noise + gyro-bias drift)
    and Stage 1's Mahony-filter ``AttitudeEstimator`` that fuses it into a
    quaternion estimate -- both ``run_full_demo.py`` and ``live_viewer.py``
    feed ``attitude_ctrl.compute_torque`` this estimate (and the raw gyro
    for rate feedback) rather than MuJoCo's ground-truth ``qpos[3:7]``, so
    the pipeline tracks what a real flight controller would actually see.

    The estimator is seeded with ``data``'s current true attitude to model
    a pre-arm leveling calibration, isolating steady-state tracking noise
    from cold-start convergence transients. Call after ``reset_drone_state``
    (or any other pose initialization) so the seed matches the drone's
    actual starting attitude.

    Returns: (imu: IMUSensor, estimator: AttitudeEstimator)
    """
    imu = IMUSensor(model, data, seed=7)
    estimator = AttitudeEstimator(
        kp=s1_config.ESTIMATOR_KP, ki=s1_config.ESTIMATOR_KI,
        q0=data.qpos[3:7].copy(),
    )
    return imu, estimator


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


# --- Interactive-viewer visuals ------------------------------------------------
# The waypoints, adaptively-adjusted control points, and planned B-spline path
# aren't MuJoCo bodies/geoms -- they only ever showed up in the static PNG
# plots each demo saves. In ``viewer.launch_passive`` (live_viewer.py /
# run_full_demo.py --viewer) they'd otherwise be entirely invisible, so we
# draw them as ad-hoc markers into the viewer's ``user_scn`` instead.

_ORIGINAL_WAYPOINT_RGBA = (0.6, 0.6, 0.6, 0.9)   # gray
_ADJUSTED_WAYPOINT_RGBA = (1.0, 0.85, 0.0, 0.9)  # yellow
_NAIVE_PATH_RGBA = (0.9, 0.15, 0.15, 0.45)       # red   -- original plan, pre-avoidance
_PLANNED_PATH_RGBA = (0.1, 0.8, 1.0, 0.65)       # cyan  -- final plan actually flown


def _add_marker(scn, pos, size: float, rgba, geom_type=mujoco.mjtGeom.mjGEOM_SPHERE):
    if scn.ngeom >= scn.maxgeom:
        return
    geom = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(
        geom, geom_type,
        np.array([size, 0.0, 0.0]),
        np.asarray(pos, dtype=float),
        np.eye(3).flatten(),
        np.array(rgba, dtype=np.float32),
    )
    scn.ngeom += 1


def draw_planned_scene(viewer_ctx, planner: ReactiveReplanner, n_path_samples: int = 60):
    """
    Draw, as static markers in a passive viewer:
      - the original waypoints (gray spheres)
      - the *currently active* segment's obstacle-adjusted control points
        (yellow spheres)
      - the naive pre-avoidance curve straight through the raw waypoints
        (red dots) -- what the planner *would* have flown with no obstacle
        avoidance at all, for comparison
      - the currently active planned curve actually being flown from here
        on (cyan dots)

    Call again after ``planner.maybe_replan(...)`` returns True to refresh
    the yellow/cyan markers with the newly re-planned route -- neither
    curve is a MuJoCo geom, so without this they're invisible even though
    the adaptive re-route is the whole point of Stage 2/4.
    """
    scn = viewer_ctx.user_scn
    scn.ngeom = 0

    for wp in planner.waypoints:
        _add_marker(scn, wp, 0.05, _ORIGINAL_WAYPOINT_RGBA)
    for wp in planner.adjusted_waypoints[1:-1]:
        _add_marker(scn, wp, 0.04, _ADJUSTED_WAYPOINT_RGBA)

    naive_traj = BSplineTrajectory(planner.waypoints, duration=planner.duration, degree=planner.degree)
    for s in np.linspace(0.0, 1.0, n_path_samples):
        _add_marker(scn, naive_traj.position(s * naive_traj.duration), 0.015, _NAIVE_PATH_RGBA)

    for pos in planner.sample_active_curve(n_path_samples):
        _add_marker(scn, pos, 0.02, _PLANNED_PATH_RGBA)


def frame_camera(viewer_ctx, model: mujoco.MjModel, data: mujoco.MjData,
                  planner: ReactiveReplanner):
    """
    Point the viewer's free camera at the whole flight envelope (planned
    path + obstacles) instead of the default tight view around the drone's
    spawn point, which is why the drone previously looked like it flew
    straight off-screen with nothing else in view. Call once, before flight
    starts -- it frames the *original* full route, not whatever the active
    replanned segment happens to be, so it doesn't need re-centering on
    every replan.
    """
    samples = np.array([planner.position(s * planner.duration) for s in np.linspace(0.0, 1.0, 20)])
    obstacle_pos = [
        data.xpos[i].copy()
        for i in range(model.nbody)
        if model.body(i).name.startswith("obstacle")
    ]

    parts = [samples, planner.waypoints]
    if obstacle_pos:
        parts.append(np.array(obstacle_pos))
    pts = np.vstack(parts)

    lo, hi = pts.min(axis=0), pts.max(axis=0)
    extent = float(np.max(hi - lo))

    viewer_ctx.cam.lookat[:] = (lo + hi) / 2.0
    viewer_ctx.cam.distance = max(extent * 1.6, 2.5)
    viewer_ctx.cam.azimuth = 130
    viewer_ctx.cam.elevation = -25
