"""
Stage 2 deliverable demo: adaptively-planned B-spline trajectory tracked
through Stage 1's attitude controller, end to end in MuJoCo.

The waypoint path runs close enough to ``obstacle_1`` in
``models/quadrotor.xml`` that the adaptive planner has something to react
to. Stage 3's ray-casting sensor doesn't exist yet, so
``synthetic_obstacle_distance_fn`` below stands in for it: a plain
ray-sphere check against the two obstacle bodies' (hardcoded) world
positions. It satisfies the same ``get_obstacle_distance(pos, direction)``
contract, so swapping in the real Stage-3 function later is a one-line
change in ``build_planner``.

Usage:
    python -m stage2_trajectory_planning.run_trajectory_demo             # headless, saves a plot
    python -m stage2_trajectory_planning.run_trajectory_demo --viewer    # interactive 3D viewer
"""

import argparse
import os
import time

import matplotlib
import mujoco
import numpy as np

from stage1_dynamics_attitude import quaternion_utils as qu
from stage1_dynamics_attitude.attitude_controller import AttitudeController
from stage1_dynamics_attitude.mixer import MotorMixer
from stage1_dynamics_attitude import config as s1_config

from . import config
from .adaptive_planner import AdaptiveBSplinePlanner
from .position_controller import PositionController

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(REPO_ROOT, "models", "quadrotor.xml")
OUTPUT_DIR = os.path.join(REPO_ROOT, "outputs")

# Obstacle geometry copied from models/quadrotor.xml (Stage 3 will read this
# from the model directly via mj_ray instead of hardcoding it here). Radii
# are bounding spheres around each geom's full extent -- box (0.15, 0.15,
# 0.5) half-sizes -> sqrt(0.15^2+0.15^2+0.5^2), cylinder (0.2 radius, 0.75
# half-height) -> sqrt(0.2^2+0.75^2) -- so the planner treats the whole
# obstacle as covered, not just its footprint.
_OBSTACLES = [
    {"center": np.array([1.5, 0.5, 1.0]), "radius": 0.55},   # box -> bounding sphere
    {"center": np.array([-1.2, -1.0, 0.75]), "radius": 0.78},  # cylinder -> bounding sphere
]

WAYPOINTS = np.array([
    [0.0, 0.0, 1.0],
    [0.8, 0.3, 1.2],
    [1.5, 0.5, 1.4],
    [2.2, 0.2, 1.2],
    [3.0, 0.0, 1.0],
])


def synthetic_obstacle_distance_fn(position: np.ndarray, direction: np.ndarray,
                                    max_range: float = config.STUB_SENSOR_RANGE) -> float:
    """Ray-sphere distance to the nearest obstacle, clipped to max_range."""
    direction = direction / np.linalg.norm(direction)
    best = max_range
    for obs in _OBSTACLES:
        oc = obs["center"] - position
        r2 = obs["radius"] ** 2
        dist_to_center_sq = np.dot(oc, oc)
        if dist_to_center_sq <= r2:
            # position is already inside the obstacle -- report contact
            # regardless of ray direction, rather than only checking for
            # hits ahead of the ray (which would miss this entirely).
            best = 0.0
            continue
        proj = np.dot(oc, direction)
        if proj < 0:
            continue
        perp_sq = dist_to_center_sq - proj ** 2
        if perp_sq > r2:
            continue
        thc = np.sqrt(r2 - perp_sq)
        t0 = proj - thc
        best = min(best, max(t0, 0.0))
    return min(best, max_range)


def build_planner(model: mujoco.MjModel = None, data: mujoco.MjData = None) -> AdaptiveBSplinePlanner:
    obs_fn = synthetic_obstacle_distance_fn
    if model is not None and data is not None:
        try:
            from stage3_sensing_environment import create_obstacle_distance_fn
            obs_fn = create_obstacle_distance_fn(model, data, enable_noise=False)
        except ImportError:
            pass

    return AdaptiveBSplinePlanner(
        WAYPOINTS,
        obstacle_distance_fn=obs_fn,
        safe_distance=config.SAFE_DISTANCE,
        gain=config.PERTURB_GAIN,
        max_shift=config.MAX_PERTURB_SHIFT,
        max_iters=config.MAX_PLANNER_ITERS,
        degree=config.BSPLINE_DEGREE,
    )


def build_controllers():
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
    pos_ctrl = PositionController(
        kp=config.POSITION_KP,
        kd=config.POSITION_KD,
        mass=config.MASS,
        gravity=config.GRAVITY,
        max_tilt_deg=config.MAX_TILT_DEG,
    )
    return pos_ctrl, attitude_ctrl, mixer


def run(use_viewer: bool = False):
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    data.qpos[0:3] = WAYPOINTS[0]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)

    planner = build_planner(model, data)
    traj = planner.plan()
    duration = traj.duration

    pos_ctrl, attitude_ctrl, mixer = build_controllers()

    log = {"t": [], "pos": [], "pos_des": [], "q": [], "q_des": []}

    viewer_ctx = None
    if use_viewer:
        from mujoco import viewer as mujoco_viewer
        viewer_ctx = mujoco_viewer.launch_passive(model, data)

    n_steps = int(duration / model.opt.timestep)
    wall_start = time.time()
    for step in range(n_steps):
        t = data.time
        pos = data.qpos[0:3].copy()
        vel = data.qvel[0:3].copy()
        q = data.qpos[3:7].copy()
        omega = data.qvel[3:6].copy()

        pos_des = traj.position(t)
        vel_des = traj.velocity(t)
        acc_des = traj.acceleration(t)
        yaw_des = traj.yaw(t)

        thrust_des, q_des = pos_ctrl.compute(pos, vel, pos_des, vel_des, acc_des, yaw_des)
        torque = attitude_ctrl.compute_torque(q, omega, q_des)
        motor_thrusts = mixer.allocate(thrust_des, torque)
        data.ctrl[:] = motor_thrusts

        mujoco.mj_step(model, data)

        log["t"].append(t)
        log["pos"].append(pos)
        log["pos_des"].append(pos_des)
        log["q"].append(q)
        log["q_des"].append(q_des)

        if viewer_ctx is not None:
            viewer_ctx.sync()
            elapsed_wall = time.time() - wall_start
            sleep_time = t - elapsed_wall
            if sleep_time > 0:
                time.sleep(sleep_time)
            if not viewer_ctx.is_running():
                break

    if viewer_ctx is not None:
        viewer_ctx.close()

    pos_arr = np.array(log["pos"])
    pos_des_arr = np.array(log["pos_des"])
    tracking_err = np.linalg.norm(pos_arr - pos_des_arr, axis=1)
    print(f"Final position error: {tracking_err[-1]:.3f} m")
    print(f"Mean position error:  {tracking_err.mean():.3f} m")
    print(f"Max position error:   {tracking_err.max():.3f} m")

    save_plot(log, planner)
    return log


def save_plot(log, planner: AdaptiveBSplinePlanner):
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    t = np.array(log["t"])
    pos = np.array(log["pos"])
    pos_des = np.array(log["pos_des"])

    fig, axes = plt.subplots(2, 2, figsize=(11, 9))

    ax = axes[0, 0]
    ax.plot(pos_des[:, 0], pos_des[:, 1], "--", label="desired path")
    ax.plot(pos[:, 0], pos[:, 1], label="actual path")
    ax.plot(planner.waypoints[:, 0], planner.waypoints[:, 1], "o", color="gray", label="original waypoints")
    ax.plot(planner.adjusted_waypoints[:, 0], planner.adjusted_waypoints[:, 1], "x", color="red",
            label="adjusted control points")
    for obs in [{"center": (1.5, 0.5), "r": 0.35}, {"center": (-1.2, -1.0), "r": 0.75}]:
        circle = plt.Circle(obs["center"], obs["r"], color="red", alpha=0.2)
        ax.add_patch(circle)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Top-down path vs. adaptive re-plan")
    ax.legend(fontsize=8)
    ax.axis("equal")
    ax.grid(True, alpha=0.3)

    labels = ["x", "y", "z"]
    for i in range(3):
        axes[0, 1].plot(t, pos[:, i], label=f"{labels[i]} actual")
        axes[0, 1].plot(t, pos_des[:, i], "--", label=f"{labels[i]} desired")
    axes[0, 1].set_ylabel("Position (m)")
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].legend(ncol=3, fontsize=8)
    axes[0, 1].set_title("Position tracking")
    axes[0, 1].grid(True, alpha=0.3)

    err = np.linalg.norm(pos - pos_des, axis=1)
    axes[1, 0].plot(t, err)
    axes[1, 0].set_ylabel("Position error (m)")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_title("Tracking error")
    axes[1, 0].grid(True, alpha=0.3)

    euler = np.array([qu.quat_to_euler_deg(q) for q in log["q"]])
    euler_des = np.array([qu.quat_to_euler_deg(q) for q in log["q_des"]])
    for i, label in enumerate(["roll", "pitch", "yaw"]):
        axes[1, 1].plot(t, euler[:, i], label=f"{label} actual")
        axes[1, 1].plot(t, euler_des[:, i], "--", label=f"{label} desired")
    axes[1, 1].set_ylabel("Angle (deg)")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].legend(ncol=3, fontsize=8)
    axes[1, 1].set_title("Attitude setpoint from position controller")
    axes[1, 1].grid(True, alpha=0.3)

    fig.tight_layout()
    out_path = os.path.join(OUTPUT_DIR, "stage2_trajectory_tracking.png")
    fig.savefig(out_path, dpi=150)
    print(f"Saved tracking plot to {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true", help="open the interactive MuJoCo viewer")
    args = parser.parse_args()
    run(use_viewer=args.viewer)


if __name__ == "__main__":
    main()
