"""
Stage 4 deliverable demo: full pipeline integration test.

Runs Stage 1 (attitude) + Stage 2 (adaptive B-spline + position control) +
Stage 3 (REAL IR ray-cast sensing, not Stage 2's synthetic stand-in)
together end to end in MuJoCo, logs every step through ``FlightLogger``,
and saves a combined tracking + live-obstacle-clearance plot.

This is the integration check Worker 4 owns: if any two stages' contracts
have drifted apart (a changed function signature, a config value one
stage updated but another didn't), this is where it surfaces as a crash
or a garbage plot -- not just each stage's own demo passing in isolation.

Usage:
    python -m stage4_logging_integration.run_full_demo             # headless, saves a plot
    python -m stage4_logging_integration.run_full_demo --viewer    # interactive 3D viewer
"""

import argparse
import os
import time

import matplotlib
import mujoco
import numpy as np

from stage1_dynamics_attitude import quaternion_utils as qu

from .logger import FlightLogger
from .pipeline import MODEL_PATH, build_pipeline, nearest_obstacle_distance, reset_drone_state

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT_DIR = os.path.join(REPO_ROOT, "outputs")


def run(use_viewer: bool = False):
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    reset_drone_state(data)
    mujoco.mj_forward(model, data)

    planner, traj, pos_ctrl, attitude_ctrl, mixer, obstacle_distance_fn = build_pipeline(model, data)
    duration = traj.duration
    logger = FlightLogger()

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

        obstacle_dist = nearest_obstacle_distance(obstacle_distance_fn, pos)

        mujoco.mj_step(model, data)
        logger.record(t, pos, pos_des, q, q_des, omega, motor_thrusts, obstacle_dist)

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

    print(logger.summary())
    save_plot(logger, planner)
    return logger


def save_plot(logger: FlightLogger, planner):
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    arrs = logger.as_arrays()
    t = arrs["t"]
    pos = arrs["pos"]
    pos_des = arrs["pos_des"]
    obstacle_dist = arrs["obstacle_dist"]

    fig, axes = plt.subplots(2, 2, figsize=(11, 9))

    ax = axes[0, 0]
    ax.plot(pos_des[:, 0], pos_des[:, 1], "--", label="desired path")
    ax.plot(pos[:, 0], pos[:, 1], label="actual path")
    ax.plot(planner.waypoints[:, 0], planner.waypoints[:, 1], "o", color="gray", label="original waypoints")
    ax.plot(planner.adjusted_waypoints[:, 0], planner.adjusted_waypoints[:, 1], "x", color="red",
            label="adjusted control points")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Top-down path -- Stage 4 full integration")
    ax.legend(fontsize=8)
    ax.axis("equal")
    ax.grid(True, alpha=0.3)

    err = np.linalg.norm(pos - pos_des, axis=1)
    axes[0, 1].plot(t, err)
    axes[0, 1].set_ylabel("Position error (m)")
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].set_title("Tracking error")
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].plot(t, obstacle_dist)
    axes[1, 0].axhline(0.5, color="red", linestyle=":", label="safety margin")
    axes[1, 0].set_ylabel("Nearest obstacle distance (m)")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_title("Live IR-sensed obstacle clearance (real Stage 3 sensor)")
    axes[1, 0].legend(fontsize=8)
    axes[1, 0].grid(True, alpha=0.3)

    euler = np.array([qu.quat_to_euler_deg(q) for q in arrs["q"]])
    euler_des = np.array([qu.quat_to_euler_deg(q) for q in arrs["q_des"]])
    for i, label in enumerate(["roll", "pitch", "yaw"]):
        axes[1, 1].plot(t, euler[:, i], label=f"{label} actual")
        axes[1, 1].plot(t, euler_des[:, i], "--", label=f"{label} desired")
    axes[1, 1].set_ylabel("Angle (deg)")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].legend(ncol=3, fontsize=8)
    axes[1, 1].set_title("Attitude tracking")
    axes[1, 1].grid(True, alpha=0.3)

    fig.tight_layout()
    out_path = os.path.join(OUTPUT_DIR, "stage4_full_integration.png")
    fig.savefig(out_path, dpi=150)
    print(f"Saved integration plot to {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true", help="open the interactive MuJoCo viewer")
    args = parser.parse_args()
    run(use_viewer=args.viewer)


if __name__ == "__main__":
    main()
