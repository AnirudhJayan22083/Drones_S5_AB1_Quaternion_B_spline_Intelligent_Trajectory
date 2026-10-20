"""
Stage 5 deliverable demo: the exact same Stage 1 (attitude) + Stage 2
(adaptive B-spline + reactive replanning + position control) + Stage 3
(ray-cast sensing + IMU) stack as Stage 4's ``run_full_demo.py``, run
against PyBullet physics instead of MuJoCo -- proof that nothing in the
control/planning code is tied to a specific physics engine.

Usage:
    python -m stage5_pybullet_sim.run_pybullet_demo             # headless, saves a plot
    python -m stage5_pybullet_sim.run_pybullet_demo --viewer    # interactive PyBullet GUI
"""

import argparse
import os
import time

import matplotlib
import numpy as np
import pybullet as p

from stage1_dynamics_attitude import config as s1_config
from stage1_dynamics_attitude import quaternion_utils as qu
from stage1_dynamics_attitude.attitude_estimator import AttitudeEstimator
from stage1_dynamics_attitude.mixer import MotorMixer

from stage4_logging_integration.logger import FlightLogger
from stage4_logging_integration.pipeline import WAYPOINTS, nearest_obstacle_distance

from . import pybullet_backend as backend
from .pipeline import build_pipeline

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT_DIR = os.path.join(REPO_ROOT, "outputs")


def run(use_viewer: bool = False):
    dt = s1_config.SIM_TIMESTEP

    client = backend.connect(use_gui=use_viewer)
    start_pos = WAYPOINTS[0]

    drone_id, obstacle_ids = backend.build_world(client, start_position=start_pos)
    backend.reset_drone_state(client, drone_id, start_pos)

    planner, traj, pos_ctrl, attitude_ctrl, obstacle_distance_fn = build_pipeline(client, obstacle_ids)
    mixer = MotorMixer(
        motor_positions=s1_config.MOTOR_POSITIONS,
        motor_spin=s1_config.MOTOR_SPIN,
        yaw_torque_coeff=s1_config.YAW_TORQUE_COEFF,
        thrust_min=s1_config.THRUST_MIN_PER_MOTOR,
        thrust_max=s1_config.THRUST_MAX_PER_MOTOR,
    )
    imu = backend.PyBulletIMU(client, drone_id, seed=7)
    _, _, q0, _ = backend.get_state(client, drone_id)
    estimator = AttitudeEstimator(kp=s1_config.ESTIMATOR_KP, ki=s1_config.ESTIMATOR_KI, q0=q0)

    logger = FlightLogger()

    if use_viewer:
        p.resetDebugVisualizerCamera(cameraDistance=6.0, cameraYaw=50, cameraPitch=-30,
                                      cameraTargetPosition=[0.0, 0.0, 1.0], physicsClientId=client)

    t = 0.0
    wall_start = time.time()
    while t < planner.duration:
        pos, vel, _, _ = backend.get_state(client, drone_id)

        accel_meas, gyro_meas = imu.read(dt)
        q_est = estimator.update(accel_meas, gyro_meas, dt)

        planner.maybe_replan(t, pos, vel)

        pos_des = traj.position(t)
        vel_des = traj.velocity(t)
        acc_des = traj.acceleration(t)
        yaw_des = traj.yaw(t)

        thrust_des, q_des = pos_ctrl.compute(pos, vel, pos_des, vel_des, acc_des, yaw_des)
        # Rate feedback comes straight off the raw gyro, matching Stage 4 --
        # only attitude needs the accel/gyro fusion.
        torque = attitude_ctrl.compute_torque(q_est, gyro_meas, q_des)
        motor_thrusts = mixer.allocate(thrust_des, torque)
        backend.apply_motor_thrusts(client, drone_id, motor_thrusts)

        obstacle_dist = nearest_obstacle_distance(obstacle_distance_fn, pos)

        p.stepSimulation(physicsClientId=client)
        logger.record(t, pos, pos_des, q_est, q_des, gyro_meas, motor_thrusts, obstacle_dist)

        if use_viewer:
            if not p.isConnected(client):
                break
            elapsed_wall = time.time() - wall_start
            sleep_time = t - elapsed_wall
            if sleep_time > 0:
                time.sleep(sleep_time)

        t += dt

    p.disconnect(client)

    print(logger.summary())
    save_plot(logger)
    return logger


def save_plot(logger: FlightLogger):
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
    for obs in backend.OBSTACLES:
        cx, cy = obs["position"][0], obs["position"][1]
        r = obs.get("radius", max(obs.get("half_extents", [0.3, 0.3, 0.3])[:2]))
        ax.add_patch(plt.Circle((cx, cy), r, color="red", alpha=0.25))
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Top-down path -- Stage 5 (PyBullet)")
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
    axes[1, 0].set_title("Live ray-cast obstacle clearance (PyBullet rayTest)")
    axes[1, 0].legend(fontsize=8)
    axes[1, 0].grid(True, alpha=0.3)

    euler = np.array([qu.quat_to_euler_deg(q) for q in arrs["q"]])
    euler_des = np.array([qu.quat_to_euler_deg(q) for q in arrs["q_des"]])
    for i, label in enumerate(["roll", "pitch", "yaw"]):
        axes[1, 1].plot(t, euler[:, i], label=f"{label} IMU est.")
        axes[1, 1].plot(t, euler_des[:, i], "--", label=f"{label} desired")
    axes[1, 1].set_ylabel("Angle (deg)")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].legend(ncol=3, fontsize=8)
    axes[1, 1].set_title("Attitude tracking (IMU/Mahony estimate vs. desired)")
    axes[1, 1].grid(True, alpha=0.3)

    fig.tight_layout()
    out_path = os.path.join(OUTPUT_DIR, "stage5_pybullet_integration.png")
    out_tracking = os.path.join(OUTPUT_DIR, "stage5_pybullet_tracking.png")
    fig.savefig(out_path, dpi=150)
    fig.savefig(out_tracking, dpi=150)
    print(f"Saved integration plot to {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true", help="open the interactive PyBullet GUI")
    args = parser.parse_args()
    run(use_viewer=args.viewer)


if __name__ == "__main__":
    main()
