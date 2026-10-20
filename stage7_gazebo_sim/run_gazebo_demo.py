"""
Stage 7 deliverable demo: the exact same Stage 1 (attitude) + Stage 2
(adaptive B-spline + reactive replanning + position control) + Stage 3
(ray-cast sensing + IMU) stack as Stage 4 (MuJoCo) and Stage 5 (PyBullet),
run against Gazebo (gz-sim) physics.

The controller runs inside gz-sim's PreUpdate callback via the Python
``TestFixture``, so it is stepped in lockstep with Gazebo's physics at
``SIM_TIMESTEP``.

Usage (needs Gazebo Harmonic+ and its Python bindings -- see README.md):
    python -m stage7_gazebo_sim.run_gazebo_demo             # headless, saves a plot
    python -m stage7_gazebo_sim.run_gazebo_demo --viewer    # real-time, opens the Gazebo GUI
"""

import argparse
import math
import os
import shutil
import subprocess
import tempfile
import time

import matplotlib
import numpy as np

from stage1_dynamics_attitude import config as s1_config
from stage1_dynamics_attitude import quaternion_utils as qu
from stage1_dynamics_attitude.attitude_estimator import AttitudeEstimator

from stage2_trajectory_planning import config as s2_config
from stage4_logging_integration.logger import FlightLogger
from stage4_logging_integration.pipeline import WAYPOINTS, nearest_obstacle_distance
from stage6_ardupilot_sim.environment import OBSTACLES

from . import gazebo_backend as backend
from .pipeline import build_pipeline

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT_DIR = os.path.join(REPO_ROOT, "outputs")


def _launch_gui():
    """Start the Gazebo GUI client; it attaches to our in-process server over gz-transport."""
    gz_exe = shutil.which("gz")
    if gz_exe is None:
        print("[Stage 7] `gz` not on PATH -- run `gz sim -g` yourself to watch the flight.")
        return None
    print("[Stage 7] Launching Gazebo GUI (gz sim -g)...")
    return subprocess.Popen([gz_exe, "sim", "-g"])


def run(use_viewer: bool = False):
    dt = s1_config.SIM_TIMESTEP
    start_pos = WAYPOINTS[0]

    world_dir = tempfile.mkdtemp(prefix="stage7_gazebo_")
    world_path = os.path.join(world_dir, "quadrotor_world.sdf")
    with open(world_path, "w", encoding="utf-8") as f:
        f.write(backend.build_world_sdf(start_pos, real_time=use_viewer))

    planner, traj, pos_ctrl, attitude_ctrl, mixer, obstacle_distance_fn = build_pipeline()
    drone = backend.GazeboDrone()
    imu = backend.GazeboIMU(seed=7)
    logger = FlightLogger()

    # Mutable loop state shared with the server callback.
    sim = {"t": 0.0, "estimator": None, "done": False, "error": None}

    def on_pre_update(info, ecm):
        if info.paused or sim["done"]:
            return
        try:
            control_step(ecm)
        except Exception as err:
            # Exceptions don't propagate out of the server's C++ loop, so
            # stash it and stop -- otherwise the run loop below never ends.
            sim["error"] = err
            sim["done"] = True

    def control_step(ecm):
        t = sim["t"]
        if t >= planner.duration:
            sim["done"] = True
            return

        if drone.link is None:
            drone.bind(ecm)
        pos, vel, q_true, omega_body = drone.get_state(ecm)

        if sim["estimator"] is None:
            # Seeded with the true starting attitude -- the same pre-arm
            # leveling calibration Stage 4/5 model.
            sim["estimator"] = AttitudeEstimator(kp=s1_config.ESTIMATOR_KP,
                                                 ki=s1_config.ESTIMATOR_KI, q0=q_true)

        accel_meas, gyro_meas = imu.read(vel, q_true, omega_body, dt)
        q_est = sim["estimator"].update(accel_meas, gyro_meas, dt)

        if planner.maybe_replan(t, pos, vel):
            print(f"[t={t:.2f}s] Obstacle sensed -- replanned remaining route "
                  f"({planner.replan_count} replan(s) so far)")

        pos_des = traj.position(t)
        vel_des = traj.velocity(t)
        acc_des = traj.acceleration(t)
        yaw_des = traj.yaw(t)

        thrust_des, q_des = pos_ctrl.compute(pos, vel, pos_des, vel_des, acc_des, yaw_des)
        # Rate feedback straight off the raw gyro, matching Stage 4/5.
        torque = attitude_ctrl.compute_torque(q_est, gyro_meas, q_des)
        motor_thrusts = mixer.allocate(thrust_des, torque)
        drone.apply_motor_thrusts(ecm, q_true, motor_thrusts)

        obstacle_dist = nearest_obstacle_distance(obstacle_distance_fn, pos)
        logger.record(t, pos, pos_des, q_est, q_des, gyro_meas, motor_thrusts, obstacle_dist)

        sim["t"] = t + dt

    fixture = backend.gz_sim.TestFixture(world_path)
    fixture.on_pre_update(on_pre_update)
    fixture.finalize()
    server = fixture.server()

    gui_proc = _launch_gui() if use_viewer else None
    if gui_proc is not None:
        time.sleep(3.0)  # give the GUI a moment to attach before the flight starts

    print(f"[Stage 7] Flying {len(WAYPOINTS)}-waypoint route in Gazebo "
          f"(planned duration {planner.duration:.2f} s)...")
    # Replans can stretch planner.duration mid-flight, so keep running
    # until the callback reports the (possibly extended) route is done.
    while not sim["done"]:
        remaining_steps = max(1, math.ceil((planner.duration - sim["t"]) / dt))
        server.run(True, remaining_steps, False)

    if sim["error"] is not None:
        if gui_proc is not None:
            gui_proc.terminate()
        shutil.rmtree(world_dir, ignore_errors=True)
        raise sim["error"]

    if gui_proc is not None:
        print("[Stage 7] Flight finished -- close the Gazebo window to exit.")
        gui_proc.wait()
    shutil.rmtree(world_dir, ignore_errors=True)

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
    for obs in OBSTACLES:
        cx, cy = obs["position"][0], obs["position"][1]
        if obs["shape"] == "box":
            hx, hy = obs["half_extents"][0], obs["half_extents"][1]
            ax.add_patch(plt.Rectangle((cx - hx, cy - hy), 2 * hx, 2 * hy, color="red", alpha=0.25))
        else:
            ax.add_patch(plt.Circle((cx, cy), obs["radius"], color="red", alpha=0.25))
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Top-down path -- Stage 7 (Gazebo)")
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
    axes[1, 0].axhline(s2_config.SAFE_DISTANCE, color="red", linestyle=":",
                       label=f"safety margin ({s2_config.SAFE_DISTANCE:.2f} m)")
    axes[1, 0].set_ylabel("Nearest obstacle distance (m)")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_title("Live ray-cast obstacle clearance")
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
    out_path = os.path.join(OUTPUT_DIR, "stage7_gazebo_integration.png")
    out_tracking = os.path.join(OUTPUT_DIR, "stage7_gazebo_tracking.png")
    fig.savefig(out_path, dpi=150)
    fig.savefig(out_tracking, dpi=150)
    plt.close(fig)
    print(f"Saved integration plot to {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true",
                        help="run in real time and open the Gazebo GUI")
    args = parser.parse_args()
    run(use_viewer=args.viewer)


if __name__ == "__main__":
    main()
