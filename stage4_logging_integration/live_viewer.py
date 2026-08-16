"""
Stage 4 -- live interactive viewer.

Wires Stage 1's attitude controller + mixer, Stage 2's adaptive B-spline
planner + position controller, and Stage 3's REAL IR ray-cast sensor
together into one MuJoCo ``viewer.launch_passive`` loop, so the whole
pipeline can be watched flying in real time -- not just inspected via the
static PNGs each stage's own demo produces.

Needs a display; will not run headless (use run_full_demo.py for that).

Usage:
    python -m stage4_logging_integration.live_viewer
    python -m stage4_logging_integration.live_viewer --duration 10
"""

import argparse
import time

import mujoco
from mujoco import viewer as mujoco_viewer

from .logger import FlightLogger
from .pipeline import MODEL_PATH, build_pipeline, nearest_obstacle_distance, reset_drone_state


def run(duration: float = None):
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    reset_drone_state(data)
    mujoco.mj_forward(model, data)

    planner, traj, pos_ctrl, attitude_ctrl, mixer, obstacle_distance_fn = build_pipeline(model, data)
    duration = duration if duration is not None else traj.duration
    logger = FlightLogger()

    with mujoco_viewer.launch_passive(model, data) as viewer_ctx:
        wall_start = time.time()
        while data.time < duration and viewer_ctx.is_running():
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

            viewer_ctx.sync()
            elapsed_wall = time.time() - wall_start
            sleep_time = t - elapsed_wall
            if sleep_time > 0:
                time.sleep(sleep_time)

    print(logger.summary())
    return logger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=None,
                         help="override flight duration (s); default is the planned trajectory's duration")
    args = parser.parse_args()
    run(duration=args.duration)


if __name__ == "__main__":
    main()
