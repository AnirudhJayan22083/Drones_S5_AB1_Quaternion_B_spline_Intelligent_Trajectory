"""
Stage 1 deliverable demo: a drone that hovers and holds attitude when given
a target quaternion + thrust.

The drone starts with an initial attitude/rate disturbance. For the first
half of the run the target attitude is level hover (identity quaternion);
partway through, the target steps to a tilted attitude (roll/pitch/yaw) to
show the controller tracking an arbitrary commanded quaternion, not just
recovering to level. Thrust is held at the constant hover value throughout
-- Stage 1 has no position controller yet, so some altitude drift while
tilted is expected (Stage 2 closes that loop).

Usage:
    python -m stage1_dynamics_attitude.run_hover_demo             # headless, saves a plot
    python -m stage1_dynamics_attitude.run_hover_demo --viewer    # interactive 3D viewer
"""

import argparse
import os
import time

import matplotlib
import mujoco
import numpy as np

from stage3_sensing_environment.imu_sensor import IMUSensor

from . import config
from .attitude_controller import AttitudeController
from .attitude_estimator import AttitudeEstimator
from .mixer import MotorMixer
from . import quaternion_utils as qu

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(REPO_ROOT, "models", "quadrotor.xml")
OUTPUT_DIR = os.path.join(REPO_ROOT, "outputs")


def build_controller_and_mixer():
    controller = AttitudeController(
        kp=config.ATTITUDE_KP,
        kd=config.ATTITUDE_KD,
        inertia_diag=config.INERTIA_DIAG,
        feedforward=True,
    )
    mixer = MotorMixer(
        motor_positions=config.MOTOR_POSITIONS,
        motor_spin=config.MOTOR_SPIN,
        yaw_torque_coeff=config.YAW_TORQUE_COEFF,
        thrust_min=config.THRUST_MIN_PER_MOTOR,
        thrust_max=config.THRUST_MAX_PER_MOTOR,
    )
    return controller, mixer


def desired_quat_at(t: float) -> np.ndarray:
    """Target attitude schedule: level hover, then a step to a tilted setpoint."""
    if t < 3.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return qu.euler_to_quat(
        roll=np.radians(15.0), pitch=np.radians(8.0), yaw=np.radians(25.0)
    )


def thrust_for_quat(q_des: np.ndarray) -> float:
    """
    Stage 1 has no altitude/position feedback yet -- ``thrust`` is a given
    input, not something this stage regulates. As a sensible feedforward for
    the demo, scale the hover thrust by the known tilt of q_des so a tilted
    setpoint doesn't starve vertical lift (thrust_world_z = thrust_body *
    cos(tilt), so compensate by dividing by cos(tilt)). This uses only the
    *desired* attitude, not any measured position/altitude, so it stays a
    feedforward choice of the "given thrust" rather than closed-loop
    altitude control (that's Stage 2's job).
    """
    R_des = qu.quat_to_rotmat(q_des)
    cos_tilt = np.clip(R_des[2, 2], 0.2, 1.0)  # avoid blow-up at extreme tilts
    return config.HOVER_THRUST_TOTAL / cos_tilt


def run(duration: float = 6.0, use_viewer: bool = False):
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    # Initial disturbance: tilted attitude + nonzero body rates, away from
    # the first target (level hover), so the plot shows real convergence.
    q0 = qu.euler_to_quat(roll=np.radians(-15.0), pitch=np.radians(10.0), yaw=0.0)
    data.qpos[0:3] = [0.0, 0.0, 1.0]
    data.qpos[3:7] = q0
    data.qvel[3:6] = [0.5, -0.3, 0.2]
    mujoco.mj_forward(model, data)

    controller, mixer = build_controller_and_mixer()

    # IMU (gyro + accelerometer, noise + gyro-bias drift) feeding a Mahony
    # complementary filter -- the controller below tracks this *estimated*
    # quaternion, not MuJoCo's ground truth, so the plot reflects what a
    # real flight controller actually has to work with. The estimator is
    # seeded with the true initial attitude to model a pre-arm leveling
    # calibration, isolating steady-state tracking noise from cold-start
    # convergence (a separate effect, not what this demo is about).
    imu = IMUSensor(model, data, seed=7)
    estimator = AttitudeEstimator(kp=config.ESTIMATOR_KP, ki=config.ESTIMATOR_KI, q0=q0)

    log = {"t": [], "pos": [], "q": [], "q_est": [], "q_des": [], "err_deg": []}

    viewer_ctx = None
    if use_viewer:
        from mujoco import viewer as mujoco_viewer
        viewer_ctx = mujoco_viewer.launch_passive(model, data)

    n_steps = int(duration / model.opt.timestep)
    wall_start = time.time()
    for step in range(n_steps):
        t = data.time
        q = data.qpos[3:7].copy()

        accel_meas, gyro_meas = imu.read()
        q_est = estimator.update(accel_meas, gyro_meas, model.opt.timestep)

        q_des = desired_quat_at(t)
        thrust_des = thrust_for_quat(q_des)

        # Rate feedback also comes straight off the (raw) gyro, matching a
        # real flight controller -- only attitude needs the accel fusion.
        torque = controller.compute_torque(q_est, gyro_meas, q_des)
        motor_thrusts = mixer.allocate(thrust_des, torque)
        data.ctrl[:] = motor_thrusts

        mujoco.mj_step(model, data)

        q_e = qu.quat_error(q_est, q_des)
        err_deg = np.degrees(2.0 * np.arccos(np.clip(abs(q_e[0]), -1.0, 1.0)))
        log["t"].append(t)
        log["pos"].append(data.qpos[0:3].copy())
        log["q"].append(q)
        log["q_est"].append(q_est)
        log["q_des"].append(q_des)
        log["err_deg"].append(err_deg)

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

    final_err = log["err_deg"][-1]
    print(f"Final attitude error: {final_err:.3f} deg")
    print(f"Final altitude: {log['pos'][-1][2]:.3f} m")

    save_plot(log)
    return log


def save_plot(log):
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    t = np.array(log["t"])
    err_deg = np.array(log["err_deg"])
    pos = np.array(log["pos"])
    euler = np.array([qu.quat_to_euler_deg(q) for q in log["q"]])
    euler_est = np.array([qu.quat_to_euler_deg(q) for q in log["q_est"]])
    euler_des = np.array([qu.quat_to_euler_deg(q) for q in log["q_des"]])
    est_err_deg = np.degrees([
        2.0 * np.arccos(np.clip(abs(qu.quat_error(qe, qt)[0]), -1.0, 1.0))
        for qe, qt in zip(log["q_est"], log["q"])
    ])

    fig, axes = plt.subplots(4, 1, figsize=(9, 11), sharex=True)

    axes[0].plot(t, err_deg)
    axes[0].set_ylabel("Attitude error (deg)")
    axes[0].set_title("Stage 1: quaternion attitude hold (tracking the IMU estimate)")
    axes[0].grid(True, alpha=0.3)

    labels = ["roll", "pitch", "yaw"]
    for i, label in enumerate(labels):
        axes[1].plot(t, euler[:, i], label=f"{label} true")
        axes[1].plot(t, euler_est[:, i], ":", label=f"{label} IMU est.")
        axes[1].plot(t, euler_des[:, i], "--", label=f"{label} desired")
    axes[1].set_ylabel("Angle (deg)")
    axes[1].legend(ncol=3, fontsize=7)
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, est_err_deg, color="tab:purple")
    axes[2].set_ylabel("Estimator error (deg)")
    axes[2].set_title("|q_est - q_true| -- Mahony filter accuracy")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t, pos[:, 2], label="altitude")
    axes[3].axhline(1.0, color="gray", linestyle=":", label="start altitude")
    axes[3].set_ylabel("Altitude (m)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(fontsize=8)
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    out_path = os.path.join(OUTPUT_DIR, "stage1_hover_tracking.png")
    fig.savefig(out_path, dpi=150)
    print(f"Saved tracking plot to {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true", help="open the interactive MuJoCo viewer")
    parser.add_argument("--duration", type=float, default=6.0, help="simulation duration (s)")
    args = parser.parse_args()
    run(duration=args.duration, use_viewer=args.viewer)


if __name__ == "__main__":
    main()
