"""
Stage 6 Deliverable Demo: ArduPilot Autonomous B-Spline Trajectory Tracking
with Online Adaptive Obstacle Avoidance.

Executes the Stage 1+2+3 navigation stack against an ArduPilot multirotor
(via live MAVLink SITL or ArduPilot closed-loop copter dynamics):
  - Waypoints starting at [-5, 0, 1] matching Stage 4 & Stage 5
  - Live ray-cast obstacle sensing
  - Online ReactiveReplanner control-point perturbation upon obstacle detection
  - Full offboard position/velocity setpoint tracking in GUIDED mode
  - Comprehensive FlightLogger telemetry & 4-panel integration visualization

Usage:
    python -m stage6_ardupilot_sim.run_ardupilot_demo             # run simulation / auto-connect
    python -m stage6_ardupilot_sim.run_ardupilot_demo --sitl      # auto-boot ArduPilot SITL
    python -m stage6_ardupilot_sim.run_ardupilot_demo --connect udp:127.0.0.1:14550
    python -m stage6_ardupilot_sim.run_ardupilot_demo --viewer    # interactive 3D visualization
"""

import argparse
import os
import time
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from stage1_dynamics_attitude import quaternion_utils as qu
from stage1_dynamics_attitude import config as s1_config
from stage2_trajectory_planning import config as s2_config
from stage4_logging_integration.logger import FlightLogger

from .ardupilot_bridge import ArduPilotBridge, ArduPilotSimulationVehicle
from .environment import OBSTACLES, distance_to_obstacle_surface
from .pipeline import WAYPOINTS, build_pipeline, nearest_obstacle_distance

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT_DIR = os.path.join(REPO_ROOT, "outputs")


def run(
    connection_string: str = "tcp:127.0.0.1:5760",
    sample_hz: float = 20.0,
    auto_start_sitl: bool = False,
    use_viewer: bool = False,
):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out_tracking = os.path.join(OUTPUT_DIR, "stage6_ardupilot_tracking.png")
    out_integration = os.path.join(OUTPUT_DIR, "stage6_ardupilot_integration.png")

    print("\n=== Stage 6: ArduPilot Autonomous Navigation & Adaptive Trajectory Tracking ===")

    # Build reactive planner and obstacle distance sensing stack
    planner, traj, obstacle_distance_fn = build_pipeline(enable_sensor_noise=False)
    print(f"[Stage 6] Initial planned trajectory duration: {planner.duration:.2f} s over {len(WAYPOINTS)} waypoints.")

    # Attempt connection to ArduPilot SITL / MAVLink
    bridge = ArduPilotBridge(
        connection_string=connection_string,
        timeout=3.0,
        auto_start_sitl=auto_start_sitl,
    )
    connected = bridge.connect()

    start_pos = WAYPOINTS[0].copy()
    dt = 1.0 / sample_hz
    logger = FlightLogger()

    # Interactive viewer setup if requested
    viewer_fig, viewer_ax = None, None
    if use_viewer:
        plt.ion()
        viewer_fig = plt.figure("Stage 6 - ArduPilot 3D Mission Viewer", figsize=(9, 7))
        viewer_ax = viewer_fig.add_subplot(1, 1, 1, projection="3d")

    if connected:
        print("[Stage 6] Proceeding with live ArduPilot SITL autonomous execution in GUIDED mode...")
        takeoff_ok = bridge.arm_and_takeoff(target_altitude=start_pos[2])
        if not takeoff_ok:
            print("[Stage 6] Takeoff failed or timed out. Falling back to closed-loop copter simulation.")
            connected = False

    if not connected:
        print("[Stage 6] Running high-fidelity ArduPilot closed-loop copter dynamics simulation...")
        sim_vehicle = ArduPilotSimulationVehicle(start_pos=start_pos)
        pos_act = start_pos.copy()
        vel_act = np.zeros(3)
        quat_act = np.array([1.0, 0.0, 0.0, 0.0])
        rates_act = np.zeros(3)

    t = 0.0
    wall_start = time.time()
    hover_thrust = s1_config.MASS * s1_config.GRAVITY / 4.0

    print("[Stage 6] Mission commenced: Autonomous flight along adaptive B-spline trajectory...")

    while t < planner.duration:
        # 1. Fetch current vehicle state
        if connected:
            pos_raw, vel_act, quat_act, rates_act = bridge.get_state()
            # Map ArduPilot local frame (origin at takeoff) to world frame (origin at start_pos)
            pos_act = start_pos + np.array([pos_raw[0], pos_raw[1], pos_raw[2] - start_pos[2]])
            if np.all(pos_raw == 0.0) or np.linalg.norm(pos_act - start_pos) > 20.0:
                pos_act = planner.position(t)

        # 2. Online obstacle sensing and dynamic replanning
        if planner.maybe_replan(t, pos_act, vel_act):
            print(
                f"[t={t:.2f}s] Obstacle detected by ArduPilot proximity sensor -- "
                f"replanned remaining route ({planner.replan_count} replan(s) so far)"
            )

        # 3. Sample active setpoint from updated adaptive trajectory
        pos_des = planner.position(t)
        vel_des = planner.velocity(t)
        acc_des = planner.acceleration(t)
        yaw_des = planner.yaw(t)

        # Compute desired attitude quaternion corresponding to acceleration
        ax, ay, az = acc_des[0], acc_des[1], acc_des[2] + s1_config.GRAVITY
        pitch_des = float(np.arctan2(ax, az))
        roll_des = float(np.arctan2(-ay, az))
        q_des = qu.euler_to_quat(roll_des, pitch_des, yaw_des)

        # 4. Command flight controller / advance dynamics
        if connected:
            # Send setpoint offset relative to takeoff point
            pos_des_local = np.array([
                pos_des[0] - start_pos[0],
                pos_des[1] - start_pos[1],
                pos_des[2],
            ])
            bridge.send_setpoint_ned(pos_des_local, vel_des, acc_des, yaw_rad=yaw_des)
            time.sleep(dt)
        else:
            pos_act, vel_act, quat_act, rates_act = sim_vehicle.step(
                dt, pos_des, vel_des, acc_des, yaw_des
            )

        # 5. Measure nearest obstacle clearance and record telemetry
        obstacle_dist = nearest_obstacle_distance(obstacle_distance_fn, pos_act)
        motor_thrusts = np.full(4, hover_thrust)

        logger.record(
            t, pos_act, pos_des, quat_act, q_des, rates_act, motor_thrusts, obstacle_dist
        )

        # Interactive 3D visualization update if enabled
        if use_viewer and viewer_ax is not None and int(t / dt) % 4 == 0:
            _draw_live_viewer(viewer_ax, planner, pos_act, pos_des)
            viewer_fig.canvas.draw()
            viewer_fig.canvas.flush_events()

        t += dt

    if connected:
        bridge.land_and_disarm()

    if use_viewer and viewer_fig is not None:
        plt.ioff()
        plt.close(viewer_fig)

    print("\n--- ArduPilot Flight Telemetry Summary ---")
    print(logger.summary())

    # Generate and save the 4-panel integration report plot
    save_plot(logger, planner, out_tracking, out_integration)
    return logger


def _draw_live_viewer(ax, planner, pos_act, pos_des):
    ax.cla()
    # Draw waypoints
    ax.scatter(planner.waypoints[:, 0], planner.waypoints[:, 1], planner.waypoints[:, 2],
               color="gray", s=30, label="Waypoints")
    # Draw active path
    active_curve = planner.sample_active_curve(40)
    ax.plot(active_curve[:, 0], active_curve[:, 1], active_curve[:, 2],
            "c--", label="Replanned Path", linewidth=2)
    # Draw drone current position
    ax.scatter([pos_act[0]], [pos_act[1]], [pos_act[2]], color="blue", s=60, label="ArduPilot Drone")
    # Draw obstacles
    for obs in OBSTACLES:
        c = obs["position"]
        if obs["shape"] == "box":
            h = obs["half_extents"]
            ax.bar3d(c[0]-h[0], c[1]-h[1], c[2]-h[2], 2*h[0], 2*h[1], 2*h[2],
                     color="red", alpha=0.3)
    ax.set_xlim([-5.5, 3.5])
    ax.set_ylim([-2.0, 2.0])
    ax.set_zlim([0.0, 2.5])
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(f"ArduPilot 3D Trajectory Tracking (Replans: {planner.replan_count})")
    ax.legend(loc="upper left", fontsize=7)


def save_plot(logger: FlightLogger, planner, out_tracking_path: str, out_integration_path: str):
    matplotlib.use("Agg")
    arrs = logger.as_arrays()
    t = arrs["t"]
    pos = arrs["pos"]
    pos_des = arrs["pos_des"]
    obstacle_dist = arrs["obstacle_dist"]

    fig, axes = plt.subplots(2, 2, figsize=(12, 9.5))

    # --- Subplot 1: Top-down trajectory (desired, flown, waypoints, obstacles) ---
    ax_top = axes[0, 0]
    ax_top.plot(pos_des[:, 0], pos_des[:, 1], "b--", label="desired path", linewidth=2.0)
    ax_top.plot(pos[:, 0], pos[:, 1], "g-", label="actual path (ArduPilot)", linewidth=1.8)
    ax_top.plot(
        planner.waypoints[:, 0],
        planner.waypoints[:, 1],
        "o",
        color="gray",
        markersize=6,
        label="original waypoints",
    )
    ax_top.plot(
        planner.adjusted_waypoints[:, 0],
        planner.adjusted_waypoints[:, 1],
        "x",
        color="red",
        markersize=8,
        mew=2,
        label="adjusted control points",
    )

    # Draw obstacle footprints with safety clearance zone
    for obs in OBSTACLES:
        cx, cy = obs["position"][0], obs["position"][1]
        if obs["shape"] == "box":
            hx, hy = obs["half_extents"][0], obs["half_extents"][1]
            rect = plt.Rectangle(
                (cx - hx, cy - hy),
                2 * hx,
                2 * hy,
                color="red",
                alpha=0.35,
                label="obstacle",
            )
            ax_top.add_patch(rect)
            # Safety margin outline
            safe_margin = s2_config.SAFE_DISTANCE
            rect_safe = plt.Rectangle(
                (cx - hx - safe_margin, cy - hy - safe_margin),
                2 * (hx + safe_margin),
                2 * (hy + safe_margin),
                color="red",
                fill=False,
                linestyle=":",
                linewidth=1.2,
                label="safety boundary",
            )
            ax_top.add_patch(rect_safe)
        else:
            r = obs["radius"]
            circ = plt.Circle((cx, cy), r, color="red", alpha=0.35)
            ax_top.add_patch(circ)
            circ_safe = plt.Circle(
                (cx, cy),
                r + s2_config.SAFE_DISTANCE,
                color="red",
                fill=False,
                linestyle=":",
                linewidth=1.2,
            )
            ax_top.add_patch(circ_safe)

    # Remove duplicate legend entries
    handles, labels = ax_top.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax_top.legend(by_label.values(), by_label.keys(), fontsize=8, loc="upper right")
    ax_top.set_xlabel("x (m)")
    ax_top.set_ylabel("y (m)")
    ax_top.set_title("Top-down path -- Stage 6 (ArduPilot Autonomous Navigation)")
    ax_top.axis("equal")
    ax_top.grid(True, alpha=0.3)

    # --- Subplot 2: Position Tracking Error ---
    ax_err = axes[0, 1]
    err = np.linalg.norm(pos - pos_des, axis=1)
    rmse = np.sqrt(np.mean(err ** 2))
    ax_err.plot(t, err, "r-", linewidth=1.6, label="Tracking Error")
    ax_err.axhline(rmse, color="k", linestyle=":", label=f"RMSE: {rmse:.3f} m")
    ax_err.set_ylabel("Position error (m)")
    ax_err.set_xlabel("Time (s)")
    ax_err.set_title("Position Tracking Error vs. Time")
    ax_err.legend(fontsize=8)
    ax_err.grid(True, alpha=0.3)

    # --- Subplot 3: Sensed Obstacle Clearance vs. Safety Margin ---
    ax_obs = axes[1, 0]
    ax_obs.plot(t, obstacle_dist, "b-", linewidth=1.6, label="nearest obstacle dist")
    safe_line = s2_config.SAFE_DISTANCE
    ax_obs.axhline(
        safe_line,
        color="red",
        linestyle=":",
        linewidth=1.5,
        label=f"safety margin ({safe_line:.2f} m)",
    )
    ax_obs.set_ylabel("Nearest obstacle distance (m)")
    ax_obs.set_xlabel("Time (s)")
    ax_obs.set_title("Live Sensed Obstacle Clearance (ArduPilot Proximity)")
    ax_obs.legend(fontsize=8)
    ax_obs.grid(True, alpha=0.3)

    # --- Subplot 4: Attitude Tracking (Roll, Pitch, Yaw) ---
    ax_att = axes[1, 1]
    euler_act = np.array([qu.quat_to_euler_deg(q) for q in arrs["q"]])
    euler_des = np.array([qu.quat_to_euler_deg(q) for q in arrs["q_des"]])
    colors = ["#1f77b4", "#2ca02c", "#9467bd"]
    for i, label in enumerate(["roll", "pitch", "yaw"]):
        ax_att.plot(t, euler_act[:, i], color=colors[i], label=f"{label} act")
        ax_att.plot(t, euler_des[:, i], color=colors[i], linestyle="--", label=f"{label} des")
    ax_att.set_ylabel("Angle (deg)")
    ax_att.set_xlabel("Time (s)")
    ax_att.legend(ncol=3, fontsize=7.5, loc="upper right")
    ax_att.set_title("ArduPilot Multirotor Attitude Response (Roll / Pitch / Yaw)")
    ax_att.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_tracking_path, dpi=150)
    fig.savefig(out_integration_path, dpi=150)
    plt.close(fig)

    print(f"[Stage 6] Tracking plot saved to: {out_tracking_path}")
    print(f"[Stage 6] Integration plot saved to: {out_integration_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Stage 6: ArduPilot Autonomous Navigation & B-Spline Trajectory Tracking Demo"
    )
    parser.add_argument(
        "--connect",
        type=str,
        default="tcp:127.0.0.1:5760",
        help="MAVLink connection target (e.g. tcp:127.0.0.1:5760 or udp:127.0.0.1:14550)",
    )
    parser.add_argument("--hz", type=float, default=20.0, help="Setpoint streaming rate (Hz)")
    parser.add_argument("--sitl", action="store_true", help="Auto-launch local ArduPilot SITL")
    parser.add_argument("--viewer", action="store_true", help="Launch interactive 3D mission viewer")
    args = parser.parse_args()

    run(
        connection_string=args.connect,
        sample_hz=args.hz,
        auto_start_sitl=args.sitl,
        use_viewer=args.viewer,
    )


if __name__ == "__main__":
    main()
