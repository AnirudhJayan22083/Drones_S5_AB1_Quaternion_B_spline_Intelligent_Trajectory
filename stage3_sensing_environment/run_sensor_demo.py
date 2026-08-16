"""
Stage 3 deliverable demo: IR distance sensing, MuJoCo ray-casting (mj_ray),
noise & clipping validation, and integration with Stage 2 adaptive planning.

Usage:
    py -m stage3_sensing_environment.run_sensor_demo
"""

import os
import matplotlib
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from stage2_trajectory_planning.adaptive_planner import AdaptiveBSplinePlanner
from stage2_trajectory_planning import config as s2_config

from . import config
from .environment import EnvironmentManager
from .ir_sensor import IRSensor, create_obstacle_distance_fn

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT_DIR = os.path.join(REPO_ROOT, "outputs")


def run_demo():
    print("=" * 60)
    print("Stage 3 — Sensing & Environment Demo")
    print("=" * 60)

    # 1. Load model and environment manager
    xml_path = config.DEFAULT_MODEL_XML
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    env_mgr = EnvironmentManager(model, data)
    obstacles = env_mgr.get_obstacles()
    print(f"\n[Environment] Discovered {len(obstacles)} obstacle(s) in model:")
    for obs in obstacles:
        print(f"  - {obs['name']} (ID {obs['body_id']}) at pos {obs['pos']} with radius ~{obs['bounding_radius']:.2f}m")

    # 2. Instantiate IRSensor with noise & clipping
    sensor = IRSensor(
        model=model,
        data=data,
        min_range=config.SENSOR_MIN_RANGE,
        max_range=config.SENSOR_MAX_RANGE,
        noise_sigma=config.SENSOR_NOISE_SIGMA,
        enable_noise=True,
        seed=42,
    )

    # 3. Ray casting verification: Single ray tests
    print("\n[Ray Casting] Testing specific ray queries:")

    # Ray from [0, 0, 1] towards obstacle_1 [1.5, 0.5, 1.0]
    origin = np.array([0.0, 0.0, 1.0])
    dir_obs1 = np.array([1.5, 0.5, 0.0])
    dir_obs1 = dir_obs1 / np.linalg.norm(dir_obs1)

    clean_dist, hit_id = sensor.measure_ray(origin, dir_obs1, add_noise=False)
    noisy_dist, _ = sensor.measure_ray(origin, dir_obs1, add_noise=True)
    hit_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, hit_id) if hit_id >= 0 else "None"
    print(f"  Ray -> obstacle_1: clean={clean_dist:.3f}m, noisy={noisy_dist:.3f}m, hit_geom='{hit_name}'")

    # Ray straight up (open sky / no obstacle)
    dir_sky = np.array([0.0, 0.0, 1.0])
    sky_dist, hit_sky = sensor.measure_ray(origin, dir_sky)
    print(f"  Ray -> open sky: measured={sky_dist:.3f}m (max_range={config.SENSOR_MAX_RANGE}m), hit_geom_id={hit_sky}")

    # 4. Multi-direction horizontal 360-degree sensor scan
    print("\n[Sensor Scan] Performing 360° horizontal ray scan from [1.0, 0.5, 1.0]...")
    scan_pos = np.array([1.0, 0.5, 1.0])
    n_rays = 72
    angles = np.linspace(0, 2 * np.pi, n_rays, endpoint=False)
    scan_dirs = np.column_stack([np.cos(angles), np.sin(angles), np.zeros(n_rays)])

    clean_scan = sensor.measure_rays(scan_pos, scan_dirs, add_noise=False)
    noisy_scan = sensor.measure_rays(scan_pos, scan_dirs, add_noise=True)

    min_scan_idx = np.argmin(clean_scan)
    print(f"  Nearest obstacle detected at angle {np.degrees(angles[min_scan_idx]):.1f}° at dist={clean_scan[min_scan_idx]:.3f}m")

    # 5. Integration with Stage 2 AdaptiveBSplinePlanner
    print("\n[Integration] Wiring real Stage 3 mj_ray distance function into Stage 2 AdaptiveBSplinePlanner...")
    obstacle_fn = create_obstacle_distance_fn(
        model, data, enable_noise=False, max_range=config.SENSOR_MAX_RANGE
    )

    waypoints = np.array([
        [0.0, 0.0, 1.0],
        [0.8, 0.3, 1.2],
        [1.5, 0.5, 1.4],
        [2.2, 0.2, 1.2],
        [3.0, 0.0, 1.0],
    ])

    planner = AdaptiveBSplinePlanner(
        waypoints,
        obstacle_distance_fn=obstacle_fn,
        safe_distance=s2_config.SAFE_DISTANCE,
        gain=s2_config.PERTURB_GAIN,
        max_shift=s2_config.MAX_PERTURB_SHIFT,
        max_iters=s2_config.MAX_PLANNER_ITERS,
    )

    traj = planner.plan()
    print("  Original interior waypoints:", waypoints[1:-1])
    print("  Adjusted interior control points:", planner.adjusted_waypoints[1:-1])
    print("  Planner successfully generated B-spline trajectory using real Stage 3 sensor queries!")

    # 6. Save visualization plot
    save_demo_plot(angles, clean_scan, noisy_scan, planner, traj, obstacles)
    print("=" * 60)


def save_demo_plot(angles, clean_scan, noisy_scan, planner, traj, obstacles):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    fig = plt.figure(figsize=(14, 5))

    # Subplot 1: Polar sensor scan plot
    ax1 = fig.add_subplot(121, projection='polar')
    ax1.plot(angles, clean_scan, label="Clean mj_ray distance", color="blue", linewidth=2)
    ax1.scatter(angles, noisy_scan, label="Noisy IR sensor reading", color="red", s=15, alpha=0.7)
    ax1.set_title("360° Horizontal IR Sensor Scan from (1.0, 0.5, 1.0)", va='bottom')
    ax1.set_rmax(config.SENSOR_MAX_RANGE)
    ax1.legend(loc="upper right", fontsize=8)

    # Subplot 2: Planner trajectory push visualization using Stage 3 sensor
    ax2 = fig.add_subplot(122)
    t_eval = np.linspace(0, traj.duration, 200)
    pos_eval = np.array([traj.position(t) for t in t_eval])

    ax2.plot(pos_eval[:, 0], pos_eval[:, 1], "b-", label="Planned B-spline path", linewidth=2)
    ax2.plot(planner.waypoints[:, 0], planner.waypoints[:, 1], "ko--", label="Original waypoints")
    ax2.plot(planner.adjusted_waypoints[:, 0], planner.adjusted_waypoints[:, 1], "rx", label="Perturbed control points", markersize=8, markeredgewidth=2)

    # Draw obstacles
    for obs in obstacles:
        pos = obs["pos"]
        r = obs["bounding_radius"]
        circle = plt.Circle((pos[0], pos[1]), r, color="red", alpha=0.3, label=f"Obstacle ({obs['name']})")
        ax2.add_patch(circle)

    ax2.set_xlabel("X position (m)")
    ax2.set_ylabel("Y position (m)")
    ax2.set_title("Stage 2 Planner using Stage 3 Real mj_ray Sensor")
    ax2.axis("equal")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8, loc="best")

    fig.tight_layout()
    out_path = os.path.join(OUTPUT_DIR, "stage3_sensor_demo.png")
    fig.savefig(out_path, dpi=150)
    print(f"\nSaved Stage 3 sensor demo plot to {out_path}")


if __name__ == "__main__":
    run_demo()
