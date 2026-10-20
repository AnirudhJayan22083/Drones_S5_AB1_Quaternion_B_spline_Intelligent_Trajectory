"""
Stage 7 pipeline construction: identical control/planning stack to Stage 4
and Stage 5, run against Gazebo. Reuses the same WAYPOINTS route so all
three physics backends are directly comparable.

Obstacle sensing uses Stage 6's exact analytical ray-box / ray-cylinder
caster over the same OBSTACLES list the Gazebo world is built from
(``gazebo_backend.build_world_sdf``), so the sensed geometry and the
simulated geometry can't drift apart. gz-sim exposes no Python ray-query
API -- the Gazebo-native alternative is a ``gpu_lidar`` sensor, which needs
a GPU rendering engine and would make headless runs depend on one.
"""

from stage1_dynamics_attitude.attitude_controller import AttitudeController
from stage1_dynamics_attitude.mixer import MotorMixer
from stage1_dynamics_attitude import config as s1_config

from stage2_trajectory_planning import config as s2_config
from stage2_trajectory_planning.adaptive_planner import ReactiveReplanner
from stage2_trajectory_planning.position_controller import PositionController

from stage4_logging_integration.pipeline import WAYPOINTS
from stage6_ardupilot_sim.environment import OBSTACLES, create_obstacle_distance_fn


def build_pipeline():
    """
    Build the Stage 1+2+3 control/planning stack for a Gazebo run. Mirrors
    ``stage5_pybullet_sim.pipeline.build_pipeline``, plus the mixer (Stage 5
    builds it in its demo script instead).

    Returns: (planner, traj, pos_ctrl, attitude_ctrl, mixer, obstacle_distance_fn)
    """
    obstacle_distance_fn = create_obstacle_distance_fn(obstacles=OBSTACLES, enable_noise=False)

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
        replan_speed_factor=s2_config.REPLAN_SPEED_FACTOR,
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
