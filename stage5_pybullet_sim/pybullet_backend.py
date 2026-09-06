"""
PyBullet physics/sensing backend for Stage 5.

Stage 4 wires Stage 1 (attitude control), Stage 2 (adaptive B-spline +
position control), and Stage 3 (IR ray-casting + IMU) around MuJoCo. All of
that control/planning code is physics-engine-agnostic -- it only ever reads
plain numpy state (position, velocity, quaternion, angular velocity) and
returns plain numpy commands (thrust, torque). This module supplies the
PyBullet equivalents of the *engine-specific* pieces MuJoCo was providing
(rigid-body stepping, ray-cast sensing, IMU synthesis) so the exact same
Stage 1/2/3 classes can drive a PyBullet simulation instead, unchanged.

Obstacle geometry mirrors ``models/quadrotor.xml`` exactly (box + cylinder,
same positions/sizes) so Stage 5 runs are comparable to Stage 4's.
"""

import numpy as np
import pybullet as p
import pybullet_data

from stage1_dynamics_attitude import config as s1_config
from stage1_dynamics_attitude import quaternion_utils as qu
from stage2_trajectory_planning import config as s2_config
from stage3_sensing_environment import config as s3_config

# --- Obstacle geometry, mirroring models/quadrotor.xml exactly -----------------
OBSTACLES = [
    {"name": "obstacle_1", "shape": "box", "half_extents": [0.15, 0.15, 0.5],
     "position": [1.5, 0.5, 1.0]},
    {"name": "obstacle_2", "shape": "cylinder", "radius": 0.2, "height": 1.5,
     "position": [-1.2, -1.0, 0.75]},
]
OBSTACLE_COLOR = [0.85, 0.25, 0.2, 0.55]
GROUND_COLOR = [0.6, 0.65, 0.7, 1.0]


def quat_xyzw_to_wxyz(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([w, x, y, z], dtype=float)


def quat_wxyz_to_xyzw(q_wxyz) -> list:
    w, x, y, z = q_wxyz
    return [x, y, z, w]


def connect(use_gui: bool) -> int:
    """Start a PyBullet physics client (GUI or headless DIRECT)."""
    client = p.connect(p.GUI if use_gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0.0, 0.0, -s1_config.GRAVITY, physicsClientId=client)
    p.setPhysicsEngineParameter(fixedTimeStep=s1_config.SIM_TIMESTEP, physicsClientId=client)
    p.setRealTimeSimulation(0, physicsClientId=client)
    return client


def build_world(client: int, start_position):
    """
    Load the ground plane, both obstacles (matching models/quadrotor.xml),
    and the drone body. The drone is modeled as a uniform sphere of radius
    ``DRONE_RADIUS`` (stage2_trajectory_planning/config.py) -- the same
    point+radius approximation every safety margin in this project (
    SAFE_DISTANCE, DETECTION_RANGE, TRIGGER_BODY_CLEARANCE) is already
    built around, rather than reproducing the MuJoCo model's exact
    chassis/arm/rotor geoms, which no planning or control code here
    actually depends on.

    Returns: (drone_id, obstacle_ids) -- ``obstacle_ids`` is needed by
    ``create_obstacle_distance_fn`` to tell a real obstacle hit apart from
    a ground-plane hit (PyBullet's ``rayTest`` has no MuJoCo-style
    "OBSTACLE_PREFIX" body-name filter to fall back on; Stage 3's
    equivalent MuJoCo function uses exactly this "obstacle bodies only"
    filtering so a downward probe doesn't see the ground as the nearest
    obstacle).
    """
    p.loadURDF("plane.urdf", physicsClientId=client)

    obstacle_ids = []
    for obs in OBSTACLES:
        if obs["shape"] == "box":
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=obs["half_extents"],
                                          physicsClientId=client)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=obs["half_extents"],
                                       rgbaColor=OBSTACLE_COLOR, physicsClientId=client)
        else:
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=obs["radius"],
                                          height=obs["height"], physicsClientId=client)
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=obs["radius"],
                                       length=obs["height"], rgbaColor=OBSTACLE_COLOR,
                                       physicsClientId=client)
        obstacle_id = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=col,
                                         baseVisualShapeIndex=vis, basePosition=obs["position"],
                                         physicsClientId=client)
        obstacle_ids.append(obstacle_id)

    drone_col = p.createCollisionShape(p.GEOM_SPHERE, radius=s2_config.DRONE_RADIUS,
                                        physicsClientId=client)
    drone_vis = p.createVisualShape(p.GEOM_SPHERE, radius=s2_config.DRONE_RADIUS,
                                     rgbaColor=[0.25, 0.25, 0.28, 1.0], physicsClientId=client)
    drone_id = p.createMultiBody(baseMass=s1_config.MASS, baseCollisionShapeIndex=drone_col,
                                  baseVisualShapeIndex=drone_vis, basePosition=list(start_position),
                                  physicsClientId=client)

    # PyBullet's default per-body linear/angular damping (~0.04) models
    # generic air drag; Stage 1's dynamics (and the MuJoCo model, which
    # declares no <option> drag terms) assume a drag-free rigid body, so
    # the same attitude/position gains apply here without retuning.
    p.changeDynamics(drone_id, -1, linearDamping=0.0, angularDamping=0.0,
                      localInertiaDiagonal=list(s1_config.INERTIA_DIAG),
                      physicsClientId=client)
    return drone_id, obstacle_ids


def reset_drone_state(client: int, drone_id: int, position) -> None:
    """Place the drone at ``position``, level, at rest -- mirrors pipeline.reset_drone_state."""
    p.resetBasePositionAndOrientation(drone_id, list(position), [0.0, 0.0, 0.0, 1.0],
                                       physicsClientId=client)
    p.resetBaseVelocity(drone_id, linearVelocity=[0.0, 0.0, 0.0], angularVelocity=[0.0, 0.0, 0.0],
                         physicsClientId=client)


def get_state(client: int, drone_id: int):
    """Returns (pos, vel, q_wxyz, omega_body) -- the same shapes Stage 1/2 controllers expect."""
    pos, orn_xyzw = p.getBasePositionAndOrientation(drone_id, physicsClientId=client)
    lin_vel_world, ang_vel_world = p.getBaseVelocity(drone_id, physicsClientId=client)
    q_wxyz = quat_xyzw_to_wxyz(orn_xyzw)
    R = qu.quat_to_rotmat(q_wxyz)  # body -> world
    omega_body = R.T @ np.array(ang_vel_world, dtype=float)
    return np.array(pos, dtype=float), np.array(lin_vel_world, dtype=float), q_wxyz, omega_body


def apply_motor_thrusts(client: int, drone_id: int, motor_thrusts: np.ndarray) -> None:
    """
    Apply Stage 1's MotorMixer output as real forces/torques on the PyBullet
    body: each motor pushes along body +Z at its ``MOTOR_POSITIONS`` offset
    (PyBullet derives the resulting roll/pitch torque itself from that
    offset, exactly like a real airframe), plus each spinning prop's
    reaction (drag) torque about body Z -- the same wrench model
    ``MotorMixer`` was built to invert, just applied forward here.
    """
    pos, orn_xyzw = p.getBasePositionAndOrientation(drone_id, physicsClientId=client)
    q_wxyz = quat_xyzw_to_wxyz(orn_xyzw)
    R = qu.quat_to_rotmat(q_wxyz)
    pos = np.array(pos, dtype=float)

    net_torque_world = np.zeros(3)
    for i in range(4):
        thrust = float(motor_thrusts[i])
        force_world = R @ np.array([0.0, 0.0, thrust])
        motor_pos_world = pos + R @ s1_config.MOTOR_POSITIONS[i]
        p.applyExternalForce(drone_id, -1, forceObj=force_world.tolist(),
                              posObj=motor_pos_world.tolist(), flags=p.WORLD_FRAME,
                              physicsClientId=client)
        reaction_torque_body = np.array([0.0, 0.0,
                                          s1_config.MOTOR_SPIN[i] * s1_config.YAW_TORQUE_COEFF * thrust])
        net_torque_world += R @ reaction_torque_body

    p.applyExternalTorque(drone_id, -1, torqueObj=net_torque_world.tolist(), flags=p.WORLD_FRAME,
                           physicsClientId=client)


class PyBulletIMU:
    """
    PyBullet-backed stand-in for Stage 3's MuJoCo-sensor-based ``IMUSensor``.

    MuJoCo exposes a direct accelerometer sensor (specific force); PyBullet
    doesn't, so the accelerometer channel here is synthesized by finite-
    differencing the body's world-frame velocity between calls and
    subtracting gravity -- the same "proper acceleration" quantity a real
    accelerometer measures (a stationary, level body reads ``[0, 0, +g]``,
    matching Stage 3's convention). Noise/bias magnitudes are the same
    ``stage3_sensing_environment.config`` values Stage 4 uses, so Stage 5
    runs are comparable rather than accidentally easier/harder to control.

    Interface intentionally matches ``IMUSensor.read()`` closely enough to
    drop into the same call site, but takes ``dt`` explicitly (MuJoCo's
    sensor doesn't need it; this finite-difference does).
    """

    def __init__(self, client: int, drone_id: int,
                 gyro_noise_std: float = s3_config.IMU_GYRO_NOISE_STD,
                 accel_noise_std: float = s3_config.IMU_ACCEL_NOISE_STD,
                 gyro_bias_walk_std: float = s3_config.IMU_GYRO_BIAS_WALK_STD,
                 enable_noise: bool = s3_config.IMU_ENABLE_NOISE,
                 gravity: float = s1_config.GRAVITY,
                 seed: int = None):
        self.client = client
        self.drone_id = drone_id
        self.gyro_noise_std = gyro_noise_std
        self.accel_noise_std = accel_noise_std
        self.gyro_bias_walk_std = gyro_bias_walk_std
        self.enable_noise = enable_noise
        self.gravity_vec = np.array([0.0, 0.0, -gravity])
        self.rng = np.random.default_rng(seed)
        self.gyro_bias = np.zeros(3)

        _, lin_vel_world, _, _ = get_state(client, drone_id)
        self._prev_vel_world = lin_vel_world

    def read(self, dt: float, add_noise: bool = None):
        pos, lin_vel_world, q_wxyz, omega_body = get_state(self.client, self.drone_id)
        R = qu.quat_to_rotmat(q_wxyz)

        accel_world_true = (lin_vel_world - self._prev_vel_world) / dt - self.gravity_vec
        accel_body_true = R.T @ accel_world_true
        self._prev_vel_world = lin_vel_world

        should_add_noise = self.enable_noise if add_noise is None else add_noise
        if not should_add_noise:
            return accel_body_true, omega_body

        self.gyro_bias += self.rng.normal(0.0, self.gyro_bias_walk_std, size=3)
        accel_meas = accel_body_true + self.rng.normal(0.0, self.accel_noise_std, size=3)
        gyro_meas = omega_body + self.gyro_bias + self.rng.normal(0.0, self.gyro_noise_std, size=3)
        return accel_meas, gyro_meas


def create_obstacle_distance_fn(client: int, obstacle_ids,
                                 max_range: float = s3_config.SENSOR_MAX_RANGE,
                                 min_range: float = s3_config.SENSOR_MIN_RANGE,
                                 noise_sigma: float = s3_config.SENSOR_NOISE_SIGMA,
                                 enable_noise: bool = s3_config.ENABLE_NOISE,
                                 seed: int = None):
    """
    PyBullet ``rayTest``-backed equivalent of Stage 3's
    ``create_obstacle_distance_fn`` (which uses MuJoCo's ``mj_ray``) --
    same signature, same range clipping / Gaussian noise model, so
    ``AdaptiveBSplinePlanner`` / ``ReactiveReplanner`` (Stage 2) work
    against it unmodified.

    ``obstacle_ids`` (from ``build_world``) restricts hits to the actual
    obstacle bodies -- a hit on anything else (the ground plane, the drone
    itself) is treated as "no hit", matching Stage 3's ``OBSTACLE_PREFIX``
    filtering, which exists for exactly the same reason: an unfiltered
    downward ray would otherwise report the ground as the nearest
    "obstacle" at a distance equal to the drone's altitude.
    """
    obstacle_id_set = set(obstacle_ids)
    rng = np.random.default_rng(seed)

    def obstacle_distance_fn(position: np.ndarray, direction: np.ndarray) -> float:
        direction = np.asarray(direction, dtype=float)
        norm = np.linalg.norm(direction)
        if norm < 1e-8:
            return max_range
        direction = direction / norm

        start = np.asarray(position, dtype=float)
        end = start + direction * max_range
        hit_id, _, hit_fraction, _, _ = p.rayTest(start.tolist(), end.tolist(),
                                                   physicsClientId=client)[0]

        if hit_id not in obstacle_id_set:
            raw_dist = max_range
        else:
            raw_dist = hit_fraction * max_range

        clipped = float(np.clip(raw_dist, min_range, max_range))
        if enable_noise and raw_dist < max_range:
            clipped = float(np.clip(clipped + rng.normal(0.0, noise_sigma), min_range, max_range))
        return clipped

    return obstacle_distance_fn
