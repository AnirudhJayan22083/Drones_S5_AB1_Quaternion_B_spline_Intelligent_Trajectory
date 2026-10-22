"""
Gazebo (gz-sim, Harmonic or newer) physics/sensing backend for Stage 7.

Same idea as Stage 5's ``pybullet_backend.py``: the Stage 1/2/3 control and
planning code only ever reads plain numpy state and returns plain numpy
commands, so this module supplies just the Gazebo-specific pieces --
world construction, rigid-body state readout, wrench application and IMU
synthesis -- and the exact same controller classes drive a Gazebo
simulation instead.

Gazebo is driven *in-process* through gz-sim's Python ``TestFixture``: the
controller runs inside the server's PreUpdate callback, so every physics
step sees exactly one fresh control command (true lockstep, no MAVLink /
transport round-trip, no race between "command sent" and "step taken").
The running server still publishes the scene over gz-transport, so the
standard Gazebo GUI (``gz sim -g``) can attach to it for live viewing.

Requires the gz-sim + gz-math Python bindings, which are NOT on pip -- they
come with a Gazebo install (Ubuntu / WSL2: ``python3-gz-sim8`` and
``python3-gz-math7`` from packages.osrfoundation.org; see README.md).
"""

import numpy as np

from stage1_dynamics_attitude import config as s1_config
from stage1_dynamics_attitude import quaternion_utils as qu
from stage2_trajectory_planning import config as s2_config
from stage3_sensing_environment import config as s3_config
from stage6_ardupilot_sim.environment import OBSTACLES

# gz-sim / gz-math Python modules are versioned by major release:
_GZ_VERSIONS = [
    ("gz.sim", "gz.math"),
    ("gz.sim8", "gz.math7"),
    ("gz.sim9", "gz.math8"),
    ("gz.sim10", "gz.math9"),
]


def _import_gz():
    import importlib

    for sim_name, math_name in _GZ_VERSIONS:
        try:
            return importlib.import_module(sim_name), importlib.import_module(math_name)
        except ImportError:
            continue
    raise ImportError(
        "Gazebo Python bindings not found (tried gz.sim8/9/10). They ship with a "
        "Gazebo install, not pip -- on Ubuntu/WSL2: "
        "`sudo apt install gz-harmonic python3-gz-sim8 python3-gz-math7`. See README.md."
    )


gz_sim, gz_math = _import_gz()

WORLD_NAME = "quadrotor_world"
DRONE_MODEL_NAME = "quadrotor"
DRONE_LINK_NAME = "base_link"


def _rgba(color) -> str:
    return " ".join(f"{c:g}" for c in color)


def _obstacle_sdf(obs: dict) -> str:
    """Static obstacle model, geometry taken from Stage 6's OBSTACLES (== models/quadrotor.xml)."""
    if obs["shape"] == "box":
        size = " ".join(f"{2.0 * h:g}" for h in obs["half_extents"])
        geometry = f"<box><size>{size}</size></box>"
    else:
        geometry = (f"<cylinder><radius>{obs['radius']:g}</radius>"
                    f"<length>{obs['height']:g}</length></cylinder>")
    x, y, z = obs["position"]
    r, g, b, a = obs["color"]
    return f"""
    <model name="{obs['name']}">
      <static>true</static>
      <pose>{x:g} {y:g} {z:g} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry>{geometry}</geometry></collision>
        <visual name="visual">
          <geometry>{geometry}</geometry>
          <transparency>{1.0 - a:g}</transparency>
          <material>
            <ambient>{_rgba([r, g, b, 1.0])}</ambient>
            <diffuse>{_rgba([r, g, b, 1.0])}</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def _drone_sdf(start_position) -> str:
    """
    The quadrotor: one rigid link carrying the full mass/inertia from
    ``stage1_dynamics_attitude/config.py`` (same as models/quadrotor.xml's
    <inertial>), a sphere collision of radius ``DRONE_RADIUS`` (the same
    point+radius approximation Stage 5 and every planner safety margin use),
    and chassis/arm/rotor visuals matching the MuJoCo model's layout.
    """
    ixx, iyy, izz = s1_config.INERTIA_DIAG
    x, y, z = start_position
    arm_len = s1_config.ARM_LENGTH

    visuals = ["""
        <visual name="chassis">
          <geometry><box><size>0.16 0.16 0.06</size></box></geometry>
          <material><ambient>0.25 0.25 0.28 1</ambient><diffuse>0.25 0.25 0.28 1</diffuse></material>
        </visual>"""]
    for name, motor_pos in zip(s1_config.MOTOR_NAMES, s1_config.MOTOR_POSITIONS):
        mx, my, _ = motor_pos
        # Cylinders are Z-aligned in SDF; tip each arm over onto its motor's axis.
        arm_rot = "0 1.5708 0" if abs(mx) > 0 else "1.5708 0 0"
        visuals.append(f"""
        <visual name="arm_{name}">
          <pose>{mx / 2:g} {my / 2:g} 0 {arm_rot}</pose>
          <geometry><cylinder><radius>0.012</radius><length>{arm_len:g}</length></cylinder></geometry>
          <material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material>
        </visual>
        <visual name="rotor_{name}">
          <pose>{mx:g} {my:g} 0.006 0 0 0</pose>
          <geometry><cylinder><radius>0.06</radius><length>0.012</length></cylinder></geometry>
          <transparency>0.15</transparency>
          <material><ambient>0.2 0.5 0.9 1</ambient><diffuse>0.2 0.5 0.9 1</diffuse></material>
        </visual>""")

    return f"""
    <model name="{DRONE_MODEL_NAME}">
      <pose>{x:g} {y:g} {z:g} 0 0 0</pose>
      <link name="{DRONE_LINK_NAME}">
        <inertial>
          <mass>{s1_config.MASS:g}</mass>
          <inertia>
            <ixx>{ixx:g}</ixx><iyy>{iyy:g}</iyy><izz>{izz:g}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><sphere><radius>{s2_config.DRONE_RADIUS:g}</radius></sphere></geometry>
        </collision>{''.join(visuals)}
      </link>
    </model>"""


def build_world_sdf(start_position, real_time: bool) -> str:
    """
    Build the Stage 7 world as an SDF string: ground plane, both obstacles
    and the quadrotor, with a fixed physics step of ``SIM_TIMESTEP`` (same
    as MuJoCo/PyBullet) and gravity from Stage 1's config.

    ``real_time=False`` sets real_time_factor 0 (run as fast as possible,
    for headless batch runs); ``True`` paces the sim at wall-clock speed so
    the attached Gazebo GUI shows the flight at normal speed.
    """
    rtf = 1.0 if real_time else 0.0
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <physics name="fixed_step" type="ignored">
      <max_step_size>{s1_config.SIM_TIMESTEP:g}</max_step_size>
      <real_time_factor>{rtf:g}</real_time_factor>
    </physics>
    <gravity>0 0 {-s1_config.GRAVITY:g}</gravity>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.65 0.75 0.9 1</background>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.3 0.35 0.4 1</ambient><diffuse>0.3 0.35 0.4 1</diffuse></material>
        </visual>
      </link>
    </model>
{''.join(_obstacle_sdf(obs) for obs in OBSTACLES)}
{_drone_sdf(start_position)}
  </world>
</sdf>
"""


class GazeboDrone:
    """
    Handle on the quadrotor link inside a running gz-sim server. Only valid
    inside a TestFixture callback (it needs that callback's ``ecm``); call
    ``bind(ecm)`` once from the first callback before anything else.
    """

    def __init__(self):
        self.link = None

    def bind(self, ecm) -> None:
        world = gz_sim.World(gz_sim.world_entity(ecm))
        model = gz_sim.Model(world.model_by_name(ecm, DRONE_MODEL_NAME))
        self.link = gz_sim.Link(model.link_by_name(ecm, DRONE_LINK_NAME))
        # Gazebo only populates world-frame velocity components for links
        # that ask for them; they become readable from the next step on.
        self.link.enable_velocity_checks(ecm, True)

    def get_state(self, ecm):
        """Returns (pos, vel, q_wxyz, omega_body) -- the same shapes Stage 1/2 controllers expect."""
        pose = self.link.world_pose(ecm)
        p_vec, rot = pose.pos(), pose.rot()
        pos = np.array([p_vec.x(), p_vec.y(), p_vec.z()], dtype=float)
        q_wxyz = qu.quat_normalize(np.array([rot.w(), rot.x(), rot.y(), rot.z()], dtype=float))

        lin = self.link.world_linear_velocity(ecm)
        ang = self.link.world_angular_velocity(ecm)
        vel = np.array([lin.x(), lin.y(), lin.z()], dtype=float) if lin is not None else np.zeros(3)
        omega_world = np.array([ang.x(), ang.y(), ang.z()], dtype=float) if ang is not None else np.zeros(3)

        R = qu.quat_to_rotmat(q_wxyz)  # body -> world
        return pos, vel, q_wxyz, R.T @ omega_world

    def apply_motor_thrusts(self, ecm, q_wxyz: np.ndarray, motor_thrusts: np.ndarray) -> None:
        """
        Apply Stage 1's MotorMixer output as one net wrench at the link's
        origin (== its CoM): each motor's thrust along body +Z, the roll/pitch
        torque from its ``MOTOR_POSITIONS`` lever arm, and its prop reaction
        torque about body Z -- the same wrench model PyBullet's
        ``apply_motor_thrusts`` applies per-motor, summed here because
        ``Link.add_world_wrench`` takes a single force/torque pair.
        Gazebo clears external wrench commands every step, so this must be
        called once per PreUpdate.
        """
        R = qu.quat_to_rotmat(q_wxyz)
        force_body = np.zeros(3)
        torque_body = np.zeros(3)
        for i in range(4):
            thrust = float(motor_thrusts[i])
            motor_force = np.array([0.0, 0.0, thrust])
            force_body += motor_force
            torque_body += np.cross(s1_config.MOTOR_POSITIONS[i], motor_force)
            torque_body[2] += s1_config.MOTOR_SPIN[i] * s1_config.YAW_TORQUE_COEFF * thrust

        force_world = R @ force_body
        torque_world = R @ torque_body
        self.link.add_world_wrench(ecm, gz_math.Vector3d(*force_world), gz_math.Vector3d(*torque_world))


class GazeboIMU:
    """
    Gazebo-backed stand-in for Stage 3's ``IMUSensor``, same approach as
    Stage 5's ``PyBulletIMU``: accelerometer synthesized by finite-
    differencing world-frame velocity and subtracting gravity (a level,
    stationary body reads ``[0, 0, +g]``), gyro from the link's angular
    velocity, with Stage 3's noise/bias-walk magnitudes.

    Takes the state explicitly rather than querying Gazebo itself, since
    link state is only readable inside a server callback.
    """

    def __init__(self, gyro_noise_std: float = s3_config.IMU_GYRO_NOISE_STD,
                 accel_noise_std: float = s3_config.IMU_ACCEL_NOISE_STD,
                 gyro_bias_walk_std: float = s3_config.IMU_GYRO_BIAS_WALK_STD,
                 enable_noise: bool = s3_config.IMU_ENABLE_NOISE,
                 gravity: float = s1_config.GRAVITY,
                 seed: int = None):
        self.gyro_noise_std = gyro_noise_std
        self.accel_noise_std = accel_noise_std
        self.gyro_bias_walk_std = gyro_bias_walk_std
        self.enable_noise = enable_noise
        self.gravity_vec = np.array([0.0, 0.0, -gravity])
        self.rng = np.random.default_rng(seed)
        self.gyro_bias = np.zeros(3)
        self._prev_vel_world = None

    def read(self, vel_world: np.ndarray, q_wxyz: np.ndarray, omega_body: np.ndarray,
             dt: float, add_noise: bool = None):
        if self._prev_vel_world is None:
            self._prev_vel_world = vel_world.copy()
        R = qu.quat_to_rotmat(q_wxyz)

        accel_world_true = (vel_world - self._prev_vel_world) / dt - self.gravity_vec
        accel_body_true = R.T @ accel_world_true
        self._prev_vel_world = vel_world.copy()

        should_add_noise = self.enable_noise if add_noise is None else add_noise
        if not should_add_noise:
            return accel_body_true, omega_body

        self.gyro_bias += self.rng.normal(0.0, self.gyro_bias_walk_std, size=3)
        accel_meas = accel_body_true + self.rng.normal(0.0, self.accel_noise_std, size=3)
        gyro_meas = omega_body + self.gyro_bias + self.rng.normal(0.0, self.gyro_noise_std, size=3)
        return accel_meas, gyro_meas
