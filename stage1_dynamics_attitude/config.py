"""
Physical and control parameters for the Stage-1 quadrotor.

This module is the single source of truth for the drone's physical
parameters. The numbers here (mass, arm length, motor layout, yaw-torque
coefficient) MUST match ``models/quadrotor.xml`` — the XML comments repeat
these values so the two stay in sync. If you change one, change the other.

Body frame convention: X-forward, Y-left, Z-up, right-handed (matches the
MuJoCo world frame at zero attitude). Motors are arranged in a "+"
configuration (not "X"): one motor on each of +X, -X, +Y, -Y.
"""

import numpy as np

# --- Mass properties -------------------------------------------------------
MASS = 1.2  # kg
GRAVITY = 9.81  # m/s^2

# Principal moments of inertia about the body-fixed CoM axes (kg*m^2).
# Diagonal because the model is built symmetric about all three planes.
INERTIA_DIAG = np.array([0.02, 0.02, 0.035])  # Ixx, Iyy, Izz

# --- Airframe geometry -------------------------------------------------------
ARM_LENGTH = 0.20  # m, motor distance from CoM along body X or Y axis

# Motor order: front (+X), right (-Y), back (-X), left (+Y).
MOTOR_NAMES = ["front", "right", "back", "left"]

MOTOR_POSITIONS = np.array([
    [ARM_LENGTH, 0.0, 0.0],   # front
    [0.0, -ARM_LENGTH, 0.0],  # right
    [-ARM_LENGTH, 0.0, 0.0],  # back
    [0.0, ARM_LENGTH, 0.0],   # left
])

# Propeller spin direction at each motor: +1 = CCW, -1 = CW (viewed from
# above, looking down -Z). Front/back spin one way, left/right the other,
# so that yaw reaction torques cancel at equal thrust (pure hover -> no net
# yaw torque).
MOTOR_SPIN = np.array([+1.0, -1.0, +1.0, -1.0])

# Reaction (drag) torque per unit thrust produced by each spinning prop,
# about the motor's local Z axis: tau_reaction = spin_dir * YAW_TORQUE_COEFF * thrust.
# Units: meters (an effective moment arm). Must match the XML motor <gear> z-torque term.
YAW_TORQUE_COEFF = 0.02  # m

# --- Actuator limits ---------------------------------------------------------
THRUST_MAX_PER_MOTOR = 8.0  # N
THRUST_MIN_PER_MOTOR = 0.0  # N (propellers can't push in reverse)

HOVER_THRUST_TOTAL = MASS * GRAVITY  # N, total thrust needed to hover
HOVER_THRUST_PER_MOTOR = HOVER_THRUST_TOTAL / 4.0

# --- Attitude controller gains ------------------------------------------------
# Proportional (orientation error) and derivative (angular-rate error) gains,
# applied per-axis (roll, pitch, yaw) since INERTIA_DIAG is diagonal.
ATTITUDE_KP = np.array([6.0, 6.0, 3.0])
ATTITUDE_KD = np.array([1.2, 1.2, 0.8])

# --- Simulation ---------------------------------------------------------------
SIM_TIMESTEP = 0.002  # s, must match <option timestep="..."/> in the XML
MODEL_XML_PATH = "models/quadrotor.xml"
