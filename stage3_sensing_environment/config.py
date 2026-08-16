"""
Configuration parameters for Stage 3 -- Sensing & Environment.

Defines default settings for MuJoCo ray-cast sensors, noise models,
range clipping, and obstacle query settings.
"""

import os

# --- Sensor limits & noise specifications -------------------------------------
SENSOR_MIN_RANGE = 0.05  # m, distances below this are clipped to SENSOR_MIN_RANGE
SENSOR_MAX_RANGE = 10.0  # m, distances above this (or ray misses) are clipped to SENSOR_MAX_RANGE
SENSOR_NOISE_SIGMA = 0.01  # m, standard deviation of zero-mean Gaussian measurement noise
ENABLE_NOISE = True  # whether noise is applied by default in sensor measurements

# --- IMU (gyro + accelerometer) noise specifications ---------------------------
# Roughly MEMS-IMU-grade magnitudes (e.g. MPU6050-class), not a specific datasheet.
IMU_GYRO_NOISE_STD = 0.01  # rad/s, gyro white measurement noise std dev
IMU_ACCEL_NOISE_STD = 0.05  # m/s^2, accelerometer white measurement noise std dev
IMU_GYRO_BIAS_WALK_STD = 0.0002  # rad/s per sample, gyro bias random-walk step std dev
IMU_ENABLE_NOISE = True  # whether noise/bias are applied by default

# --- Model & Scene naming conventions -----------------------------------------
DRONE_BODY_NAME = "drone"  # body name to exclude from ray casting to avoid self-collision
OBSTACLE_PREFIX = "obstacle"  # default prefix for obstacle bodies in MuJoCo worldbody

# Path to the default XML model relative to repository root
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_MODEL_XML = os.path.join(REPO_ROOT, "models", "quadrotor.xml")
