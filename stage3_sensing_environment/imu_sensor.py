"""
IMU sensor stub built on MuJoCo's native ``gyro`` and ``accelerometer``
sensors (see ``imu_site`` in models/quadrotor.xml).

Simulates a 6-axis IMU (3-axis gyroscope + 3-axis accelerometer) mounted at
the drone's body origin: white measurement noise on both channels, plus a
slowly-drifting random-walk bias on the gyro (the dominant real-IMU error
source that pure white noise doesn't capture -- it's what makes gyro-only
integration drift over time and is the reason Stage 1's
``AttitudeEstimator`` needs the accelerometer's gravity reference to correct
it back).

MuJoCo's ``accelerometer`` sensor reports *specific force* (proper
acceleration net of gravity) in the site's local frame, exactly like a real
accelerometer: a stationary, level IMU reads ``[0, 0, +9.81]`` because the
airframe is pushing up against gravity, not ``[0, 0, 0]``.
"""

from typing import Optional, Tuple

import mujoco
import numpy as np

from . import config


class IMUSensor:
    """
    6-axis IMU (gyro + accelerometer) using MuJoCo's ``imu_gyro`` /
    ``imu_accel`` sensors, with configurable white noise and gyro bias
    random walk.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        gyro_noise_std: float = config.IMU_GYRO_NOISE_STD,
        accel_noise_std: float = config.IMU_ACCEL_NOISE_STD,
        gyro_bias_walk_std: float = config.IMU_GYRO_BIAS_WALK_STD,
        enable_noise: bool = config.IMU_ENABLE_NOISE,
        seed: Optional[int] = None,
    ):
        """
        model, data: live MuJoCo model/data (data.sensordata is read each
            call, so ``data`` must be the same object being stepped).
        gyro_noise_std: gyro white-noise standard deviation (rad/s).
        accel_noise_std: accelerometer white-noise standard deviation (m/s^2).
        gyro_bias_walk_std: per-step standard deviation (rad/s) of the gyro
            bias's random walk -- integrated every ``read()`` call to model
            slow bias drift, independent of the sim timestep.
        enable_noise: whether noise/bias are applied at all.
        seed: optional random seed for reproducibility.
        """
        self.model = model
        self.data = data
        self.gyro_noise_std = gyro_noise_std
        self.accel_noise_std = accel_noise_std
        self.gyro_bias_walk_std = gyro_bias_walk_std
        self.enable_noise = enable_noise
        self.rng = np.random.default_rng(seed)

        self.gyro_bias = np.zeros(3)

        self._gyro_adr = self._sensor_adr("imu_gyro")
        self._accel_adr = self._sensor_adr("imu_accel")

    def _sensor_adr(self, name: str) -> int:
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sid < 0:
            raise ValueError(f"Sensor '{name}' not found in model -- check models/quadrotor.xml")
        return int(self.model.sensor_adr[sid])

    def read(self, add_noise: Optional[bool] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Read one IMU sample from the current ``data.sensordata``.

        Returns:
            (accel: np.ndarray[3], gyro: np.ndarray[3]) -- body-frame
            specific force (m/s^2) and angular velocity (rad/s), both with
            noise/bias applied if enabled.
        """
        accel_true = self.data.sensordata[self._accel_adr:self._accel_adr + 3].copy()
        gyro_true = self.data.sensordata[self._gyro_adr:self._gyro_adr + 3].copy()

        should_add_noise = self.enable_noise if add_noise is None else add_noise
        if not should_add_noise:
            return accel_true, gyro_true

        self.gyro_bias += self.rng.normal(0.0, self.gyro_bias_walk_std, size=3)

        accel_meas = accel_true + self.rng.normal(0.0, self.accel_noise_std, size=3)
        gyro_meas = gyro_true + self.gyro_bias + self.rng.normal(0.0, self.gyro_noise_std, size=3)

        return accel_meas, gyro_meas
