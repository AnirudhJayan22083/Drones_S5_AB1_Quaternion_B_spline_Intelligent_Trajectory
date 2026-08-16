"""
IMU-based quaternion attitude estimator (explicit-complementary / Mahony
filter, accelerometer + gyroscope only -- no magnetometer, so absolute yaw
is not observable and is left to drift with the gyro; roll/pitch are fully
observable via the gravity reference).

Reference: Mahony, Hamel & Pflimlin, "Nonlinear Complementary Filters on the
Special Orthogonal Group" (2008). High-rate gyro integration is corrected
every step by a low-rate error signal built from comparing the accelerometer's
measured "up" direction against the direction the current quaternion estimate
predicts gravity should point in the body frame -- this is what bounds the
drift that pure gyro integration would otherwise accumulate.

Convention matches ``quaternion_utils``: q = [w, x, y, z], body -> world.
"""

import numpy as np

from . import quaternion_utils as qu


class AttitudeEstimator:
    """
    Mahony complementary filter: fuses gyro (fast, drifts) with accelerometer
    (slow, drift-free about the gravity vector, but noisy) into a single
    quaternion estimate.
    """

    def __init__(self, kp: float = 1.0, ki: float = 0.1,
                 q0: np.ndarray = None, gravity: float = 9.81):
        """
        kp: proportional gain on the gravity-direction error -- how hard the
            accelerometer pulls the estimate back towards level per second
            of error. Higher = faster correction, more sensitive to accel
            noise (e.g. from vibration/maneuvering thrust).
        ki: integral gain -- slowly learns and cancels a constant gyro bias
            by integrating the same error signal into ``bias_est``, which is
            added back into the corrected gyro rate each step.
        q0: initial quaternion estimate, defaults to identity (level).
        gravity: magnitude of gravity (m/s^2), used only to sanity-scale the
            accelerometer reading -- the correction uses direction, not
            magnitude, so this doesn't need to be exact.
        """
        self.kp = kp
        self.ki = ki
        self.gravity = gravity
        self.q = qu.quat_normalize(np.array([1.0, 0.0, 0.0, 0.0]) if q0 is None else np.asarray(q0, dtype=float))
        self.bias_est = np.zeros(3)

    def reset(self, q0: np.ndarray = None):
        """Reset the filter state (quaternion estimate and learned gyro bias)."""
        self.q = qu.quat_normalize(np.array([1.0, 0.0, 0.0, 0.0]) if q0 is None else np.asarray(q0, dtype=float))
        self.bias_est = np.zeros(3)

    def update(self, accel: np.ndarray, gyro: np.ndarray, dt: float) -> np.ndarray:
        """
        Advance the filter by one sample.

        accel: measured specific force in the body frame (m/s^2) -- a
            stationary, level IMU reads ~[0, 0, +gravity] (see IMUSensor).
        gyro: measured body-frame angular velocity (rad/s).
        dt: time step (s).

        Returns: updated quaternion estimate [w, x, y, z].
        """
        gyro = np.asarray(gyro, dtype=float)
        accel = np.asarray(accel, dtype=float)

        accel_norm = np.linalg.norm(accel)
        if accel_norm > 1e-6:
            accel_dir = accel / accel_norm

            # Direction gravity "should" point in the body frame, per the
            # current estimate: R^T (world->body) applied to world +Z.
            R = qu.quat_to_rotmat(self.q)
            gravity_dir_est = R.T @ np.array([0.0, 0.0, 1.0])

            # Rotational error between measured and predicted "up" -- zero
            # when the estimate is correct, otherwise an axis*sin(angle)
            # vector pointing the way to correct it.
            error = np.cross(accel_dir, gravity_dir_est)
        else:
            error = np.zeros(3)

        self.bias_est += self.ki * error * dt
        gyro_corrected = gyro + self.bias_est + self.kp * error

        # Quaternion kinematics: q_dot = 0.5 * q (x) [0, omega_body]
        omega_quat = np.array([0.0, *gyro_corrected])
        q_dot = 0.5 * qu.quat_mul(self.q, omega_quat)

        self.q = qu.quat_normalize(self.q + q_dot * dt)
        return self.q
