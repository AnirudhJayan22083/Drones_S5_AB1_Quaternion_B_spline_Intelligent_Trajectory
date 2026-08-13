"""
Motor mixing: converts a desired (total thrust, body torque) pair into 4
individual motor thrust commands, matching the "+"-configuration layout in
``models/quadrotor.xml`` / ``config.py``.

Each motor i sits at position p_i (body frame) and produces a force f_i
along body +Z plus a reaction torque spin_i * k_yaw * f_i about body Z.
Stacking the wrench contribution of all 4 motors gives a linear map

    [T, tau_x, tau_y, tau_z]^T = A @ [f0, f1, f2, f3]^T

where
    T       = sum_i f_i
    tau_x   = sum_i  y_i * f_i
    tau_y   = sum_i -x_i * f_i
    tau_z   = sum_i  spin_i * k_yaw * f_i

A is fixed by the airframe geometry, so it's built and inverted once at
construction time; ``allocate`` just applies A^{-1} and clips/rescales to
respect actuator limits.
"""

import numpy as np


class MotorMixer:
    def __init__(self, motor_positions: np.ndarray, motor_spin: np.ndarray,
                 yaw_torque_coeff: float, thrust_min: float, thrust_max: float):
        """
        motor_positions: (4, 3) array of motor positions in the body frame.
        motor_spin: (4,) array of +-1 spin directions.
        yaw_torque_coeff: reaction-torque-per-thrust coefficient (m).
        thrust_min, thrust_max: per-motor thrust limits (N).
        """
        motor_positions = np.asarray(motor_positions, dtype=float)
        motor_spin = np.asarray(motor_spin, dtype=float)
        n = motor_positions.shape[0]

        A = np.zeros((4, n))
        A[0, :] = 1.0                          # total thrust
        A[1, :] = motor_positions[:, 1]        # tau_x from y_i
        A[2, :] = -motor_positions[:, 0]       # tau_y from -x_i
        A[3, :] = motor_spin * yaw_torque_coeff  # tau_z from reaction torque

        self.A = A
        self.A_inv = np.linalg.inv(A)
        self.thrust_min = thrust_min
        self.thrust_max = thrust_max

    def allocate(self, total_thrust: float, torque: np.ndarray) -> np.ndarray:
        """
        total_thrust: desired total thrust along body +Z (N).
        torque: desired body torque [tau_x, tau_y, tau_z] (N*m).

        Returns: (4,) array of per-motor thrust commands, clipped to
        [thrust_min, thrust_max]. If any motor saturates, all four are
        rescaled together (preserving the requested torque direction as
        closely as possible) before the final per-motor clip.
        """
        wrench = np.concatenate(([total_thrust], np.asarray(torque, dtype=float)))
        f = self.A_inv @ wrench

        peak = np.max(np.abs(f))
        if peak > self.thrust_max:
            f = f * (self.thrust_max / peak)

        return np.clip(f, self.thrust_min, self.thrust_max)
