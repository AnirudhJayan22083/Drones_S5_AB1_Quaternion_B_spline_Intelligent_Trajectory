"""
Quaternion-based attitude controller.

Implements the error-quaternion PD law of Fresk & Nikolakopoulos, "Full
Quaternion Based Attitude Control for a Quadrotor" (2013):

    q_e = q_des^{-1} (x) q
    tau = -Kp * sign(q_e.w) * q_e.vec  -  Kd * (omega - omega_des)  +  omega x (J omega)

- The sign(q_e.w) term picks the shortest rotational path, avoiding the
  "unwind" problem where the controller spins the long way around because a
  quaternion and its negation represent the same rotation.
- The final term is a gyroscopic feedforward (omega x (J omega)) that
  cancels the rigid-body Coriolis coupling between axes; it's optional
  (set ``feedforward=False``) but improves tracking once you're off the
  origin.

All inputs/outputs are in the body frame, matching MuJoCo's free-joint
convention: ``qvel[3:6]`` is body-frame angular velocity, and
``qpos[3:7]`` is the world-frame body orientation quaternion.
"""

import numpy as np

from . import quaternion_utils as qu


class AttitudeController:
    def __init__(self, kp: np.ndarray, kd: np.ndarray, inertia_diag: np.ndarray,
                 feedforward: bool = True):
        """
        kp, kd: 3-vectors of proportional/derivative gains (roll, pitch, yaw).
        inertia_diag: 3-vector, principal moments of inertia (kg*m^2).
        feedforward: include the omega x (J omega) gyroscopic term.
        """
        self.kp = np.asarray(kp, dtype=float)
        self.kd = np.asarray(kd, dtype=float)
        self.J = np.asarray(inertia_diag, dtype=float)
        self.feedforward = feedforward

    def compute_torque(self, q: np.ndarray, omega: np.ndarray,
                        q_des: np.ndarray, omega_des: np.ndarray = None) -> np.ndarray:
        """
        q, q_des: current / desired orientation, [w, x, y, z], world frame.
        omega: current body-frame angular velocity (rad/s).
        omega_des: desired body-frame angular velocity (rad/s), default zero.

        Returns: body-frame torque command (3-vector, N*m).
        """
        if omega_des is None:
            omega_des = np.zeros(3)

        q = qu.quat_normalize(q)
        q_des = qu.quat_normalize(q_des)

        q_e = qu.quat_error(q, q_des)
        sign = 1.0 if q_e[0] >= 0.0 else -1.0
        q_e_vec = q_e[1:4]

        omega_err = omega - omega_des

        tau = -self.kp * sign * q_e_vec - self.kd * omega_err

        if self.feedforward:
            tau = tau + np.cross(omega, self.J * omega)

        return tau
