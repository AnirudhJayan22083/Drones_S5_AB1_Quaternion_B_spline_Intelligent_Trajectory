"""
Position controller: turns a position/velocity/acceleration setpoint into a
desired thrust magnitude and desired attitude quaternion, closing the outer
loop around Stage 1's attitude controller.

Standard differentially-flat-quadrotor law (Mellinger & Kumar style):

    a_cmd = Kp*(p_des - p) + Kd*(v_des - v) + a_des_ff + [0, 0, g]
    z_b   = a_cmd / |a_cmd|                     (desired body +Z, world frame)
    T_des = m * (a_cmd . z_b)                   (thrust magnitude along z_b)
    x_b, y_b built from z_b + desired yaw       (heading around thrust axis)
    q_des = quaternion(x_b, y_b, z_b)

a_cmd already includes gravity compensation, so at rest with zero error it
evaluates to [0, 0, g], giving z_b = world-up and T_des = m*g -- exactly
Stage 1's hover condition. Output feeds directly into
``AttitudeController.compute_torque(q, omega, q_des)``.
"""

import numpy as np

from stage1_dynamics_attitude import quaternion_utils as qu


class PositionController:
    def __init__(self, kp: np.ndarray, kd: np.ndarray, mass: float,
                 gravity: float = 9.81, max_tilt_deg: float = 35.0):
        self.kp = np.asarray(kp, dtype=float)
        self.kd = np.asarray(kd, dtype=float)
        self.mass = mass
        self.gravity = gravity
        self.max_tilt = np.radians(max_tilt_deg)

    def compute(self, pos: np.ndarray, vel: np.ndarray,
                pos_des: np.ndarray, vel_des: np.ndarray,
                acc_des: np.ndarray = None, yaw_des: float = 0.0):
        """
        pos, vel: current world-frame position (m) / velocity (m/s).
        pos_des, vel_des: desired position / velocity (from the trajectory).
        acc_des: desired feedforward acceleration (m/s^2), default zero.
        yaw_des: desired heading about world Z (rad).

        Returns: (thrust_des: float, q_des: np.ndarray [w,x,y,z]).
        """
        if acc_des is None:
            acc_des = np.zeros(3)

        a_cmd = (self.kp * (pos_des - pos) + self.kd * (vel_des - vel)
                 + acc_des + np.array([0.0, 0.0, self.gravity]))

        a_norm = np.linalg.norm(a_cmd)
        z_b = np.array([0.0, 0.0, 1.0]) if a_norm < 1e-6 else a_cmd / a_norm

        # Cap the tilt away from vertical so large position errors can't
        # demand an attitude Stage 1's controller has no hope of tracking.
        tilt = np.arccos(np.clip(z_b[2], -1.0, 1.0))
        if tilt > self.max_tilt:
            horiz = np.array([z_b[0], z_b[1], 0.0])
            horiz_norm = np.linalg.norm(horiz)
            horiz_dir = horiz / horiz_norm if horiz_norm > 1e-9 else np.zeros(3)
            z_b = np.cos(self.max_tilt) * np.array([0.0, 0.0, 1.0]) + np.sin(self.max_tilt) * horiz_dir
            z_b = z_b / np.linalg.norm(z_b)

        thrust_des = max(self.mass * np.dot(a_cmd, z_b), 0.0)

        x_c = np.array([np.cos(yaw_des), np.sin(yaw_des), 0.0])
        y_b = np.cross(z_b, x_c)
        y_b_norm = np.linalg.norm(y_b)
        if y_b_norm < 1e-6:
            # x_c is (anti)parallel to z_b, i.e. commanding near-vertical
            # thrust -- pick a heading reference 90 deg away to stay well
            # conditioned.
            x_c = np.array([np.cos(yaw_des + np.pi / 2), np.sin(yaw_des + np.pi / 2), 0.0])
            y_b = np.cross(z_b, x_c)
            y_b_norm = np.linalg.norm(y_b)
        y_b = y_b / y_b_norm
        x_b = np.cross(y_b, z_b)

        R = np.column_stack([x_b, y_b, z_b])
        q_des = qu.quat_from_rotmat(R)

        return thrust_des, q_des
