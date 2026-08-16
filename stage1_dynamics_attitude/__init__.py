"""
Stage 1 — Dynamics & Attitude Control.

Public interface for other stages to import:

    from stage1_dynamics_attitude import AttitudeController, MotorMixer, config

Stage 2 (trajectory/position control) feeds ``(q_des, thrust_des)`` (and
optionally ``omega_des``) into ``AttitudeController.compute_torque``, then
``MotorMixer.allocate`` turns (thrust, torque) into the 4 motor commands
that get written to ``data.ctrl``.
"""

from .attitude_controller import AttitudeController
from .attitude_estimator import AttitudeEstimator
from .mixer import MotorMixer
from . import config
from . import quaternion_utils

__all__ = ["AttitudeController", "AttitudeEstimator", "MotorMixer", "config", "quaternion_utils"]
