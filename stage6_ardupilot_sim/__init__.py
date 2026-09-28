"""
Stage 6: ArduPilot SITL Integration via MAVLink.

Streams high-level B-spline trajectory setpoints (position, velocity, acceleration)
from the Stage 2 adaptive planner to an ArduPilot SITL (Software In The Loop) vehicle
in GUIDED mode using pymavlink.
"""

from .ardupilot_bridge import ArduPilotBridge

__all__ = ["ArduPilotBridge"]
