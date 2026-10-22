"""
Stage 7 -- Gazebo (gz-sim) physics backend.

Runs Stage 1 (attitude control), Stage 2 (adaptive B-spline planning +
reactive replanning + position control), and Stage 3 (ray-cast sensing +
IMU) unchanged, against Gazebo instead of MuJoCo/PyBullet -- see
``gazebo_backend.py`` for the Gazebo-specific pieces (world SDF, link
state readout, wrench application, IMU synthesis).

    python -m stage7_gazebo_sim.run_gazebo_demo             # headless, saves a plot
    python -m stage7_gazebo_sim.run_gazebo_demo --viewer    # real-time, opens the Gazebo GUI
"""

from . import gazebo_backend
from .pipeline import build_pipeline

__all__ = ["gazebo_backend", "build_pipeline"]
