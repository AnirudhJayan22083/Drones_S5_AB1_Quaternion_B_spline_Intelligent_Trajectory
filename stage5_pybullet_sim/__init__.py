"""
Stage 5 -- PyBullet physics backend.

Runs Stage 1 (attitude control), Stage 2 (adaptive B-spline planning +
reactive replanning + position control), and Stage 3 (ray-cast sensing +
IMU) unchanged, against PyBullet instead of MuJoCo -- see
``pybullet_backend.py`` for the PyBullet-specific pieces (rigid-body
stepping, ray-cast obstacle sensing, IMU synthesis) that stand in for what
MuJoCo was providing in Stage 4.

    python -m stage5_pybullet_sim.run_pybullet_demo             # headless, saves a plot
    python -m stage5_pybullet_sim.run_pybullet_demo --viewer    # interactive PyBullet GUI
"""

from . import pybullet_backend
from .pipeline import build_pipeline

__all__ = ["pybullet_backend", "build_pipeline"]
