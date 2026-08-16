"""
Stage 3 -- Sensing & Environment.

Public interface for other stages to import:

    from stage3_sensing_environment import (
        IRSensor,
        EnvironmentManager,
        get_obstacle_distance,
        create_obstacle_distance_fn,
        config,
    )

Provides IR distance sensing via MuJoCo ray-casting (`mj_ray`), obstacle
management, noise models, and range clipping.
"""

from .ir_sensor import (
    IRSensor,
    create_obstacle_distance_fn,
    get_obstacle_distance,
)
from .environment import EnvironmentManager
from . import config

__all__ = [
    "IRSensor",
    "EnvironmentManager",
    "get_obstacle_distance",
    "create_obstacle_distance_fn",
    "config",
]
