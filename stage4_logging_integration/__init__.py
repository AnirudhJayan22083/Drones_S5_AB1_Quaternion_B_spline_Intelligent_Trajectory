"""
Stage 4 -- Logging, Visualization & Integration.

Wires Stage 1 (attitude control), Stage 2 (adaptive B-spline + position
control), and Stage 3 (real IR ray-cast sensing) together into one running
pipeline, logs every step via ``FlightLogger``, and exposes both a headless
integration demo (``run_full_demo``) and an interactive live viewer
(``live_viewer``).

    from stage4_logging_integration import FlightLogger

Run directly:
    python -m stage4_logging_integration.run_full_demo             # headless, saves a plot
    python -m stage4_logging_integration.run_full_demo --viewer    # interactive 3D viewer
    python -m stage4_logging_integration.live_viewer                # always interactive
"""

from .logger import FlightLogger

__all__ = ["FlightLogger"]
