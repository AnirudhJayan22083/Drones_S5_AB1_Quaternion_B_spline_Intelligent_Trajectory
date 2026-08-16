"""
FlightLogger: a minimal, stage-agnostic per-step state recorder.

Stages 1-3's own demo scripts each built an ad-hoc ``log = {...}`` dict
inline. This pulls that pattern into one reusable class with a fixed
schema, so ``run_full_demo.py`` and ``live_viewer.py`` (and any future
Stage 5 work) all log to the same shape and get the same summary/plotting
helpers for free, instead of every script re-inventing its own log dict.
"""

import numpy as np


class FlightLogger:
    def __init__(self):
        self.data = {
            "t": [],
            "pos": [],
            "pos_des": [],
            "q": [],
            "q_des": [],
            "omega": [],
            "motor_thrusts": [],
            "obstacle_dist": [],
        }

    def record(self, t, pos, pos_des, q, q_des, omega, motor_thrusts, obstacle_dist=None):
        """
        Record one simulation step. ``obstacle_dist`` (m) is optional --
        pass None if no obstacle-distance function is wired in for this run
        (e.g. a Stage-1-only hover test); it's logged as NaN and excluded
        from the clearance summary/plot.
        """
        self.data["t"].append(float(t))
        self.data["pos"].append(np.asarray(pos, dtype=float).copy())
        self.data["pos_des"].append(np.asarray(pos_des, dtype=float).copy())
        self.data["q"].append(np.asarray(q, dtype=float).copy())
        self.data["q_des"].append(np.asarray(q_des, dtype=float).copy())
        self.data["omega"].append(np.asarray(omega, dtype=float).copy())
        self.data["motor_thrusts"].append(np.asarray(motor_thrusts, dtype=float).copy())
        self.data["obstacle_dist"].append(float(obstacle_dist) if obstacle_dist is not None else np.nan)

    def as_arrays(self) -> dict:
        """Return every logged field as a numpy array, for plotting/analysis."""
        return {k: np.array(v) for k, v in self.data.items()}

    def summary(self) -> str:
        """
        Human-readable summary matching the print-outs Stage 1/2's demos
        already produce (final/mean/max position error), plus obstacle
        clearance if any was logged.
        """
        arrs = self.as_arrays()
        if len(arrs["t"]) == 0:
            return "FlightLogger: no steps recorded."

        pos_err = np.linalg.norm(arrs["pos"] - arrs["pos_des"], axis=1)
        lines = [
            f"Final position error: {pos_err[-1]:.3f} m",
            f"Mean position error:  {pos_err.mean():.3f} m",
            f"Max position error:   {pos_err.max():.3f} m",
        ]

        od = arrs["obstacle_dist"]
        od_valid = od[~np.isnan(od)]
        if od_valid.size > 0:
            lines.append(f"Min obstacle clearance seen: {od_valid.min():.3f} m")
            if od_valid.min() < 0:
                lines.append("  WARNING: negative clearance logged -- trajectory intersected an obstacle.")

        return "\n".join(lines)
