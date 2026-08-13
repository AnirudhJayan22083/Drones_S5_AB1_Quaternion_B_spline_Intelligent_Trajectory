"""
Adaptive control-point perturbation: nudges B-spline control points away
from nearby obstacles before the trajectory is built, using distance
readings from an injected ``obstacle_distance_fn``.

That function's signature is fixed by Stage 3's contract:

    obstacle_distance_fn(position: np.ndarray, direction: np.ndarray) -> float

i.e. ``get_obstacle_distance(drone_pos, direction)`` -- a single ray query
returning range-to-hit (or a large "clear" value) along ``direction`` from
``position``. Stage 3 hasn't landed yet, so ``default_obstacle_distance_fn``
below is a stand-in: it always reports "clear", so the planner degrades to
a no-op and Stage 2 can be developed/tested standalone. Swap in Stage 3's
real ray-casting function (module-level, same signature) once it exists --
no other change needed here.
"""

import numpy as np

from . import config
from .bspline import BSplineTrajectory

# A coarse but reasonably even set of probe directions (cube face + corner
# normals) used to estimate the nearest-obstacle direction from a point,
# since the Stage 3 contract only exposes single-ray queries, not an
# omnidirectional distance.
_FACE_DIRS = np.array([
    [1, 0, 0], [-1, 0, 0],
    [0, 1, 0], [0, -1, 0],
    [0, 0, 1], [0, 0, -1],
], dtype=float)
_CORNER_DIRS = np.array([
    [sx, sy, sz] for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)
], dtype=float)
_CORNER_DIRS /= np.linalg.norm(_CORNER_DIRS, axis=1, keepdims=True)
DEFAULT_PROBE_DIRECTIONS = np.vstack([_FACE_DIRS, _CORNER_DIRS])


def default_obstacle_distance_fn(position: np.ndarray, direction: np.ndarray,
                                  max_range: float = config.STUB_SENSOR_RANGE) -> float:
    """Placeholder for Stage 3's get_obstacle_distance: reports "no obstacle"."""
    return max_range


class AdaptiveBSplinePlanner:
    def __init__(self, waypoints: np.ndarray, obstacle_distance_fn=None,
                 safe_distance: float = config.SAFE_DISTANCE,
                 gain: float = config.PERTURB_GAIN,
                 max_shift: float = config.MAX_PERTURB_SHIFT,
                 max_iters: int = config.MAX_PLANNER_ITERS,
                 duration: float = None,
                 degree: int = config.BSPLINE_DEGREE,
                 probe_directions: np.ndarray = None):
        """
        waypoints: (N, 3) array; endpoints are held fixed, interior points
            are candidates for perturbation.
        obstacle_distance_fn: Stage 3's get_obstacle_distance(pos, direction)
            -> float. Defaults to ``default_obstacle_distance_fn`` (no
            obstacles) so this class works before Stage 3 exists.
        duration: total trajectory time (s); defaults to
            (N - 1) * config.SECONDS_PER_SEGMENT.
        """
        self.waypoints = np.asarray(waypoints, dtype=float)
        self.obstacle_distance_fn = obstacle_distance_fn or default_obstacle_distance_fn
        self.safe_distance = safe_distance
        self.gain = gain
        self.max_shift = max_shift
        self.max_iters = max_iters
        self.degree = degree
        self.probe_directions = (
            probe_directions if probe_directions is not None else DEFAULT_PROBE_DIRECTIONS
        )

        n_segments = max(self.waypoints.shape[0] - 1, 1)
        self.duration = duration if duration is not None else n_segments * config.SECONDS_PER_SEGMENT

        self.adjusted_waypoints = self.waypoints.copy()

    def _nearest_obstacle(self, point: np.ndarray):
        """Probe all directions from ``point``; return (min_dist, dir_to_it)."""
        best_dist = np.inf
        best_dir = None
        for direction in self.probe_directions:
            dist = self.obstacle_distance_fn(point, direction)
            if dist < best_dist:
                best_dist = dist
                best_dir = direction
        return best_dist, best_dir

    def plan(self) -> BSplineTrajectory:
        """
        Iteratively push interior control points clear of nearby obstacles,
        then build and return the resulting BSplineTrajectory.
        """
        points = self.waypoints.copy()

        for _ in range(self.max_iters):
            moved = False
            for i in range(1, points.shape[0] - 1):
                dist, direction = self._nearest_obstacle(points[i])
                if dist < self.safe_distance:
                    shift = min(self.gain * (self.safe_distance - dist), self.max_shift)
                    # obstacle lies `direction` away from the point, so move
                    # the point the opposite way to clear it.
                    points[i] = points[i] - direction * shift
                    moved = True
            if not moved:
                break

        self.adjusted_waypoints = points
        return BSplineTrajectory(points, duration=self.duration, degree=self.degree)
