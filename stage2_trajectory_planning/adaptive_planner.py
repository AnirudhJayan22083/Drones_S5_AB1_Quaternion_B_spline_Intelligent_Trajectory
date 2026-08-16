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

    def plan(self, n_curve_samples: int = 50) -> BSplineTrajectory:
        """
        Iteratively push interior control points clear of nearby obstacles,
        then build and return the resulting BSplineTrajectory.

        A B-spline curve only touches its first/last control points --
        interior control points merely pull the curve toward them, so
        clearing an obstacle at the *control points* doesn't guarantee the
        *curve* clears it too (corner-cutting can still route the curve
        through a gap between a pushed-out point and its unperturbed
        neighbors). So each iteration re-samples the actual candidate curve
        and, for any sample that's too close to an obstacle, pushes the
        nearest interior control point away and rebuilds -- not just the
        raw waypoints.
        """
        points = self.waypoints.copy()
        traj = BSplineTrajectory(points, duration=self.duration, degree=self.degree)

        if points.shape[0] < 3:
            # Only a start and an end -- no interior control point exists to
            # push away from an obstacle, so there's nothing this method can
            # do; return the (degree-clamped, effectively straight-line) curve
            # as-is rather than crash looking for a control point that isn't
            # there.
            self.adjusted_waypoints = points
            return traj

        for _ in range(self.max_iters):
            moved = False
            interior = points[1:-1]
            for s in np.linspace(0.0, 1.0, n_curve_samples):
                sample_pos = traj.position(s * self.duration)
                dist, direction = self._nearest_obstacle(sample_pos)
                if dist < self.safe_distance and direction is not None:
                    idx = 1 + int(np.argmin(np.linalg.norm(interior - sample_pos, axis=1)))
                    shift = min(self.gain * (self.safe_distance - dist), self.max_shift)
                    # obstacle lies `direction` away from the sample, so move
                    # the nearest control point the opposite way to clear it.
                    points[idx] = points[idx] - direction * shift
                    interior = points[1:-1]
                    moved = True
            if not moved:
                break
            traj = BSplineTrajectory(points, duration=self.duration, degree=self.degree)

        self.adjusted_waypoints = points
        return traj


class ReactiveReplanner:
    """
    Starts by flying the *naive* B-spline curve straight through the raw
    waypoints -- no obstacle-avoidance pre-scan, so the drone does not know
    an obstacle exists at takeoff. Only once the live ``obstacle_distance_fn``
    reports something within ``detection_range`` of the drone's *current*
    position does it replan the remaining route (via AdaptiveBSplinePlanner,
    seeded from where the drone actually is right now) and switch to flying
    that instead -- mirroring a sensor-limited vehicle that reacts to what
    it currently senses, not a route computed with full foreknowledge of
    the whole map before takeoff.

    ``detection_range`` must be bigger than ``safe_distance`` (it defaults
    to a fixed margin above it) -- otherwise "detected" and "already
    inside the safety margin" would be the same moment, leaving no room to
    react.

    Detection uses two-threshold hysteresis, not a single boundary: it
    triggers when distance drops below ``detection_range``, and only
    re-arms once distance climbs back above ``rearm_range`` (bigger than
    ``detection_range``). A single shared boundary chatters, because the
    "nearest obstacle" reading comes from the min over a fixed set of probe
    directions (not a true continuous omnidirectional distance) -- as the
    drone turns during a maneuver, the reading can jump between directions
    and briefly nudge back above a single threshold while still genuinely
    close, re-triggering a fresh replan before the previous one had a
    chance to actually clear the obstacle. A ``min_replan_gap`` cooldown
    backs this up so two replans can never land closer together than the
    control loop can meaningfully act on anyway.

    Original waypoints are treated as nominally due at
    t_i = i * seconds_per_segment (matching AdaptiveBSplinePlanner's own
    SECONDS_PER_SEGMENT convention) purely to decide which ones still count
    as "ahead" of the drone at replan time -- interior waypoints were never
    interpolated exactly by the curve to begin with, replanning or not.
    """

    def __init__(self, waypoints: np.ndarray, obstacle_distance_fn,
                 seconds_per_segment: float = config.SECONDS_PER_SEGMENT,
                 detection_range: float = config.DETECTION_RANGE,
                 rearm_range: float = config.REARM_RANGE,
                 min_replan_gap: float = config.MIN_REPLAN_GAP,
                 safe_distance: float = config.SAFE_DISTANCE,
                 gain: float = config.PERTURB_GAIN,
                 max_shift: float = config.MAX_PERTURB_SHIFT,
                 max_iters: int = config.MAX_PLANNER_ITERS,
                 degree: int = config.BSPLINE_DEGREE,
                 probe_directions: np.ndarray = None):
        self.waypoints = np.asarray(waypoints, dtype=float)
        self.obstacle_distance_fn = obstacle_distance_fn
        self.seconds_per_segment = seconds_per_segment
        self.detection_range = detection_range
        self.rearm_range = rearm_range
        self.min_replan_gap = min_replan_gap
        self.safe_distance = safe_distance
        self.gain = gain
        self.max_shift = max_shift
        self.max_iters = max_iters
        self.degree = degree
        self.probe_directions = (
            probe_directions if probe_directions is not None else DEFAULT_PROBE_DIRECTIONS
        )

        n_segments = max(self.waypoints.shape[0] - 1, 1)
        self.duration = n_segments * seconds_per_segment
        self._nominal_arrival = np.arange(self.waypoints.shape[0]) * seconds_per_segment

        # No pre-flight avoidance scan -- fly straight through the raw
        # waypoints until something is actually detected en route.
        self._segment_traj = BSplineTrajectory(self.waypoints, duration=self.duration, degree=self.degree)
        self._segment_start_t = 0.0
        self.adjusted_waypoints = self.waypoints.copy()

        self._armed = True  # True == currently outside rearm_range (can trigger again)
        self._last_replan_t = -np.inf
        self.replan_count = 0

    def _live_nearest_obstacle(self, point: np.ndarray) -> float:
        best = np.inf
        for direction in self.probe_directions:
            dist = self.obstacle_distance_fn(point, direction)
            if dist < best:
                best = dist
        return best

    def maybe_replan(self, t: float, current_pos: np.ndarray) -> bool:
        """
        Call once per sim step with the current time and the drone's actual
        position. Replans the remaining route in place the moment an
        obstacle is newly detected within ``detection_range``. Returns True
        iff a replan happened, so callers (e.g. the viewer) know to redraw.
        """
        if t >= self.duration - 1e-6:
            return False

        dist = self._live_nearest_obstacle(current_pos)
        if dist >= self.rearm_range:
            self._armed = True
        triggers = (dist < self.detection_range and self._armed
                    and t - self._last_replan_t >= self.min_replan_gap)
        if not triggers:
            return False
        self._armed = False

        k = int(np.searchsorted(self._nominal_arrival, t, side="right"))
        k = int(np.clip(k, 1, self.waypoints.shape[0] - 1))
        remaining = np.vstack([current_pos, self.waypoints[k:]])
        if remaining.shape[0] < 2:
            return False

        remaining_duration = max(self.duration - t, 1e-3)
        segment_planner = AdaptiveBSplinePlanner(
            remaining,
            obstacle_distance_fn=self.obstacle_distance_fn,
            safe_distance=self.safe_distance,
            gain=self.gain,
            max_shift=self.max_shift,
            max_iters=self.max_iters,
            duration=remaining_duration,
            degree=self.degree,
        )
        self._segment_traj = segment_planner.plan()
        self._segment_start_t = t
        self.adjusted_waypoints = segment_planner.adjusted_waypoints
        self._last_replan_t = t
        self.replan_count += 1
        return True

    def _local_t(self, t: float) -> float:
        return t - self._segment_start_t

    def position(self, t: float) -> np.ndarray:
        return self._segment_traj.position(self._local_t(t))

    def velocity(self, t: float) -> np.ndarray:
        return self._segment_traj.velocity(self._local_t(t))

    def acceleration(self, t: float) -> np.ndarray:
        return self._segment_traj.acceleration(self._local_t(t))

    def yaw(self, t: float) -> float:
        return self._segment_traj.yaw(self._local_t(t))

    def sample_active_curve(self, n_samples: int = 60) -> np.ndarray:
        """Sample the currently-active segment over its own remaining time span (for visualization)."""
        ts = np.linspace(self._segment_start_t,
                          self._segment_start_t + self._segment_traj.duration, n_samples)
        return np.array([self.position(t) for t in ts])
