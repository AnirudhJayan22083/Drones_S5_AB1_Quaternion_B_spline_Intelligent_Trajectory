"""
Minimal clamped B-spline evaluator + a time-parameterized trajectory wrapper.

No scipy dependency (the repo only requires mujoco/numpy/matplotlib), so
curve evaluation is a direct implementation of the Cox-de Boor recursion,
and derivatives are formed analytically as lower-degree B-splines over
derived control points -- the standard identity

    d/du C(u) = sum_i  p * (P_{i+1} - P_i) / (U_{i+p+1} - U_{i+1})  N_{i,p-1}(u)

so velocity/acceleration along the curve come from the same machinery as
position, just evaluated on the once/twice-differentiated control polygon.

Waypoints are used directly as control points of a clamped spline (not
interpolated through), which is the standard choice for trajectory
generation: the curve is guaranteed C2-continuous and stays close to the
waypoints without the numerical cost of solving an interpolation system --
and it's what makes ``adaptive_planner``'s control-point perturbation
meaningful (nudging a control point reshapes the local curve predictably).
"""

import numpy as np


def clamped_uniform_knots(n_control_points: int, degree: int) -> np.ndarray:
    """
    Build a clamped, uniform knot vector for ``n_control_points`` control
    points and the given ``degree``. Clamped means the first/last knots
    repeat (degree + 1) times, so the curve starts exactly at the first
    control point and ends exactly at the last one.
    """
    n = n_control_points
    p = degree
    n_internal = n - p - 1
    if n_internal > 0:
        internal = np.linspace(0.0, 1.0, n_internal + 2)[1:-1]
    else:
        internal = np.array([])
    return np.concatenate([np.zeros(p + 1), internal, np.ones(p + 1)])


def de_boor(u: float, degree: int, knots: np.ndarray, control_points: np.ndarray) -> np.ndarray:
    """Evaluate a B-spline curve at parameter u via the de Boor recursion."""
    n = control_points.shape[0] - 1
    p = degree

    if u >= knots[n + 1]:
        k = n
    else:
        k = np.searchsorted(knots, u, side="right") - 1
        k = int(np.clip(k, p, n))

    d = [control_points[j + k - p].astype(float).copy() for j in range(p + 1)]
    for r in range(1, p + 1):
        for j in range(p, r - 1, -1):
            i = j + k - p
            denom = knots[i + p - r + 1] - knots[i]
            alpha = 0.0 if denom == 0.0 else (u - knots[i]) / denom
            d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j]
    return d[p]


class BSpline:
    """A degree-``degree`` B-spline curve over ``control_points`` (N, dim)."""

    def __init__(self, control_points: np.ndarray, knots: np.ndarray, degree: int):
        self.control_points = np.asarray(control_points, dtype=float)
        self.knots = np.asarray(knots, dtype=float)
        self.degree = degree

    def evaluate(self, u: float) -> np.ndarray:
        u = float(np.clip(u, self.knots[0], self.knots[-1]))
        if self.degree < 1:
            # A degree-0 "curve" is piecewise-constant; used only as the
            # base case when repeatedly differentiating a low-order spline.
            n = self.control_points.shape[0] - 1
            k = int(np.clip(np.searchsorted(self.knots, u, side="right") - 1, 0, n))
            return self.control_points[k].copy()
        return de_boor(u, self.degree, self.knots, self.control_points)

    def derivative(self) -> "BSpline":
        """Return a new BSpline representing dC/du."""
        p = self.degree
        n = self.control_points.shape[0]
        if p < 1 or n < 2:
            return BSpline(np.zeros_like(self.control_points[:1]), np.array([0.0, 1.0]), 0)

        U = self.knots
        q_points = np.zeros((n - 1, self.control_points.shape[1]))
        for i in range(n - 1):
            denom = U[i + p + 1] - U[i + 1]
            if denom == 0.0:
                q_points[i] = 0.0
            else:
                q_points[i] = p * (self.control_points[i + 1] - self.control_points[i]) / denom
        q_knots = U[1:-1]
        return BSpline(q_points, q_knots, p - 1)


class BSplineTrajectory:
    """
    Wraps a BSpline with a wall-clock time parameterization: position(t),
    velocity(t), acceleration(t) over t in [0, duration], plus a heading
    estimate derived from the velocity direction.
    """

    def __init__(self, waypoints: np.ndarray, duration: float, degree: int = 3):
        waypoints = np.asarray(waypoints, dtype=float)
        n = waypoints.shape[0]
        if n < 2:
            raise ValueError("need at least 2 waypoints to build a trajectory")

        # Degree can't exceed n - 1 control points; fall back gracefully for
        # short waypoint lists instead of throwing on the knot-vector math.
        degree = min(degree, n - 1)

        self.waypoints = waypoints
        self.degree = degree
        self.duration = float(duration)

        knots = clamped_uniform_knots(n, degree)
        self.spline = BSpline(waypoints, knots, degree)
        self.vel_spline = self.spline.derivative()
        self.acc_spline = self.vel_spline.derivative()

    def _u(self, t: float) -> float:
        if self.duration <= 0.0:
            return 1.0
        return float(np.clip(t / self.duration, 0.0, 1.0))

    def position(self, t: float) -> np.ndarray:
        return self.spline.evaluate(self._u(t))

    def velocity(self, t: float) -> np.ndarray:
        return self.vel_spline.evaluate(self._u(t)) / self.duration

    def acceleration(self, t: float) -> np.ndarray:
        return self.acc_spline.evaluate(self._u(t)) / (self.duration ** 2)

    def yaw(self, t: float) -> float:
        """Heading (rad) pointing along the direction of travel; 0 if near-static."""
        v = self.velocity(t)
        if np.linalg.norm(v[:2]) < 1e-3:
            return 0.0
        return float(np.arctan2(v[1], v[0]))
