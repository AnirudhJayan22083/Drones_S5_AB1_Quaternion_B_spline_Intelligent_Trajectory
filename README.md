<p align="center">
  <img src="assets/amrita_logo.jpeg" alt="Amrita Logo" width="60%">
</p>

## Team Member Details

| Name | Roll No. | Email |
|---|---|---|
| Anirudh Jayan | CB.SC.U4AIE24007 | cb.sc.u4aie24007@cb.students.amrita.edu |
| Anantha Krishna S K | CB.SC.U4AIE24102 | cb.sc.u4aie24102@cb.students.amrita.edu |
| Aravind S Harilal | CB.SC.U4AIE24008 | cb.sc.u4aie24008@cb.students.amrita.edu |
| Yaswant Reddy | CB.SC.U4AIE24061 | cb.sc.u4aie24061@cb.students.amrita.edu |

---

# Quaternion-Based-Autonomous-Quadrotor-Navigation-using-adaptive-B-Spline-Trajectory-Generation

---

## Title

**Quaternion-Based-Autonomous-Quadrotor-Navigation-using-adaptive-B-Spline-Trajectory-Generation**

---

## Abstract

This project develops a quaternion-based autonomous navigation framework for a
quadrotor that follows smooth trajectories generated using B-spline curves. Unlike
conventional waypoint navigation, which causes abrupt movements and oscillations, the
proposed system generates continuous trajectories for stable and efficient flight. The
quadrotor's attitude is represented entirely using unit quaternions, eliminating the
singularities associated with Euler angles. A nonlinear quaternion-based attitude
controller is combined with a position controller to track the generated trajectory. The
system is extended with an adaptive B-spline layer that reacts to obstacles detected via
simulated IR ray-cast sensing, locally perturbing trajectory control points to maintain
a required safety margin without discarding the smoothness guarantees of the underlying
spline. The entire system is validated in simulation (MuJoCo), with the framework
designed to later extend toward deployment on a LiteWing ESP32-based flight controller.

---

## Introduction

Autonomous quadrotor navigation requires simultaneously solving two coupled problems:
generating a trajectory that is dynamically feasible and collision-free, and controlling
the vehicle's attitude and position precisely enough to track that trajectory in real
time. Two design choices in this project directly address known limitations of common
alternative approaches:

**Why quaternions instead of Euler angles.** Euler angle representations suffer from
gimbal lock — a singularity where two rotational axes become aligned, causing a loss of
one degree of rotational freedom and unstable controller behavior near that
configuration. Unit quaternions represent orientation without this singularity, at the
cost of a less immediately intuitive representation. This project uses quaternions
throughout the attitude estimation and control pipeline for this reason.

**Why B-splines instead of direct waypoint navigation.** Naively flying directly between
waypoints (or interpolating them with a single high-degree polynomial) produces abrupt
changes in commanded velocity and acceleration at each waypoint, causing oscillation and
inefficient flight. B-spline curves provide continuous position, velocity, and
acceleration by construction (for a cubic, degree-3 spline), and their local-support
property means a trajectory can be locally edited — for example, to react to a newly
detected obstacle — without needing to recompute the entire path.

Combining these two ideas, and extending the trajectory layer to adapt around obstacles
detected via onboard sensing, is the central contribution this project implements and
validates in simulation.

---

## Methodology

The system is implemented as four integrated stages, each validated independently
before being combined into a full pipeline.

### 1. Dynamics & Attitude Control

A quadrotor rigid-body model is simulated in MuJoCo, with four direct force/torque
motor actuators in a "+" configuration. Attitude is tracked as a unit quaternion, and a
nonlinear attitude controller computes body-frame torque commands from the error
between the current and desired quaternion. The control law is derived using a Lyapunov
stability argument, guaranteeing convergence to the desired attitude, and includes a
feedforward term canceling the gyroscopic coupling inherent to rigid-body rotational
dynamics. Motor mixing converts the resulting thrust and torque commands into
individual motor thrust values.

### 2. Trajectory Generation & Position Control

A cubic B-spline is generated from a sequence of waypoints, providing smooth,
continuously differentiable position, velocity, and acceleration references. A position
controller computes the world-frame force required to track this trajectory (combining
gravity compensation, proportional-derivative feedback on position/velocity error, and
feedforward trajectory acceleration), then converts that force into a required thrust
magnitude and a desired attitude quaternion — since the underactuated quadrotor can only
produce thrust along its own body axis and must tilt to redirect it.

### 3. Adaptive Obstacle Sensing

Obstacle detection is simulated using ray-casting against the MuJoCo environment,
modeling a short-range IR distance sensor with realistic range limits and measurement
noise. When a trajectory control point is found to violate a required safety clearance,
it is perturbed away from the obstacle along the sensed escape direction, with the
correction applied iteratively and bounded by a maximum per-step shift for numerical
stability. This adaptive layer allows the planned trajectory to react to obstacles
without requiring a full replan of the path.

### 4. Integration, Logging & Visualization

The three stages above are combined into a single running pipeline: the adaptive
trajectory planner produces a reference path, the position controller tracks it and
requests an attitude, the attitude controller achieves that attitude, and the resulting
motor commands drive the MuJoCo simulation forward in time. Every step is logged
(position, attitude, tracking error, and live obstacle clearance) to validate the system
end to end, both as a headless batch simulation producing summary plots and as an
interactive live viewer for qualitative inspection of flight behavior.

---

## Repository Structure

```
stage1_dynamics_attitude/       Quadrotor dynamics, quaternion attitude control, motor mixing
stage2_trajectory_planning/     B-spline trajectory generation, adaptive replanning, position control
stage3_sensing_environment/     Simulated IR obstacle sensing
stage4_logging_integration/     Full pipeline integration, logging, and live/headless visualization
models/                         MuJoCo XML model(s)
outputs/                        Generated plots from each stage's demo scripts
```

---

## Mathematical Formulation

This section derives the control and trajectory-generation laws implemented in the
codebase. Notation mostly follows the base paper (Bangura *et al.*, 2014, cited in
[References](#references)): world frame $\{A\}$, body frame $\{B\}$, position
$\zeta=(x,y,z)^\top$, and unit quaternion $q=(q_0,\vec q)^\top$, $\vec
q=(q_1,q_2,q_3)^\top$, $\lVert q\rVert=1$.

### 1. Rigid-Body Dynamics

The quadrotor is modeled as a rigid body of mass $m$ and inertia tensor
$I=\mathrm{diag}(I_{xx},I_{yy},I_{zz})$, actuated by a total thrust $T$ along the body
$+Z$ axis and a body torque $\tau=(\tau_x,\tau_y,\tau_z)^\top$:

$$
\dot\zeta = v, \qquad
m\dot v = mge_3 - TRe_3, \qquad
\dot R = RS(\Omega), \qquad
I\dot\Omega = -S(\Omega)I\Omega + \tau,
$$

where $R\in SO(3)$ is the body-to-world rotation, $\Omega$ is the body-frame angular
velocity, $e_3=(0,0,1)^\top$, and $S(\cdot)$ is the skew-symmetric cross-product
matrix, $S(\Omega)w=\Omega\times w$. The quaternion kinematics equivalent to $\dot
R=RS(\Omega)$ is

$$
\dot q = \tfrac12\, q\otimes\mathfrak p(\Omega), \qquad \mathfrak p(\Omega):=(0,\Omega)^\top,
$$

with $\otimes$ the Hamilton product implemented in
[`quaternion_utils.quat_mul`](stage1_dynamics_attitude/quaternion_utils.py).

### 2. Motor Mixing

The four "+"-configuration rotors, each at position $p_i$ in the body frame with spin
direction $s_i=\pm1$ and yaw-drag coefficient $\kappa$, produce the wrench

$$
\begin{bmatrix}T\\\tau_x\\\tau_y\\\tau_z\end{bmatrix}
= \underbrace{\begin{bmatrix}
1 & 1 & 1 & 1\\
p_{1y} & p_{2y} & p_{3y} & p_{4y}\\
-p_{1x} & -p_{2x} & -p_{3x} & -p_{4x}\\
s_1\kappa & s_2\kappa & s_3\kappa & s_4\kappa
\end{bmatrix}}_{A}
\begin{bmatrix}f_1\\f_2\\f_3\\f_4\end{bmatrix},
$$

a fixed-geometry linear allocation map (analogous to Eq. (3) of the base paper). Per-motor
thrusts are recovered as $f=A^{-1}[T,\tau]^\top$ and saturated to
$[T_{\min},T_{\max}]$ — see [`mixer.py`](stage1_dynamics_attitude/mixer.py).

### 3. Quaternion Attitude Control

Given current attitude $q$ and desired attitude $q_d$, the attitude error quaternion is

$$
q_e = q_d^{-1}\otimes q = (q_{e0},\vec q_e)^\top ,
$$

which satisfies $q_e\to(1,0,0,0)^\top$ as $q\to q_d$. With angular-rate error
$\tilde\Omega=\Omega-\Omega_d$, the implemented control law (a Fresk & Nikolakopoulos
(2013)-style PD law, structurally the sign-based analogue of the $K_\Omega(\tilde\Omega
+K_q\vec q_e)$ law of Theorem 1 in the base paper) is

$$
\tau = -K_p\,\mathrm{sgn}(q_{e0})\,\vec q_e \;-\; K_d\,\tilde\Omega \;+\; \Omega\times(I\Omega),
$$

implemented in
[`attitude_controller.compute_torque`](stage1_dynamics_attitude/attitude_controller.py).
The $\mathrm{sgn}(q_{e0})$ factor always selects the shorter rotational path (since $q$
and $-q$ represent the same physical attitude), avoiding the "unwinding" phenomenon the
base paper's Section 5.3 handles by matching the sign of $q_{d_0}$ to $q_0$ at every
re-initialization; the final term $\Omega\times(I\Omega)$ is the gyroscopic
feedforward that cancels rigid-body Coriolis coupling, i.e. the base paper's
$S(\Omega)I\Omega$ term in Eq. (23)/(24). Substituting the control law into the
rotational dynamics yields the (locally, around $q_e=(1,0,0,0)^\top$) exponentially
stable closed-loop error dynamics established via the Lyapunov argument of the base
paper's Theorem 1:

$$
V_1(q_e,\tilde\Omega) = 2(1-q_{e0}) + \tfrac{1}{2k}\tilde\Omega^\top I\tilde\Omega,
\qquad
\dot V_1 = -\tfrac1k\tilde\Omega^\top K_\Omega\tilde\Omega \le 0 .
$$

### 4. Position (Outer-Loop) Control

Following the differentially-flat quadrotor formulation (Mellinger & Kumar style, and
structurally equivalent to the base paper's trajectory controller of Theorem 2), the
translational error $\tilde\zeta=\zeta-\zeta_d$ obeys $m\ddot{\tilde\zeta}= mge_3 -
TRe_3 -m\ddot\zeta_d$. Choosing the commanded acceleration

$$
a_{cmd} = K_p\big(\zeta_d-\zeta\big) + K_d\big(\dot\zeta_d-\dot\zeta\big) + \ddot\zeta_d + ge_3
$$

drives the linear error dynamics $m\ddot{\tilde\zeta}=-K_p\tilde\zeta-K_d\dot{\tilde\zeta}$
to zero (Theorem 2 of the base paper, exponentially stable for diagonal positive-definite
$K_p,K_d$). The desired body $+Z$ axis, thrust magnitude, and full attitude are then
recovered algebraically (this project's version of the base paper's $x_{con},y_{con},z_{con}$
/ Eq. (44)–(47) derivation):

$$
z_b = \frac{a_{cmd}}{\lVert a_{cmd}\rVert}, \qquad
T_d = m\,a_{cmd}\cdot z_b, \qquad
x_b = y_b\times z_b, \quad y_b = \frac{z_b\times x_c}{\lVert z_b\times x_c\rVert},
$$

where $x_c=(\cos\psi_d,\sin\psi_d,0)^\top$ encodes the desired heading $\psi_d$, and
$q_d$ is recovered from the rotation matrix $R_d=[x_b\ y_b\ z_b]$ via Shepperd's method
(see [`position_controller.py`](stage2_trajectory_planning/position_controller.py),
[`quaternion_utils.quat_from_rotmat`](stage1_dynamics_attitude/quaternion_utils.py)).

### 5. B-Spline Trajectory Generation

A degree-$p$ (cubic, $p=3$) B-spline curve over control points
$P_0,\dots,P_n$ and a clamped, uniform knot vector $U=\{u_0,\dots,u_m\}$ is evaluated
via the Cox–de Boor recursion

$$
N_{i,0}(u) = \begin{cases}1 & u_i\le u < u_{i+1}\\ 0 & \text{otherwise}\end{cases},
\qquad
N_{i,p}(u) = \frac{u-u_i}{u_{i+p}-u_i}N_{i,p-1}(u) + \frac{u_{i+p+1}-u}{u_{i+p+1}-u_{i+1}}N_{i+1,p-1}(u),
$$

$$
C(u) = \sum_{i=0}^{n} N_{i,p}(u)\,P_i .
$$

The codebase implements the de Boor evaluation directly
([`bspline.de_boor`](stage2_trajectory_planning/bspline.py)) rather than the raw
basis-function sum. Derivatives are formed analytically as a lower-degree B-spline over a
differenced control polygon,

$$
\frac{d}{du}C(u) = \sum_{i=0}^{n-1} p\,\frac{P_{i+1}-P_i}{u_{i+p+1}-u_{i+1}}\;N_{i,p-1}(u),
$$

applied once for velocity and twice for acceleration
([`BSpline.derivative`](stage2_trajectory_planning/bspline.py)). Clamping the knot
vector (repeating the first/last knots $p+1$ times) guarantees $C(0)=P_0$,
$C(1)=P_n$, and $C^2$ continuity elsewhere — avoiding the velocity/acceleration
discontinuities of piecewise-linear waypoint interpolation. Wall-clock time is mapped to
the curve parameter by $u(t)=\mathrm{clip}(t/T_{\text{dur}},0,1)$, so

$$
\text{pos}(t) = C(u(t)), \qquad
\text{vel}(t) = \frac{1}{T_{\text{dur}}}\,C'(u(t)), \qquad
\text{acc}(t) = \frac{1}{T_{\text{dur}}^2}\,C''(u(t)).
$$

### 6. Adaptive Obstacle Avoidance

Stage 3 provides a ray-cast distance reading $d(\mathbf x,\mathbf n)$ from a point
$\mathbf x$ along direction $\mathbf n$. For each interior control point $P_i$, the
nearest sensed obstacle distance $d^\*=\min_{\mathbf n\in\mathcal D} d(P_i,\mathbf n)$
is found by probing a fixed direction set $\mathcal D$ (cube-face + corner normals). If
$d^\*$ falls below a safety margin $d_{\text{safe}}$ (drone body radius + clearance), the
control point is perturbed away from the obstacle along the sensed direction
$\mathbf n^\*$:

$$
P_i \leftarrow P_i - \mathbf n^\* \cdot \min\big(k_{\text{gain}}\,(d_{\text{safe}}-d^\*),\; \delta_{\max}\big),
$$

with gain $k_{\text{gain}}$ and a per-step cap $\delta_{\max}$ for numerical stability
([`AdaptiveBSplinePlanner.plan`](stage2_trajectory_planning/adaptive_planner.py)). The
curve is resampled and the check repeated for up to $N_{\text{iters}}$ passes, since a
B-spline only interpolates its end control points — pushing a control point clear does
not by itself guarantee the *curve* clears the obstacle (corner-cutting). At the flight
level, `ReactiveReplanner` triggers this procedure online, from the drone's live
position, only once $d^\*$ drops below a detection radius $d_{\text{det}} >
d_{\text{safe}}$, with hysteresis re-arming above $d_{\text{rearm}} > d_{\text{det}}$
and a minimum inter-replan gap, so the trajectory reacts to obstacles it did not know
about at takeoff without chattering between replans.

---

## Results

All results below are produced by the repository's own demo scripts
(`python -m stageN_..._demo`), run headless against the MuJoCo model in
[`models/quadrotor.xml`](models/quadrotor.xml), and logged with
[`FlightLogger`](stage4_logging_integration/logger.py).

### Table 1 — Per-Stage Tracking Performance

| Stage | Scenario | Final error | Mean error | Max error |
|---|---|---|---|---|
| 1 — Dynamics & Attitude | Hover at 1.0 m, step attitude command | 0.237° (attitude) | — | — |
| 2 — Trajectory & Position Control | Smooth multi-waypoint B-spline path, no obstacles | 0.012 m | 0.034 m | 0.146 m |
| 4 — Full Integration | B-spline route with 2 obstacles, live IR sensing + reactive replanning | 0.240 m | 0.622 m | 2.037 m |

### Table 2 — Adaptive Obstacle Avoidance (Stage 4)

| Metric | Value |
|---|---|
| Obstacles in scene | 2 |
| Reactive replans triggered | 5 (at *t* ≈ 1.25 s, 2.25 s, 4.46 s, 5.59 s, 7.83 s) |
| Safety margin, $d_{\text{safe}}$ | 0.5 m + drone radius |
| Minimum obstacle clearance observed in flight | 0.612 m (never violates $d_{\text{safe}}$) |

Position error rises through Stage 2 → Stage 4 because Stage 4 adds the full noisy
sensing/estimation chain (Mahony-filter attitude *estimate* driving the controller
instead of MuJoCo's ground truth, IMU gyro/accelerometer noise and bias drift, and — most
significantly — the multiple mid-flight replans, each of which introduces a transient
while the position loop re-converges to a newly perturbed reference). The peak 2.037 m
error corresponds to one of these replan transients, not steady-state divergence; the
system recovers to sub-half-meter tracking within the following segment, as visible in
the tracking-error plot below.

### Figures

**Stage 1 — Attitude tracking and hover altitude:**

![Stage 1: hover and attitude tracking](assets/results/stage1_hover_tracking.png)

**Stage 2 — B-spline trajectory tracking (no obstacles):**

![Stage 2: trajectory tracking](assets/results/stage2_trajectory_tracking.png)

**Stage 3 — Simulated IR ray-cast obstacle sensing:**

![Stage 3: sensor demo](assets/results/stage3_sensor_demo.png)

**Stage 4 — Full pipeline: path, tracking error, live obstacle clearance, attitude:**

![Stage 4: full integration](assets/results/stage4_full_integration.png)

**Animation — Stage 4 flight, top-down (planned path vs. flown path):**

![Stage 4 trajectory animation](assets/results/stage4_trajectory_tracking.gif)

The animation shows the drone (blue) departing from the naive planned route (gray
dashed) as `ReactiveReplanner` perturbs the remaining control points around the two
obstacles (red discs) and re-converging onto the goal, tracking the orange desired
setpoint marker throughout.

---

## Conclusion

This project implements and validates, in a MuJoCo simulation, a complete hierarchical
navigation stack for a quadrotor: a quaternion-based nonlinear attitude controller with
gyroscopic feedforward (Stage 1), a differentially-flat position controller driving a
clamped cubic B-spline trajectory generated directly from Cox–de Boor evaluation over the
waypoint control polygon (Stage 2), simulated IR ray-cast obstacle sensing (Stage 3), and
an adaptive control-point perturbation layer that reactively replans the remaining route
online in response to sensed obstacles (Stage 4). The results confirm the two central
design choices: (1) quaternion attitude representation avoided any gimbal-lock failure
across all tested maneuvers, converging to within 0.24° of the commanded attitude during
hover; and (2) the B-spline/adaptive-replanning layer kept the vehicle's minimum sensed
obstacle clearance (0.612 m) safely above the configured margin across five independent
in-flight replans, without ever requiring a full re-plan of the whole path — confirming
the practical value of a spline's local-support property for reactive obstacle avoidance.
The largest tracking errors in the full pipeline coincide with replan transients rather
than steady-state drift, indicating the remaining performance headroom is in the
position-loop's transient response to a suddenly-shifted reference, not in the
underlying attitude or trajectory-generation math. Natural next steps are (a) tuning or
gain-scheduling the position controller specifically for the post-replan transient, (b)
extending the obstacle representation beyond ray-cast point sampling (e.g. an inflated
occupancy/ESDF map) for denser environments, and (c) porting the estimator/controller
pair — already structured around body-frame quaternion and gyro inputs matching a real
flight-controller ABI — to the target LiteWing ESP32 hardware.

---

## References

1. **Base paper** — M. Bangura, R. Mahony, H. Lim, and H. J. Kim, "An Open-Source
   Implementation of a Unit Quaternion based Attitude and Trajectory Tracking for
   Quadrotors," *Proceedings of the Australasian Conference on Robotics and Automation
   (ACRA)*, Melbourne, 2014.
   [PDF](https://www.araa.asn.au/acra/acra2014/papers/pap105.pdf)
2. R. Mahony, T. Hamel, and J.-M. Pflimlin, "Nonlinear Complementary Filters on the
   Special Orthogonal Group," *IEEE Transactions on Automatic Control*, vol. 53, no. 5,
   2008.
3. D. Mellinger and V. Kumar, "Minimum Snap Trajectory Generation and Control for
   Quadrotors," *IEEE International Conference on Robotics and Automation (ICRA)*, 2011.
4. T. Lee, M. Leok, and N. H. McClamroch, "Geometric Tracking Control of a Quadrotor UAV
   on SE(3)," *IEEE Conference on Decision and Control (CDC)*, 2010.
5. A. Tayebi and S. McGilvray, "Attitude Stabilization of a VTOL Quadrotor Aircraft,"
   *IEEE Transactions on Control Systems Technology*, vol. 14, no. 3, 2006.
6. L. Piegl and W. Tiller, *The NURBS Book*, 2nd ed., Springer, 1997 — Cox–de Boor
   recursion and B-spline derivative identities used in Stage 2's implementation.
7. E. Fresk and G. Nikolakopoulos, "Full Quaternion Based Attitude Control for a
   Quadrotor," *European Control Conference (ECC)*, 2013 — the sign-based error-quaternion
   PD law this project's `AttitudeController` follows.
8. T. Hamel, R. Mahony, R. Lozano, and J. Ostrowski, "Dynamic Modelling and Configuration
   Stabilization for an X4-Flyer," *IFAC World Congress*, 2002.
9. MuJoCo Physics Engine documentation — <https://mujoco.readthedocs.io/> (simulation
   backend used throughout this project).