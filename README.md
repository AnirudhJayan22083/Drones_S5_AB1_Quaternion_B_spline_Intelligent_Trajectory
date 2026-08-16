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