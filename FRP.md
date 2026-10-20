# Quaternion-Based Autonomous Quadrotor Navigation using Adaptive B-Spline Trajectory Generation

---

## Abstract

This project develops a quaternion-based autonomous navigation framework for a quadrotor capable of following smooth, collision-free trajectories generated via **adaptive B-spline curves**. Unlike conventional waypoint navigation, which introduces abrupt velocity and acceleration discontinuities, the proposed framework leverages Cox–de Boor evaluated B-splines to generate continuous ($C^2$) position, velocity, and acceleration references. The quadrotor's attitude is represented entirely using **unit quaternions** to eliminate gimbal-lock singularities inherent in Euler angle representations. A nonlinear quaternion attitude controller with gyroscopic feedforward is paired with a differentially-flat outer-loop position controller. 

The framework features an online adaptive trajectory layer that reactively perturbs B-spline control points based on obstacle feedback from simulated IR ray-cast sensors. The navigation stack is thoroughly validated in dual physics simulation engines: **MuJoCo** (primary high-fidelity physics environment) and **PyBullet** (multi-engine verification), paving the way for future deployment on a LiteWing ESP32-based flight controller.

---

## Repository Architecture & Stage Breakdown

The project repository is structured into six modular stages:

| Stage Folder | Core Module / Purpose | Key Features |
|---|---|---|
| [`stage1_dynamics_attitude`](stage1_dynamics_attitude) | Quadrotor Dynamics & Attitude Control | Rigid-body dynamics, unit quaternion kinematics, Lyapunov nonlinear attitude control with gyroscopic feedforward, Mahony IMU attitude estimation filter, motor mixing for "+"-configuration. |
| [`stage2_trajectory_planning`](stage2_trajectory_planning) | B-Spline Generation & Position Control | Cox–de Boor cubic B-spline evaluation, differential flatness position controller, Shepperd's rotation matrix extraction, `ReactiveReplanner` control-point perturbation. |
| [`stage3_sensing_environment`](stage3_sensing_environment) | Sensing & Environment Simulation | Simulated IR ray-cast distance sensor with range limits and measurement noise, IMU sensor simulation (gyroscope + accelerometer with bias drift). |
| [`stage4_logging_integration`](stage4_logging_integration) | Pipeline Integration & Logging | End-to-end closed-loop navigation pipeline in MuJoCo, CSV telemetry logger (`FlightLogger`), batch plot generation, live 3D visualization. |
| [`stage5_pybullet_sim`](stage5_pybullet_sim) | PyBullet Simulation Engine | Multi-engine physics verification using PyBullet backend (`pybullet_backend.py`), cross-platform trajectory tracking validation. |
| [`stage6_ardupilot_sim`](stage6_ardupilot_sim) | ArduPilot SITL & MAVLink Integration | Offboard GUIDED setpoint streaming, online proximity sensing, reactive B-spline replanning, and EKF telemetry validation. |
| [`models`](models) | XML Physics Models | High-fidelity MuJoCo quadrotor XML definition (`quadrotor.xml`) with physical inertia, motor geometry, and actuator constraints. |

---

## Detailed Step-by-Step Approach

### Phase 1: Mathematical Foundations & Quadrotor Modeling
* Formulate rigid-body 6-DOF quadrotor dynamics using unit quaternion kinematics ($\dot{q} = \frac{1}{2} q \otimes \omega$).
* Set up physical parameters (mass, inertia matrix $I$, drag coefficients, motor thrust/torque bounds).
* Implement motor mixing matrix converting total thrust $T$ and body torques $\tau$ into rotor PWM/force commands for "+"-configuration.

### Phase 2: Attitude Estimation & Filtering
* Model onboard IMU sensor with realistic gaussian noise and gyroscope bias drift.
* Implement Mahony explicit-complementary filter fusing 100 Hz gyroscope integration with accelerometer gravity vector corrections.
* Maintain drift-free roll and pitch state estimation during high-dynamic maneuvers.

### Phase 3: Smooth B-Spline Trajectory Generation
* Construct clamped, uniform cubic ($p=3$) B-splines over specified waypoints.
* Utilize Cox–de Boor recursive algorithm for accurate position $C(u)$ evaluation.
* Derivatively form continuous velocity $C'(u)$ and acceleration $C''(u)$ profiles analytically without discrete finite-difference noise.

### Phase 4: Outer-Loop Position Controller & Reference Extraction
* Design outer-loop PD position controller with gravity compensation ($m g \hat{e}_3$) and feedforward acceleration ($m \ddot{\zeta}_d$).
* Extract desired total thrust magnitude $T_d = \|F_d\|$ and body orientation matrix $R_d = [x_b\ y_b\ z_b]$.
* Convert orientation matrix to desired quaternion $q_d$ via Shepperd's algorithm.

### Phase 5: Quaternion Attitude Controller
* Compute quaternion tracking error $q_e = q_{des}^{-1} \otimes q_c$.
* Design nonlinear torque control law $\tau = \Omega \times (I\Omega) - K_{p,att} \vec{q}_e - K_{d,att} \Omega_e$.
* Include gyroscopic feedforward term $\Omega \times (I\Omega)$ to cancel Coriolis rotational coupling.

### Phase 6: Simulated IR Ray-Cast Sensing & Obstacle Detection
* Implement ray-casting against surrounding obstacles in the simulation environment.
* Compute real-time distance and clearance vectors from quadrotor to obstacle surfaces.
* Inject realistic sensor noise and detection thresholds to simulate physical IR distance sensors.

### Phase 7: Adaptive B-Spline Obstacle Avoidance (Online Replanning)
* Evaluate local control points of the active B-spline trajectory against safety margin $d_{\text{safe}}$.
* Dynamically perturb control points away from obstacles along sensed escape directions (`ReactiveReplanner`).
* Preserve global spline smoothness and $C^2$ continuity while achieving local obstacle avoidance without recomputing the full path.

### Phase 8: MuJoCo Physics Engine Simulation & Integration
* Integrate dynamics, sensing, trajectory planning, position control, and attitude control into a unified closed-loop pipeline (`stage4_logging_integration`).
* Run full-mission simulations in **MuJoCo** with XML scene modeling (`models/quadrotor.xml`).
* Log full telemetry (position error, quaternion deviation, rotor thrusts, clearance distance) using `FlightLogger`.

### Phase 9: PyBullet Multi-Engine Verification
* Deploy identical controller and trajectory modules into **PyBullet** backend (`stage5_pybullet_sim`).
* Conduct cross-engine benchmark testing to verify robustness across different contact dynamics and physics solvers.

### Phase 10: Performance Evaluation & Metrics Analysis
* Quantify tracking performance (mean, max, and steady-state error).
* Analyze attitude convergence speed and quaternion tracking error ($< 0.25^\circ$ hover precision).
* Verify minimum clearance maintenance ($> d_{\text{safe}}$) during active online replanning triggers.

---

## Expected Outcomes & Deliverables

* Singular-free, unit-quaternion nonlinear attitude control stack.
* Smooth cubic B-spline trajectory generator with analytical velocity/acceleration profiling.
* Online adaptive trajectory replanning using local control-point perturbation.
* Simulated IR ray-cast obstacle detection pipeline.
* Comprehensive **MuJoCo** rigid-body simulation with interactive 3D visualization.
* **PyBullet** multi-engine verification pipeline validating cross-backend stability.
* Complete telemetry logging and visualization scripts (`main.py` entry point generating outputs for all stages).
* Hardware-ready Python implementation structured for future porting to LiteWing ESP32 flight controller.
