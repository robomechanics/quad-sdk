# Robot Driver

## Overview

This package is the hardware abstraction and low-level control layer of Quad-SDK. It owns the main control loop: ingesting the most recent local plan, running state estimation, computing joint torques via the selected leg controller, and delivering those commands to simulation (Gazebo, MuJoCo) or hardware (Unitree Go1/Go2/B2 via `unitree_sdk2`, Ghost Robotics platforms via `mblink`). It also hosts the behavior state machine that transitions between safety, sit, stand, and stance/swing modes.

### License

The source code is released under a [MIT License](../LICENSE).

**Authors:** Joe Norby and Ardalan Tajbakhsh

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

**Maintainer:** David Ologan (dologan@andrew.cmu.edu)

Tested under [ROS2] Jazzy on Ubuntu 24.04. This is research code; expect frequent changes and no fitness for any particular purpose.

## Build

```bash
colcon build --packages-select robot_driver
```

Optional dependency: **ONNX Runtime** — if available, the `learned_policy` controller is compiled for running learned locomotion policies.

## Unit Tests

```bash
colcon test --packages-select robot_driver
colcon test-result --verbose
```

## Usage

Launched as part of robot bringup — not run standalone:

```bash
ros2 launch quad_utils quad_gazebo.py        # simulation
ros2 launch quad_utils robot_bringup.py      # hardware or sim, configurable
```

## Architecture

The driver is a single node (`robot_driver_node`) composed of three pluggable abstractions:

1. **`HardwareInterface`** — talks to the physical motors. Implementations:
   - `spirit_interface` — Ghost Robotics Spirit via `mblink`.
   - `go2_interface` — Unitree Go2 via `unitree_sdk2`.
   - `go2w_interface` — Unitree Go2-W (wheeled) via `unitree_sdk2`; extends `go2_interface` with wheel-motor commands at LowCmd indices 12-15.
2. **`StateEstimator`** — fuses IMU, joint encoders, and optional mocap. Implementations:
   - `comp_filter_estimator` — complementary filter fusing IMU with mocap (or simulator pose).
   - `ekf_estimator` — contact-aided extended Kalman filter; calls `quad_utils::QuadKD2` (Pinocchio) for forward kinematics and Jacobians.
3. **`LegController`** — produces joint torques from the local plan. Implementations:
   - `inverse_dynamics_controller` — default; inverts the commanded GRFs through leg dynamics.
   - `grf_pid_controller` — PID tracking on GRFs.
   - `inertia_estimation_controller` — system-identification aid.
   - `joint_controller` — direct joint-space PD tracking.
   - `underbrush_inverse_dynamics` — swing-leg variant for the Underbrush platform.
   - `learned_policy` — ONNX-policy controller running at a separate `policy_inference_rate`.

## Config

* **`config/robot_driver.yaml`** — rates, filter coefficients, EKF noise/bias, learned-policy settings.
* **`config/robot_driver_topics.yaml`** — topic remappings.

## Nodes

### robot_driver_node

The main control-loop node. Runs at `update_rate` Hz (default 500 Hz). Each iteration it reads sensors, runs the state estimator, computes commands via the leg controller, and writes to the hardware interface. It also ticks the control-mode state machine (safety ↔ sit ↔ stand ↔ stance/swing) based on the received local plan, heartbeats, and control-mode commands.

#### Subscribed Topics

* **`local_plan`** ([quad_msgs/LocalPlan]) — latest MPC plan.
* **`control/mode`** ([std_msgs/UInt8]) — commanded behavior (`0` safety, `1` sit, `2` stand, `3` stance).
* **`control/single_joint_command`** ([quad_msgs/MotorCommand]) — single-joint override (for hardware bring-up).
* **`control/leg_override`** ([quad_msgs/LegCommandArray]) — direct per-leg command override.
* **`control/restart_flag`** ([std_msgs/Bool]) — flag to reset the controller state.
* **`heartbeat/remote`** ([std_msgs/Header]) — remote operator heartbeat; loss triggers safety mode.
* **`mocap_node/quad/pose`** ([geometry_msgs/PoseStamped]) — optional mocap pose input.

#### Published Topics

* **`control/joint_command`** ([quad_msgs/LegCommandArray]) — motor commands sent to hardware/sim.
* **`control/grfs`** ([quad_msgs/GRFArray]) — commanded ground reaction forces.
* **`state/estimate`** ([quad_msgs/RobotState]) — fused state estimate.
* **`state/grfs`** ([quad_msgs/GRFArray]) — contact-based GRF estimate.
* **`heartbeat/robot`** ([std_msgs/Header]) — driver heartbeat.

#### Key Parameters

**Rates and timeouts:**

* `robot_driver.update_rate` (double, default: `500.0`) — control loop in Hz.
* `robot_driver.publish_rate` (double, default: `500.0`) — command publish rate in Hz.
* `robot_driver.mocap_rate` (double, default: `360.0`) — mocap frame rate (must match source or velocities mis-scale).
* `robot_driver.mocap_dropout_threshold` (double, default: `0.035`) — s.
* `robot_driver.filter_time_constant` (double, default: `0.01`) — s.
* `robot_driver.input_timeout` (double, default: `0.2`) — s without a plan before safety mode.
* `robot_driver.state_timeout` (double, default: `0.1`) — s.
* `robot_driver.heartbeat_timeout` (double, default: `0.2`) — s.

**Complementary filter** — state-space coefficients `low_pass_{a,b,c,d}` and `high_pass_{a,b,c,d}`, obtained from `c2d(1/s·(1-G(s)))` and `c2d(s·G(s))` for a 2nd-order low-pass `G(s)`.

**EKF noise / bias** (when `ekf_estimator` is selected): `na`, `nv`, `ng`, `ba`, `bg`, `nf`, `nfk`, `ne`, `nfh`, `P0`, `contact_w`, `thresh_out`, `foot_radius`, and per-axis `bias_{x,y,z,r,p,w}`.

**Learned policy:**

* `robot_driver.model_path` (string) — absolute path to the ONNX policy file.
* `robot_driver.policy_inference_rate` (double, default: `50.0`) — Hz.
* `robot_driver.cmd_vel_filter_const`, `cmd_vel_scale` — applied to policy inputs.

**Mode gains** — `sit_kp/kd`, `stand_kp/kd`, `stance_kp/kd`, `swing_kp/kd`, `swing_kp_cart/kd_cart`, `safety_kp/kd` (all 3-vectors).

**Joint targets** — `stand_joint_angles` (default `[0.0, 0.76, 1.52]`), `sit_joint_angles` (default `[0.0, 0.0, 0.0]`).

**Underbrush swing** — `underbrush_swing.*` parameters for the Underbrush controller variant.

See `config/robot_driver.yaml` for the full set.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ROS2]: https://docs.ros.org/en/jazzy/
[quad_msgs/LocalPlan]: ../quad_msgs/msg/LocalPlan.msg
[quad_msgs/LegCommandArray]: ../quad_msgs/msg/LegCommandArray.msg
[quad_msgs/MotorCommand]: ../quad_msgs/msg/MotorCommand.msg
[quad_msgs/GRFArray]: ../quad_msgs/msg/GRFArray.msg
[quad_msgs/RobotState]: ../quad_msgs/msg/RobotState.msg
[std_msgs/UInt8]: https://docs.ros.org/en/jazzy/p/std_msgs/
[std_msgs/Bool]: https://docs.ros.org/en/jazzy/p/std_msgs/
[std_msgs/Header]: https://docs.ros.org/en/jazzy/p/std_msgs/
[geometry_msgs/PoseStamped]: https://docs.ros.org/en/jazzy/p/geometry_msgs/
