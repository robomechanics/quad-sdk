# quad_controllers

The `ros2_control` controllers and selectable leg-control laws for the
switchable-controller ("Option B") quad-sdk stack. Four pluginlib controllers
realize the robot's behavior modes; a `LegControlController` delegates locomotion
to a runtime-selected control **law**. Faithful ports of `robot_driver`'s
control-mode and `LegController` logic.

> See the [Control Stack architecture page](https://github.com/robomechanics/quad-sdk/blob/main/docs/architecture/control-stack.md)
> for how the `controller_manager` loads, switches, and wires these into the full
> sim/hardware pipeline. This README is the package-level reference.

## Contents

### Controllers (pluginlib, registered in `quad_controllers_plugins.xml`)

All four derive `QuadControllerBase` (`quad_controller_base.hpp/.cpp`).

| Class | Mode | Role |
|---|---|---|
| `quad_controllers/LegControlController` | READY | Runs a selectable leg-control law; falls back to a stand pose when the law fails or no state has arrived. |
| `quad_controllers/PoseController` | SIT | Holds every leg at a fixed joint pose with PD gains. |
| `quad_controllers/SafetyController` | SAFETY | Holds every joint at position 0 with safety PD gains (damping-only). |
| `quad_controllers/TransitionController` | SIT_TO_READY & READY_TO_SIT | Linearly interpolates every joint from `from_angles` to `to_angles` over `duration` s; interpolation clock starts on activation. |

`QuadControllerBase` factors out the shared plumbing:

- **Interface configuration** — `command_interface_configuration()` claims, per
  joint, just `<joint>/effort` in **effort mode** or
  `position/velocity/kp/kd/effort` in **motor mode**;
  `state_interface_configuration()` claims `<joint>/position` and `/velocity`.
- **`writeToInterfaces()`** has two modes keyed on `interface_mode`:
    - **effort** — collapses the per-joint command into a single torque,
      `tau = torque_ff + kp*(pos_setpoint - q) + kd*(vel_setpoint - dq)`, clamped
      to `motor_limits.torque` (per joint-type: abad/hip/knee), then written to
      `<joint>/effort`.
    - **motor** — writes `pos_setpoint`, `vel_setpoint`, `kp`, `kd`, `torque_ff`
      straight to the `position/velocity/kp/kd/effort` command interfaces so the
      robot's onboard motor PD closes the loop.

The base `update()` resizes a `LegCommandArray`, calls the derived
`computeCommand()`, writes the result to the hardware interfaces, and echoes it
on `control/joint_command` (observable only — torque goes to the interfaces, not
the topic).

### Leg-control laws (`src/laws/`, `include/quad_controllers/laws/`)

Plain C++ classes — **not** pluginlib plugins. They are verbatim ports of
`robot_driver`'s `LegController` hierarchy and all derive
`quad_controllers::LegController` (`laws/leg_controller.hpp`).

| Law (`controller` value) | Source |
|---|---|
| `inverse_dynamics` (default) | `inverse_dynamics_controller.cpp` |
| `grf_pid` | `grf_pid_controller.cpp` |
| `joint` | `joint_controller.cpp` |
| `underbrush` | `underbrush_inverse_dynamics.cpp` |
| `inertia_estimation` | `inertia_estimation_controller.cpp` |
| `learned` | `learned_policy.cpp` — gated by `#ifdef HAS_ONNXRUNTIME` (compiled only when ONNX Runtime is found) |

The law is selected at runtime from the `controller` string parameter by
`LegControlController::initLegController()`. `LegControlController` builds a
`QuadKD2` on its own `LifecycleNode`, and every control cycle **primes** it with
`quad_utils::updateDynamics(*quadKD_, *last_robot_state_msg_)` before running the
law (exactly `robot_driver::testDynamics()`); if the law returns `false` it
falls back to the stand pose (`stand_joint_angles` with `stand_kp`/`stand_kd`).

!!! info "Why laws are not plugins"
    Only the four mode controllers are pluginlib classes (loaded by the
    `controller_manager`). The laws are an internal detail of
    `LegControlController`, selected by a string param — adding one is a code
    change to the controller, not a plugin registration. See the
    [Writing a control law](https://github.com/robomechanics/quad-sdk/blob/main/docs/tutorials/writing-controller.md)
    tutorial.

## Key parameters

`LegControlController` (locomotion):

* `controller` (string, default `inverse_dynamics`) — selects the leg-control law.
* `robot_state_topic` (string, default `state/ground_truth`) — state feedback for the law.
* `stance_kp/kd`, `swing_kp/kd`, `swing_kp_cart/kd_cart`, `stand_kp/kd`, `stand_joint_angles` — gains/pose (from `<robot>.yaml`).
* `policy_inference_rate`, `cmd_vel_filter_const`, `cmd_vel_scale`, `model_path` — learned-policy only.

`QuadControllerBase` (all controllers):

* `interface_mode` (string, `effort` | `motor`) — from the context overlay.
* `joints` (string array) — the 12 numeric joint names.
* `motor_limits.torque` (double array) — effort-mode clamp (legacy fallback: `torque_limits`).
* `robot_namespace` (string).

`PoseController`: `joint_angles`, `kp`, `kd`. `SafetyController`: `kp`, `kd`.
`TransitionController`: `from_angles`, `to_angles`, `duration`, `kp`, `kd`.

### Subscribed topics (`LegControlController`)

* `local_plan` ([quad_msgs/RobotPlan]) — latest MPC plan, forwarded to the law.
* `state/ground_truth` ([quad_msgs/RobotState]) — control state (topic name from `robot_state_topic`).
* `control/single_joint_command` ([geometry_msgs/Vector3]) — single-joint override (only the `joint` law).
* `cmd_vel` ([geometry_msgs/Twist]) — filtered/scaled command velocity (used by the learned policy).
* `imu` ([sensor_msgs/Imu]) — learned-policy input.

### Published topics (all controllers)

* `control/joint_command` ([quad_msgs/LegCommandArray]) — echo of the command written to the interfaces.

## Config

Parameters are split by *what varies* and merged at launch (later overrides earlier):

| File | Scope | Holds |
|---|---|---|
| `config/controllers.yaml` | node (every robot, sim+hw) | `controller_manager` `update_rate` + controller TYPE map, `imu_sensor_broadcaster` wiring, the `mode_supervisor` structure (controller-name mappings + timings), fixed `robot_state_topic`. |
| `config/sim.yaml` | context overlay | `interface_mode: effort`, `is_hardware: false`. |
| `config/hardware.yaml` | context overlay | `interface_mode: motor`, `is_hardware: true`. |

Robot-specific gains/poses/joints/kinematics live in `quad_utils/config/<robot>.yaml`
(not here). All keys are `/**/`-wildcarded so the config composes cleanly under
any robot namespace. See the
[Control Stack page](https://github.com/robomechanics/quad-sdk/blob/main/docs/architecture/control-stack.md#configuration-layering)
for the full layering story.

## How it's launched

Not run standalone — the controllers are spawned against a `controller_manager`:

```bash
# Simulation (control_stack:=ros2_control selects this stack over robot_driver)
ros2 launch quad_utils robot_bringup.py control_stack:=ros2_control controller:=inverse_dynamics

# Hardware (Go2)
ros2 launch quad_hardware go2_bringup.launch.py
```

In sim, `robot_bringup.py` deep-merges `controllers.yaml + <robot>.yaml + sim.yaml`
into the one file the Gazebo plugin loads. On hardware,
`go2_bringup.launch.py` hands the three files to a standalone `ros2_control_node`.
`sit_controller` is spawned **active** (robot holds sit from boot); the other four
controllers load **inactive** and the `quad_supervisor` switches them per mode.

## Build

```bash
colcon build --packages-select quad_controllers
```

Optional dependency: **ONNX Runtime** — when found, `HAS_ONNXRUNTIME` is defined
and the `learned` law is compiled.

## License

MIT. **Maintainer:** David Ologan (ologandavid@gmail.com),
[Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University.

[quad_msgs/RobotPlan]: ../quad_msgs/msg/RobotPlan.msg
[quad_msgs/RobotState]: ../quad_msgs/msg/RobotState.msg
[quad_msgs/LegCommandArray]: ../quad_msgs/msg/LegCommandArray.msg
[geometry_msgs/Vector3]: https://docs.ros.org/en/jazzy/p/geometry_msgs/
[geometry_msgs/Twist]: https://docs.ros.org/en/jazzy/p/geometry_msgs/
[sensor_msgs/Imu]: https://docs.ros.org/en/jazzy/p/sensor_msgs/
