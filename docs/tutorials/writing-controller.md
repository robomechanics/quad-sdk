---
title: Writing a Control Law
tags:
  - tutorial
  - control
  - extension
  - ros2
---

# Writing a Control Law

On the `ros2_control` stack, the READY-mode controller
(`quad_controllers/LegControlController`) delegates locomotion to a selectable
leg-control **law**. The shipped laws (`inverse_dynamics`, `grf_pid`, `joint`,
`underbrush`, `inertia_estimation`, `learned`) are verbatim ports of
`robot_driver`'s `LegController` hierarchy. This tutorial walks through adding a
new one and selecting it at launch.

!!! info "Laws are not pluginlib plugins"
    Only the four mode controllers (`LegControlController`, `PoseController`,
    `SafetyController`, `TransitionController`) are pluginlib classes loaded by
    the `controller_manager`. Laws are plain C++ classes selected by a **string
    param** (`controller`) inside `LegControlController` — adding one is a code
    change, not a plugin registration. To add a whole new *mode*, see
    [Adding a new controller (mode)](#adding-a-new-controller-mode) at the end.

See the [Control Stack architecture](../architecture/control-stack.md) for how
the controller, the law, and `QuadKD2` fit together.

## The `LegController` interface

A law derives `quad_controllers::LegController`
(`quad_controllers/include/quad_controllers/laws/leg_controller.hpp`). The one
method you must implement:

```cpp
// Return false to signal "nothing useful to output" -- LegControlController
// then falls back to the stand pose. The local plan arrives via
// updateLocalPlanMsg() (the base caches it as last_local_plan_msg_).
virtual bool computeLegCommandArray(
    const quad_msgs::msg::RobotState& robot_state_msg,
    quad_msgs::msg::LegCommandArray& leg_command_array_msg,
    quad_msgs::msg::GRFArray& grf_array_msg) = 0;
```

The constructor signature every law shares:

```cpp
YourLaw(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
        const std::string& robot_ns,
        std::shared_ptr<quad_utils::QuadKD2> quadKD);
```

The base class hands you `node_`, `robot_ns_`, `quadKD_`, the stance/swing PD
gains (set via `init(...)`), and `last_local_plan_msg_`.

!!! tip "QuadKD2 is already primed for you"
    `LegControlController` calls
    `quad_utils::updateDynamics(*quadKD_, robot_state)` **before** invoking your
    law every cycle, so `quadKD_->computeInverseDynamics(...)` /
    `getJacobianBodyAngVel(...)` see the current configuration. Don't re-prime it.

## Step 1 — New source files

Create the header and source, modeling on the simplest existing law
(`joint_controller`):

- `quad_controllers/include/quad_controllers/laws/<name>.hpp`
- `quad_controllers/src/laws/<name>.cpp`

```cpp
// <name>.hpp
#ifndef QUAD_CONTROLLERS__LAWS__YOUR_LAW_HPP_
#define QUAD_CONTROLLERS__LAWS__YOUR_LAW_HPP_

#include "quad_controllers/laws/leg_controller.hpp"

namespace quad_controllers {

class YourLaw : public LegController {
 public:
  YourLaw(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
          const std::string& robot_ns,
          std::shared_ptr<quad_utils::QuadKD2> quadKD);

  bool computeLegCommandArray(
      const quad_msgs::msg::RobotState& robot_state_msg,
      quad_msgs::msg::LegCommandArray& leg_command_array_msg,
      quad_msgs::msg::GRFArray& grf_array_msg) override;
};

}  // namespace quad_controllers

#endif  // QUAD_CONTROLLERS__LAWS__YOUR_LAW_HPP_
```

```cpp
// <name>.cpp
#include "quad_controllers/laws/<name>.hpp"

namespace quad_controllers {

YourLaw::YourLaw(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                 const std::string& robot_ns,
                 std::shared_ptr<quad_utils::QuadKD2> quadKD)
    : LegController(node, robot_ns, quadKD) {
  // Allocate buffers here, NOT in computeLegCommandArray (runs at 500 Hz).
}

bool YourLaw::computeLegCommandArray(
    const quad_msgs::msg::RobotState& robot_state_msg,
    quad_msgs::msg::LegCommandArray& leg_command_array_msg,
    quad_msgs::msg::GRFArray& grf_array_msg) {
  leg_command_array_msg.leg_commands.resize(num_feet_);
  for (int i = 0; i < num_feet_; ++i) {
    leg_command_array_msg.leg_commands.at(i).motor_commands.resize(3);
    for (int j = 0; j < 3; ++j) {
      auto& mc = leg_command_array_msg.leg_commands.at(i).motor_commands.at(j);
      // Fill mc.pos_setpoint / vel_setpoint / torque_ff / kp / kd from your law.
    }
  }
  return true;  // false -> LegControlController falls back to the stand pose
}

}  // namespace quad_controllers
```

## Step 2 — Build wiring

Add the source to the `QUAD_CONTROLLERS_SOURCES` list in
`quad_controllers/CMakeLists.txt`:

```cmake
set(QUAD_CONTROLLERS_SOURCES
  src/quad_controller_base.cpp
  src/leg_control_controller.cpp
  # ... mode controllers ...
  src/laws/leg_controller.cpp
  src/laws/inverse_dynamics_controller.cpp
  # ...
  src/laws/<name>.cpp)          # <-- add here
```

## Step 3 — Register the law

In `quad_controllers/src/leg_control_controller.cpp`, include the header and add
an `else if` branch to `initLegController()`:

```cpp
#include "quad_controllers/laws/<name>.hpp"   // top of file
```

```cpp
// inside LegControlController::initLegController()
} else if (controller_id_ == "<name>") {
  leg_controller_ = std::make_shared<YourLaw>(node, robot_ns_, quadKD_);
}
```

The gains are applied right after via `leg_controller_->init(...)` (handled for
you for every non-`learned` law).

## Step 4 — Allow selecting it at launch

Add `"<name>"` to the `valid_laws` set in
`quad_utils/launch/robot_bringup.py`:

```python
valid_laws = {'inverse_dynamics', 'grf_pid', 'joint', 'underbrush',
              'inertia_estimation', 'learned', '<name>'}
```

That set gates the `controller:=<name>` launch arg, which the launch turns into a
`/**/locomotion_controller: {controller: <name>}` override. You can also set it
permanently in `quad_utils/config/<robot>.yaml`:

```yaml
/**/locomotion_controller:
  ros__parameters:
    controller: <name>
```

## Step 5 — Build and run

```bash
colcon build --packages-select quad_controllers
source install/setup.bash

# Simulation (control_stack:=ros2_control selects this stack)
ros2 launch quad_utils robot_bringup.py control_stack:=ros2_control controller:=<name>
```

Then drive the mode topic:

```bash
ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 1" --once  # READY (runs your law)
ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 0" --once  # SIT
```

## Patterns to follow

- **Don't allocate in the control loop.** `computeLegCommandArray` runs at 500 Hz. Allocate buffers in the constructor.
- **Use `QuadKD2`** for FK/IK/dynamics — it is primed for you each cycle. Don't roll your own; joint sign/offset is per-robot config.
- **Trust `robot_state_msg.feet.feet[i].contact`** — branch on the published contact rather than reinventing detection.
- **Return `false`** when you can't produce a safe command, so the controller falls back to the stand pose instead of writing garbage torque.
- **Log via `RCLCPP_*_THROTTLE`** when tuning; plain `INFO` at 500 Hz floods the console.

## Adding a new controller (mode)

A whole new behavior *mode* (beyond SIT / READY / SAFETY / transitions) is a new
`ros2_control` controller, not a law:

1. Derive `quad_controllers::QuadControllerBase` and implement
   `computeCommand(LegCommandArray&)` (the base handles interface configuration
   and the effort/motor write). See `PoseController` for the minimal example.
2. Register it in `quad_controllers/quad_controllers_plugins.xml` and export it
   with `PLUGINLIB_EXPORT_CLASS(...)`.
3. Add it to the controller TYPE map in
   `quad_controllers/config/controllers.yaml` and to the `mode_supervisor`'s
   controller-name mappings, then teach the supervisor's FSM
   (`quad_supervisor/src/mode_supervisor.cpp`) when to switch to it.

[:octicons-arrow-right-24: Adding a robot](adding-a-robot.md) ·
[:octicons-arrow-right-24: Control Stack architecture](../architecture/control-stack.md)
