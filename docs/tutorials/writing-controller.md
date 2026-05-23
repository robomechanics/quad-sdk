---
title: Writing Your Own Controller
tags:
  - tutorial
  - control
  - extension
---

# Writing Your Own Controller

The `robot_driver` runtime accepts any class that implements the `LegController` abstract interface. Drop your controller in, register it, and select it via launch argument.

## Anatomy of a controller

```cpp
// robot_driver/include/robot_driver/leg_controller.h
class LegController {
 public:
  virtual ~LegController() = default;

  // Populate `cmd` with per-leg joint torques / PD setpoints for the next tick.
  virtual void computeLegCommandArray(
      const quad_msgs::msg::RobotState& state,
      const quad_msgs::msg::LocalPlan&  plan,
      quad_msgs::msg::LegCommandArray&  cmd) = 0;
};
```

Existing reference implementations live in `robot_driver/src/controllers/` — the simplest is `joint_controller.cpp`.

## Step 1 — New source files

Place:

- `robot_driver/include/robot_driver/controllers/<your>_controller.h`
- `robot_driver/src/controllers/<your>_controller.cpp`

Skeleton:

```cpp
// <your>_controller.h
#pragma once
#include "robot_driver/leg_controller.h"

class YourController : public LegController {
 public:
  void computeLegCommandArray(
      const quad_msgs::msg::RobotState& state,
      const quad_msgs::msg::LocalPlan&  plan,
      quad_msgs::msg::LegCommandArray&  cmd) override;
};
```

```cpp
// <your>_controller.cpp
#include "robot_driver/controllers/<your>_controller.h"

void YourController::computeLegCommandArray(
    const quad_msgs::msg::RobotState& state,
    const quad_msgs::msg::LocalPlan&  plan,
    quad_msgs::msg::LegCommandArray&  cmd) {
  // ... fill cmd.leg_commands[0..3] with your control law
}
```

## Step 2 — Build wiring

Add the source to `robot_driver/CMakeLists.txt`:

```cmake
add_library(robot_driver
  src/robot_driver.cpp
  # ...
  src/controllers/<your>_controller.cpp
)
```

Build:

```bash
colcon build --packages-select robot_driver
```

## Step 3 — Tests

Create `robot_driver/test/test_<your>_controller.cpp`. Register in `CMakeLists.txt`:

```cmake
ament_add_gtest(test_<your>_controller
  test/test_<your>_controller.cpp)
target_link_libraries(test_<your>_controller robot_driver)
```

Run:

```bash
colcon test --packages-select robot_driver
colcon test-result --verbose
```

## Step 4 — Register in the runtime

`robot_driver/include/robot_driver/robot_driver.h`:

```cpp
#include "robot_driver/controllers/<your>_controller.h"
```

`robot_driver/src/robot_driver.cpp` (constructor):

```cpp
} else if (controller_id_ == "<your_controller>") {
  leg_controller_ = std::make_shared<YourController>();
}
```

## Step 5 — Run it

Pick your controller via launch arg:

```bash
ros2 launch quad_utils quad_gazebo.py \
  robot_configs:='[{"name":"robot_1","type":"go2","controller":"<your_controller>","init_pose":"-x 0 -y 0 -z 0.5"}]'
```

Then drive the mode topic as usual:

```bash
ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 1" --once  # stand
ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 2" --once  # walk
```

## Patterns to follow

- **Don't allocate in the control loop.** `computeLegCommandArray` is called at 500 Hz on a real-time-ish thread. Allocate buffers in the constructor.
- **Trust `state.foot_in_contact`** — the contact estimator publishes per-leg booleans. Branch on these rather than reinventing contact detection.
- **Use `QuadKD2`** for FK/IK. Don't roll your own — joint sign/offset is per-robot config.
- **Log via `RCLCPP_DEBUG_THROTTLE`** when tuning. Plain `RCLCPP_INFO` at 500 Hz floods the console.

## What about learned policies?

For ONNX-based learned controllers, inherit from `LearnedController` (which itself derives from `LegController`). See [Specialized Controllers](specialized-controllers.md).
