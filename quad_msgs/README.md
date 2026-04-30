# Quad Msgs

## Overview

This package defines the ROS2 message interfaces used across Quad-SDK for robot state, motor commands, plans, and estimator outputs. It depends on `std_msgs`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, and `builtin_interfaces`.

### License

The source code is released under a [MIT License](../LICENSE).

**Author:** Joe Norby

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

**Maintainer:** Qishun Yu (qishuny@andrew.cmu.edu)

Tested under [ROS2] Jazzy on Ubuntu 24.04. This is research code; expect frequent changes and no fitness for any particular purpose.

## Build

```bash
colcon build --packages-select quad_msgs
```

## Messages

### State

| Message | Description |
|---|---|
| `RobotState.msg` | Full robot state: `BodyState` + `sensor_msgs/JointState` + `MultiFootState`. |
| `BodyState.msg` | Body pose (`geometry_msgs/Pose`) and twist (`geometry_msgs/Twist`). |
| `MultiFootState.msg` | Per-foot state array. Order: `0` FL, `1` BL, `2` FR, `3` BR. |
| `FootState.msg` | Position, velocity, and acceleration of a single foot. |
| `ContactMode.msg` | Per-leg contact states. Order: FL, BL, FR, BR. |
| `LegContactMode.msg` | Boolean contact flag + contact force (`geometry_msgs/Vector3`) for one leg. |
| `GRFArray.msg` | Array of ground reaction force vectors, application points, and per-leg contact flags. |

### Commands

| Message | Description |
|---|---|
| `LegCommandArray.msg` | Array of `LegCommand` — one per leg (FL, BL, FR, BR). |
| `LegCommand.msg` | Per-joint motor commands (`MotorCommand[3]`) for a leg (Abd, Hip, Knee). |
| `MotorCommand.msg` | Desired position, velocity, feedforward torque, and PD gains for one joint. |

### Plans

| Message | Description |
|---|---|
| `RobotPlan.msg` | Interpolated robot plan as an array of odometry messages. Used by global and local planners. |
| `BodyPlan.msg` | Body-only plan as an array of odometry messages. |
| `LocalPlan.msg` | Local plan: `RobotState[]` + `GRFArray[]` + plan indices + reference to the global plan timestamp. |
| `MultiFootPlanContinuous.msg` | Continuous foot-trajectory plan as `MultiFootState[]`. |
| `MultiFootPlanDiscrete.msg` | Discrete foothold plan as an array of per-foot `FootPlanDiscrete`. |
| `FootPlanDiscrete.msg` | Sequence of `FootState` for one foot. |
| `RobotPlanDiagnostics.msg` | Diagnostics (timing, success flags) for the local plan. |

### Estimators

| Message | Description |
|---|---|
| `BodyForceEstimate.msg` | Estimated external joint torques for all 12 joints (see `body_force_estimator`). |

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ROS2]: https://docs.ros.org/en/jazzy/
