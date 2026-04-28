# Local Planner

## Overview

This package determines the contact timing, foothold locations, and ground reaction forces (GRFs) needed to execute a given global body plan. Local body and footstep plans are computed separately and heuristically combined. The local body plan is produced by the `nmpc_controller` (invoked as a library) which finds GRFs that best track a nominal body trajectory; the local footstep planner then uses this body plan to produce a contact schedule and select discrete foothold positions using dynamic and kinematic heuristics similar to Raibert's.

### License

The source code is released under a [MIT License](../LICENSE).

**Authors:** Joe Norby, Yanhao Yang

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

**Maintainers:** Yanhao Yang (yanhaoy@andrew.cmu.edu), Alex Stutt (astutt@andrew.cmu.edu)

Tested under [ROS2] Jazzy on Ubuntu 24.04. This is research code; expect frequent changes and no fitness for any particular purpose.

## Build

```bash
colcon build --packages-select local_planner
```

Requires `nmpc_controller` (built as a library dependency), `quad_utils`, `quad_msgs`, and `grid_map_core`.

## Usage

Launched as part of the planning stack:

```bash
ros2 launch quad_utils quad_plan.py
```

Twist input can be provided via `cmd_vel` (e.g. from `teleop_twist_keyboard` or `quad_perf_tests/cmd_vel_publisher_node`).

## Config

* **`config/local_planner.yaml`** — local body + footstep planner hyperparameters.
* **`config/local_planner_topics.yaml`** — topic remappings.

## Nodes

### local_planner_node

Runs the local body and footstep planners on a shared update loop.

**Local body planner:** uses the NMPC (see `nmpc_controller/`) to produce GRFs and body states that track a nominal reference trajectory derived from the global plan and `cmd_vel`.

**Local footstep planner:** consumes the body plan, computes a contact schedule, and picks discrete foothold positions using Raibert-style heuristics and a traversability-aware cost layer over the terrain map.

#### Subscribed Topics

* **`terrain_map`** ([grid_map_msgs/GridMap]) — 2.5D height map with terrain and traversability layers.
* **`global_plan`** ([quad_msgs/RobotPlan]) — global plan from `global_body_planner`.
* **`state/ground_truth`** ([quad_msgs/RobotState]) — current robot state.
* **`cmd_vel`** ([geometry_msgs/Twist]) — commanded body twist.

#### Published Topics

* **`local_plan`** ([quad_msgs/LocalPlan]) — interpolated local body plan (states, GRFs, contact schedule) over the horizon.
* **`foot_plan_discrete`** ([quad_msgs/MultiFootPlanDiscrete]) — discrete foothold positions.
* **`foot_plan_continuous`** ([quad_msgs/MultiFootPlanContinuous]) — continuous foot trajectories through the horizon.

#### Parameters

**`local_planner`:**

* `update_rate` (double, default: `333.0`) — loop rate in Hz.
* `timestep` (double, default: `0.03`) — timestep in s.
* `horizon_length` (int, default: `26`) — planning horizon in timesteps.
* `cmd_vel_filter_const` (double, default: `0.10`) — low-pass filter constant for `cmd_vel`.
* `cmd_vel_scale` (double, default: `1.0`) — multiplicative scale on `cmd_vel`.
* `last_cmd_vel_msg_time_max` (double, default: `2.0`) — timeout before assuming no input (s).
* `stand_vel_threshold` (double, default: `0.1`) — body velocity below which stand mode is permitted.
* `stand_cmd_vel_threshold` (double, default: `0.1`) — commanded velocity below which stand mode is permitted.
* `stand_pos_error_threshold` (double, default: `0.05`) — position error (from foot centroid) below which stand mode is permitted.

**`local_footstep_planner`:**

* `grf_weight` (double, default: `0.45`) — weight on GRF-direction term in foothold cost.
* `standing_error_threshold` (double, default: `0.03`) — m.
* `foothold_obj_threshold` (double, default: `0.6`) — traversability threshold below which footholds are rejected. Must match the grid-map filter's `traversability_mask_lower_threshold`.
* `obj_fun_layer` (string, default: `traversability`) — grid-map layer used for the foothold objective.

Plus NMPC parameters (see `nmpc_controller/README.md`) loaded under the same node namespace.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ROS2]: https://docs.ros.org/en/jazzy/
[quad_msgs/RobotState]: ../quad_msgs/msg/RobotState.msg
[quad_msgs/RobotPlan]: ../quad_msgs/msg/RobotPlan.msg
[quad_msgs/LocalPlan]: ../quad_msgs/msg/LocalPlan.msg
[quad_msgs/MultiFootPlanDiscrete]: ../quad_msgs/msg/MultiFootPlanDiscrete.msg
[quad_msgs/MultiFootPlanContinuous]: ../quad_msgs/msg/MultiFootPlanContinuous.msg
[geometry_msgs/Twist]: https://docs.ros.org/en/jazzy/p/geometry_msgs/
[grid_map_msgs/GridMap]: https://github.com/ANYbotics/grid_map
