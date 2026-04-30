# Global Body Planner

## Overview

This package implements global planning algorithms for agile quadrupedal navigation. It produces point-to-point plans that guide the robot from its current state to a goal given a 2.5D terrain map. The primary algorithm is the Global Body Planner for Legged Robots (GBP-L) — an RRT-Connect-based planner that uses mixed motion primitives (including flight phases) to produce long-horizon plans. See the [paper] for algorithmic details.

The planner also exposes a `plan_with_constraints` service used by the [`conflict_based_search`](../conflict_based_search) package for multi-robot replanning. Each request supplies a list of time-windowed pose constraints (forbidden body OBBs during specific intervals) and a `warm_start` flag — when warm-started, the cached RRT-Connect trees from the previous solve are lazy-pruned of vertices that violate the new constraints and the search resumes instead of rebuilding from scratch. Time-windowed constraints are gated by per-vertex absolute time, so a constraint active only during `[t_start, t_end]` doesn't behave as a permanent obstacle.

### License

The source code is released under a [MIT License](../LICENSE).

**Author:** Joe Norby

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

**Maintainers:** Jiming Ren (jimingre@andrew.cmu.edu), Joe Norby (jnorby@andrew.cmu.edu)

Tested under [ROS2] Jazzy on Ubuntu 24.04. This is research code; expect frequent changes and no fitness for any particular purpose.

### Publications

If you use this work in an academic context, please cite:

* J. Norby and A. M. Johnson, "Fast global motion planning for dynamic legged robots," in *2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2020, pp. 3829–3836. ([paper])

```bibtex
@inproceedings{Norby2020,
  title={Fast global motion planning for dynamic legged robots},
  author={Norby, Joseph and Johnson, Aaron M},
  booktitle={2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={3829--3836},
  year={2020},
  organization={IEEE}
}
```

## Build

```bash
colcon build --packages-select global_body_planner
```

## Unit Tests

```bash
colcon test --packages-select global_body_planner
colcon test-result --verbose
```

## Usage

The planner is launched as part of the planning stack:

```bash
ros2 launch quad_utils quad_plan.py
```

Goals can be set interactively by publishing on `/clicked_point` from RViz (the launch file handles the remap), or via the `goal_state` parameter.

## Config

* **`config/global_body_planner.yaml`** — planning hyperparameters, dynamics bounds, friction.
* **`config/global_body_planner_topics.yaml`** — topic remappings.

## Nodes

### global_body_planner_node

Publishes global plans that guide the local planner toward the goal. The node alternates between two states:

- **`RESET`** — entered on receiving a new goal; replans from the current state for `startup_delay` seconds before publishing.
- **`REFINE`** — continuously replans from a point `max_planning_time` seconds into the future along the current plan.

A new plan is accepted if it (a) reaches the goal and is shorter than the previous plan, (b) gets closer to the goal than the previous plan, or (c) a new goal has been set since the previous plan.

#### Subscribed Topics

* **`start_state`** ([quad_msgs/RobotState]) — current robot state (typically remapped to `state/ground_truth`).
* **`goal_state`** ([geometry_msgs/PointStamped]) — goal position (typically remapped to `/clicked_point`).
* **`terrain_map`** ([grid_map_msgs/GridMap]) — 2.5D height map with terrain and traversability layers.

#### Published Topics

* **`global_plan`** ([quad_msgs/RobotPlan]) — interpolated global plan from current state to goal.
* **`global_plan_discrete`** ([quad_msgs/RobotPlan]) — discrete states from the underlying planning tree.

#### Services

* **`plan_with_constraints`** ([quad_msgs/PlanWithConstraints]) — replan under a CBS-supplied set of time-windowed pose constraints. Honours a `warm_start` flag that requests reuse of the cached RRT-Connect trees (lazy-pruned of vertices that fail the new constraints) instead of rebuilding from scratch. Suppresses the spin-loop solo publish while `global_body_planner.cbs_mode` is `true` so CBS plans aren't overwritten.

#### Parameters

* **`global_body_planner.update_rate`** (double, default: `20.0`) — planner loop rate in Hz.
* **`global_body_planner.num_calls`** (int, default: `1`) — planner calls per iteration.
* **`global_body_planner.max_planning_time`** (double, default: `0.20`) — max search time per call in s.
* **`global_body_planner.goal_state`** (double[], default: `[5.0, 0.0]`) — default goal (x, y) in the world frame.
* **`global_body_planner.pos_error_threshold`** (double, default: `25.0`) — position error (m) that triggers `RESET`. Set high to disable.
* **`global_body_planner.startup_delay`** (double, default: `2.0`) — `RESET` duration before publishing (s).
* **`global_body_planner.replanning`** (bool, default: `true`) — enable continuous replanning.
* **`global_body_planner.dt`** (double, default: `0.03`) — kinematic-check and interpolation timestep (s).
* **`global_body_planner.trapped_buffer_factor`** (int, default: `7`) — feasibility timesteps required to escape "trapped" state.
* **`global_body_planner.backup_ratio`** (double, default: `0.5`) — fraction of state-action pair to back up on invalid state (0–1).
* **`global_body_planner.num_leap_samples`** (int, default: `10`) — leap actions sampled per extend call.
* **`global_body_planner.traversability_threshold`** (double, default: `0.3`) — min traversability for a contact location. Lower = more optimistic; `0` disables.
* **`global_body_planner.mu`** (double, default: `0.25`) — friction coefficient.
* **`global_body_planner.g`** (double, default: `9.81`) — gravity in m/s².
* **`global_body_planner.t_s_{min,max}`** (double, default: `0.12` / `0.25`) — leaping stance time bounds in s.
* **`global_body_planner.dz0_{min,max}`** (double, default: `1.0` / `2.0`) — vertical velocity impulse bounds at leap onset in m/s.
* **`global_body_planner.cbs_mode`** (bool, default: `false`) — when `true`, the spin-loop suppresses solo `global_plan` publishes so the `conflict_based_search` node owns the publish stream. Set automatically by `multi_robot.py`.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[paper]: https://www.andrew.cmu.edu/user/amj1/papers/IROS2020_Fast_Global_Motion_Planning.pdf
[ROS2]: https://docs.ros.org/en/jazzy/
[quad_msgs/RobotState]: ../quad_msgs/msg/RobotState.msg
[quad_msgs/RobotPlan]: ../quad_msgs/msg/RobotPlan.msg
[quad_msgs/PlanWithConstraints]: ../quad_msgs/srv/PlanWithConstraints.srv
[geometry_msgs/PointStamped]: https://docs.ros.org/en/jazzy/p/geometry_msgs/
[grid_map_msgs/GridMap]: https://github.com/ANYbotics/grid_map
