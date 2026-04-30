# Conflict Based Search

## Overview

This package implements multi-robot global planning for Quad-SDK using Conflict-Based Search (CBS). Given a fleet of quadrupeds with individual goals, CBS solves for a set of per-robot global plans that are pairwise conflict-free at every aligned timestep. Each robot continues to be planned by its own [`global_body_planner`](../global_body_planner) instance — the CBS node sits above them, detects body-OBB conflicts between candidate plans, and asks the involved robot's planner to replan under additional time-windowed constraints until no conflicts remain.

### License

The source code is released under a [MIT License](../LICENSE).

**Author:** David Ologan

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

**Maintainer:** David Ologan (dologan@andrew.cmu.edu)

Tested under [ROS2] Jazzy on Ubuntu 24.04. This is research code; expect frequent changes and no fitness for any particular purpose.

### Publications

If you use this work in an academic context, please cite:

* G. Sharon, R. Stern, A. Felner, and N. R. Sturtevant, "Conflict-based search for optimal multi-agent pathfinding," *Artificial Intelligence*, vol. 219, pp. 40–66, 2015.

```bibtex
@article{Sharon2015,
  title={Conflict-based search for optimal multi-agent pathfinding},
  author={Sharon, Guni and Stern, Roni and Felner, Ariel and Sturtevant, Nathan R},
  journal={Artificial Intelligence},
  volume={219},
  pages={40--66},
  year={2015},
  publisher={Elsevier}
}
```

## Build

```bash
colcon build --packages-select conflict_based_search
```

## Usage

The CBS node is launched on top of the standard planning stack:

```bash
# Terminal 1: world + N robots
ros2 launch quad_utils quad_multi.py

# Terminal 2: per-robot stand
ros2 topic pub --once /<ns>/control/mode std_msgs/UInt8 "data: 1"

# Terminal 3: per-robot planning stack + central CBS
ros2 launch quad_utils multi_robot.py
```

`multi_robot.py` is a thin wrapper around `quad_plan.py` that adds the central `conflict_based_search_node`, enforces a `goal_state` per robot, and forces every robot's reference to `gbpl` (CBS requires the global body planner — twist-controlled robots have no `plan_with_constraints` service to call).

Optional CBS-debug bagging:

```bash
ros2 launch quad_utils multi_robot.py logging_cbs:=true
```

Records a focused per-robot bag (global plan, local plan, ground truth, joint state, GRFs, control commands) under `${QUAD_LOGGER_SRC}/bags/cbs/`. Off by default — subscribers + serialisation cost a few % CPU per robot.

## Config

* **`config/conflict_based_search.yaml`** — robot list, body extents for OBB collision checking, expansion cap, warm-start flag, service timeout.

## Nodes

### conflict_based_search_node

Runs the high-level CBS loop. Maintains a priority queue of CBS nodes (each holding a per-robot plan set and accumulated constraints), repeatedly pops the lowest-cost node, looks for the first body-OBB conflict between any two robots, and branches by adding a constraint and replanning the involved robot. Terminates when a conflict-free node is found and publishes the per-robot plans.

#### Conflict detection

Two robots' plans are in conflict at index `k` if their body OBBs overlap when the bodies are placed at `states[k]` of their respective plans. The check uses the separating-axis theorem on yaw-aligned planar OBBs with a 1D Z-interval reject; consecutive states are also tested at their midpoint to catch pass-through cases between sample timesteps.

#### Replanning under constraints

When a conflict is detected at index range `[t_start, t_end]`, the CBS node calls each involved robot's `/<robot>/plan_with_constraints` service with a `RobotPlanConstraints` message that lists the *other* robot's pose at every state in the conflict window. The receiving planner stores these as time-windowed dynamic obstacles and returns a fresh plan. With `warm_start: true`, the planner reuses its cached RRT-Connect trees and lazy-prunes only the vertices that violate the new constraints.

#### Subscribed Topics

None. CBS pulls plans from the per-robot global planners synchronously via service.

#### Published Topics

* **`/<robot>/global_plan`** ([quad_msgs/RobotPlan]) — per robot in `robot_names`. Published once when CBS converges; per-state stamps are re-based to "now" so downstream local planners see fresh timestamps.

#### Service Clients

* **`/<robot>/plan_with_constraints`** ([quad_msgs/PlanWithConstraints]) — per robot in `robot_names`. Sends the current constraint set + warm-start flag, receives the robot's new plan and its path length.

#### Parameters

* **`robot_names`** (string[], default: `["robot_1", "robot_2"]`) — names whose `/<robot>/plan_with_constraints` services CBS will call.
* **`update_rate`** (double, default: `5.0`) — Hz used while polling pending service futures.
* **`service_timeout_s`** (double, default: `30.0`) — per-call timeout on `plan_with_constraints`.
* **`warm_start`** (bool, default: `true`) — request that each per-robot planner lazy-prune its cached RRT-Connect trees on replan instead of building from scratch. Turn off to recover the original from-scratch behaviour.
* **`max_iterations`** (int, default: `500`) — hard cap on CBS expansions. On hit, the best-known plan is published with a warning rather than looping indefinitely.
* **`body_length`** / **`body_width`** / **`body_height`** (double, defaults: `0.70` / `0.35` / `0.35`) — OBB extents in m used for both the high-level conflict check and the planner-side constraint check. Sized for go2 with margin; assumes a homogeneous fleet.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ROS2]: https://docs.ros.org/en/jazzy/
[quad_msgs/RobotPlan]: ../quad_msgs/msg/RobotPlan.msg
[quad_msgs/PlanWithConstraints]: ../quad_msgs/srv/PlanWithConstraints.srv
