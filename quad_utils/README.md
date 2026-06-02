# Quad Utils

## Overview

This package is the shared-utility backbone of Quad-SDK. It provides:

- Kinematics and dynamics (`QuadKD2`) backed by **[Pinocchio]** for forward/inverse kinematics, Jacobians, CRBA, and RNEA.
- The top-level launch system for the entire stack (bringup, simulation, planning, logging, visualization).
- RViz interfacing, terrain-map generation and conversion, a remote-heartbeat watchdog, and math/ROS helper libraries.
- Per-platform YAML parameter sets for every supported robot.

### License

The source code is released under a [MIT License](../LICENSE).

**Author:** Joe Norby

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

**Maintainers:** Alex Stutt (astutt@andrew.cmu.edu), Qishun Yu (qishuny@andrew.cmu.edu)

Tested under [ROS2] Jazzy on Ubuntu 24.04. This is research code; expect frequent changes and no fitness for any particular purpose.

## Build

```bash
colcon build --packages-select quad_utils
```

Key dependencies (installed by `setup.sh` and `rosdep`): `pinocchio`, `grid_map_*`, `PCL`, `Eigen3`, `tf2`, `cv_bridge`, `pluginlib`.

> **Migration note:** `QuadKD2` replaces the earlier `QuadKD` class that used [RBDL] as its dynamics backend. Pinocchio delivers faster, templated rigid-body algorithms (CRBA, RNEA, analytic Jacobians) and is the canonical backend going forward. A legacy `FindRBDL.cmake` and `quad_kd.cpp` remain in-tree for reference but are not linked by the current build.

## Launch Files

All top-level Quad-SDK launch files live here. The most common entry points:

| Launch file | Purpose |
|---|---|
| `quad_gazebo.py` | Single-robot Gazebo Harmonic + bridges + robot driver. |
| `quad_multi.py` | N-robot Gazebo bringup (defaults to an 8-robot octagon-swap demo on `big_flat`). |
| `quad_mujoco.py` | MuJoCo alternative to Gazebo. |
| `quad_plan.py` | Full planning stack (global planner + local planner + NMPC). |
| `multi_robot.py` | Per-robot planning stack + central `conflict_based_search` node for the multi-robot demo. Wraps `quad_plan.py`, enforces `goal_state` per robot, forces `reference=gbpl`. |
| `robot_bringup.py` | Bring up the driver and robot-side nodes (sim or hardware). |
| `robot_driver.py` | Just the `robot_driver` node with config. |
| `remote_driver.py` | Operator-side remote driver (used for teleop over network). |
| `quad_visualization.py` | RViz interface + visualization plugins. |
| `visualization_plugins.py` | RViz plugin loader only. |
| `mapping.py` | Terrain-map publisher and grid-map filters. |
| `mocap.py` | OptiTrack / motion-capture integration. |
| `planning.py` | Planner stack without bringup (for attaching to an existing driver). |
| `logging.py` | `ros2 bag record` with the default Quad-SDK topic preset. |
| `logging_cbs.py` | Focused per-robot `ros2 bag record` for diagnosing CBS multi-robot tracking (lighter topic list than `logging.py`). Invoked per-robot by `multi_robot.py logging_cbs:=true`. |
| `force_applicator.py` / `force_app_single.py` | Runtime wrench disturbance for robustness tests. |
| `spawn_obstacles.py` | Spawn obstacles into a running Gazebo world. |
| `testing.py` | CI/testing harness. |

## Config

24 YAML/XML configs under `config/`, including:

- **Per-robot:** `a1.yaml`, `b2.yaml`, `go1.yaml`, `go2.yaml`, `go2w.yaml`, `spirit.yaml`, `spot.yaml`, `vision60.yaml`.
- **Topic names:** `topics_global.yaml`, `topics_robot.yaml` — canonical namespaces shared across all packages.
- **Visualization:** `rviz_visualization.yaml`, `rviz_interface.yaml`, `grid_map_visualization.yaml`, `plotjuggler_config*.xml`.
- **Terrain / mapping:** `terrain_map_publisher.yaml`, `filter_chain.yaml`.
- **Bridges + misc:** `ros_gz_bridge.yaml`, `joy_config.yaml`, `teleop_twist_keyboard.yaml`, `remote_heartbeat.yaml`, `trajectory_publisher.yaml`, `dashboard.perspective`.

## Nodes

### `rviz_interface_node`

Bridges `quad_msgs` types to RViz-friendly `visualization_msgs` markers — plans, GRF arrows, foot traces, joint states for each of `estimate` / `ground_truth` / `trajectory`.

### `remote_heartbeat_node`

Publishes a stamped `std_msgs/Header` at a fixed rate on `heartbeat/remote`. The robot driver uses it as a liveness signal; loss of heartbeat triggers safe-mode.

### `mesh_to_grid_map_node`

Converts a triangle mesh (e.g. terrain STL from `quad_sim_scripts/models/`) into a `grid_map_msgs/GridMap` layer usable by the planners.

### `grid_map_filters_demo`

Reference node that shows how `filter_chain.yaml` composes grid-map filters (normals, traversability mask, inpainting).

## Libraries (C++)

### `QuadKD2` (`include/quad_utils/quad_kd2.hpp`, `src/quad_kd2.cpp`)

Pinocchio-based kinematics/dynamics. All frame transforms follow the convention:

```cpp
QuadKD2::proximalLinkToDistalLinkFK/IKCoordFrame(...);
```

e.g. `worldToFootFKWorldFrame(...)` — foot origin expressed in world frame, given body pose and joint state;
`legbaseToFootIKLegbaseFrame(...)` — joint angles that place the foot at a given position in the legbase frame.

Typical use:

```cpp
#include "quad_utils/quad_kd2.hpp"

auto quadKD = std::make_shared<quad_utils::QuadKD2>();
quadKD->getRotationMatrix(rpy, rot);
quadKD->worldToFootFKWorldFrame(body_state, joint_state, foot_positions);
```

Internally uses `pinocchio::Model`, `pinocchio::Data`, CRBA for the mass matrix, and RNEA for inverse dynamics. The file is compiled with `-O2` to manage Pinocchio's template expansion cost.

### `FastTerrainMap` (`src/fast_terrain_map.cpp`)

Dense 2.5D height-map representation used by the planners; supports lookups by world-frame (x, y) and returns height, slope, and traversability.

### `math_utils.cpp`, `ros_utils.cpp`

Small helpers — quaternion/RPY conversions, filtering, trajectory interpolation, ROS2 parameter-loading wrappers.

## Tests

Unit tests for `QuadKD2`, `math_utils`, the RViz interface, and the terrain map publisher live under `test/`.

```bash
colcon test --packages-select quad_utils
colcon test-result --verbose
```

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ROS2]: https://docs.ros.org/en/jazzy/
[Pinocchio]: https://stack-of-tasks.github.io/pinocchio/
[RBDL]: https://rbdl.github.io/
[Wiki]: https://github.com/robomechanics/quad-sdk/wiki
