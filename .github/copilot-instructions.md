# Copilot instructions for quad-sdk

## Big-picture architecture
- This repo is a ROS 2 (Jazzy) workspace of modular packages; core stack lives under [quad_utils](quad_utils), planners/controllers in [global_body_planner](global_body_planner), [local_planner](local_planner), [nmpc_controller](nmpc_controller), and estimators like [body_force_estimator](body_force_estimator). Message types are defined in [quad_msgs](quad_msgs).
- Simulation and bringup are centered in launch files under [quad_utils/launch](quad_utils/launch). The stack is composed via launch-time composition (e.g., `planning.py` + `quad_plan.py` + `robot_bringup.py`).
- Gazebo Harmonic integration uses `gz sim` plus `ros_gz_bridge` for clocks, contacts, and IMU; see [quad_utils/launch/quad_gazebo.py](quad_utils/launch/quad_gazebo.py) and [quad_utils/launch/robot_bringup.py](quad_utils/launch/robot_bringup.py).

## Key flows and integration points
- Multi-robot configurations are passed as JSON strings via `robot_configs` launch arguments (e.g., `quad_gazebo.py` and `quad_plan.py`). Keep schema fields consistent: `name`, `type`, controller/mode, and `init_pose` or `reference`.
- Planner launch flow: [quad_utils/launch/quad_plan.py](quad_utils/launch/quad_plan.py) → [quad_utils/launch/planning.py](quad_utils/launch/planning.py), which conditionally wires global/local planners, body force estimator, and logging based on launch args.
- Robot parameterization is YAML-driven using ROS 2 `ros__parameters` and namespace wildcards (`/**/`). Examples in [quad_utils/config/go2.yaml](quad_utils/config/go2.yaml) show how global/local planner and driver parameters are scoped.

## Developer workflows (discoverable in repo)
- Dependency install script: run [setup.sh](setup.sh) at repo root to install ROS 2 Jazzy deps, per-package `setup_deps.sh` scripts, and `rosdep` entries.
- Build/test conventions use `ament_cmake` and gtest (see `ament_add_gtest` in [quad_utils/CMakeLists.txt](quad_utils/CMakeLists.txt) and tests in [quad_utils/test](quad_utils/test) and [global_body_planner/test](global_body_planner/test)).

## Project-specific conventions
- Launch files favor `OpaqueFunction` helpers and `FindPackageShare` to construct paths (see [quad_utils/launch/planning.py](quad_utils/launch/planning.py)). Follow this style when adding new launch logic.
- Simulation visuals and dashboards are optional and toggled by launch args (`live_plot`, `dash`) in [quad_utils/launch/quad_gazebo.py](quad_utils/launch/quad_gazebo.py).
- Hardware/sim bridging and robot spawning logic live in `robot_bringup.py`; prefer adding new bridge topics there rather than ad-hoc in other launch files.

## Where to look first
- Launch orchestration: [quad_utils/launch](quad_utils/launch)
- Robot parameters: [quad_utils/config](quad_utils/config)
- Messages: [quad_msgs](quad_msgs)
- Simulation plugins/scripts: [quad_simulator](quad_simulator) (gazebo_plugins, mujoco_plugins, quad_controllers, quad_sim_scripts)
