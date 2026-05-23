# Packages

Quad-SDK is a workspace of focused ROS 2 packages. Each one builds in isolation (`colcon build --packages-select <name>`) and most have their own unit tests.

## Planning

| Package | What it does |
|---|---|
| [`global_body_planner`](global_body_planner.md) | RRT-Connect planner with mixed motion primitives (GBP-L); exposes a `plan_with_constraints` service for multi-robot replanning |
| [`local_planner`](local_planner.md) | NMPC body plan + Raibert footstep heuristic |
| [`nmpc_controller`](nmpc_controller.md) | CasADi/IPOPT nonlinear MPC, exposed as a library |
| [`conflict_based_search`](conflict_based_search.md) | Multi-robot global planning — coordinates per-robot global planners via OBB conflict detection + time-windowed constraints |

## Control + estimation

| Package | What it does |
|---|---|
| [`robot_driver`](robot_driver.md) | Real-time control loop, EKF state estimation, hardware abstraction |
| [`body_force_estimator`](body_force_estimator.md) | Momentum-based external wrench observer |

## Simulation

| Package | What it does |
|---|---|
| [`quad_simulator`](quad_simulator.md) | Gazebo Harmonic + MuJoCo simulation — physics plugins, worlds, terrains, and robot descriptions |
| [`isaac_plugins`](isaac_plugins.md) | **Beta** NVIDIA Isaac Sim 5.1 backend — Isaac-side bridge, URDF→USD prep pipeline, underbrush scenario (Spirit 40, Go2) |
| [`force_applicator`](force_applicator.md) | Runtime disturbance injection for robustness testing |

## Utilities

| Package | What it does |
|---|---|
| [`quad_utils`](quad_utils.md) | Kinematics/dynamics (Pinocchio), launch system, shared utilities |
| [`quad_msgs`](quad_msgs.md) | Message definitions |
| [`quad_logger`](quad_logger.md) | Bag recording + log post-processing (Python + MATLAB) |
| [`quad_training`](quad_training.md) | Time-sync + episode management for RL/BC data collection |
| [`quad_perf_tests`](quad_perf_tests.md) | Performance-testing utilities |
| [`scripts`](scripts.md) | Blender visualization / rendering scripts |
