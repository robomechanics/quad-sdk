---
title: Roadmap
---

# Roadmap

Active workstreams for Quad-SDK. Each item below is in scope for the current development cycle; finer-grained tasks (PR-sized) live in the relevant package README or GitHub issues.

## Perception stack integration

Wire the perception terrain stack end-to-end into the planning + control loop so that locally-sensed terrain (depth / LiDAR-derived heightmaps) drives the global body planner and local planner without the operator pre-loading a static `GridMap`. Today the perception nodes publish maps but the planners still consume the pre-loaded terrain; closing this gap unlocks fully-onboard outdoor runs.

## Extending QuadKD2 for wheeled-legged robots

Generalize the Pinocchio-backed [`QuadKD2`](architecture/pinocchio-integration.md) kinematics/dynamics layer to support wheeled-legged platforms. Concretely: add continuous wheel joints to the URDF mapping logic, model wheel–ground contact (rolling + slip) alongside the existing per-toe contact model, and expose wheel velocities through the existing FK/Jacobian APIs so controllers and the body force estimator do not need to special-case wheel feet.

## IsaacSim 6.0 support

Port the [`isaac_plugins`](packages/isaac_plugins.md) bridge from IsaacSim 5.1 to 6.0. The 6.0 `URDFImporter` is class-based — `omni.kit.commands` registration is gone and per-import physics knobs (mesh approximation, joint armature, contact offsets) move to a post-import USD pass. The bridge's runtime path needs the matching rewrite, plus a refreshed IsaacLab conda env recipe. See the [`isaac_plugins` README roadmap](https://github.com/robomechanics/quad-sdk/blob/main/quad_simulator/isaac_plugins/README.md#roadmap) for the live punch list.

## ros2_control integration

Migrate the [`robot_driver`](packages/robot_driver.md) off its bespoke manufacturer-SDK loops onto the standard [`ros2_control`](https://control.ros.org/) framework: implement a `hardware_interface::SystemInterface` per supported platform (Unitree Go2, Spirit 40), expose joint state / effort interfaces, and drive the existing inverse-dynamics and MPC controllers as `controller_manager` plugins. This brings Quad-SDK in line with the wider ROS 2 ecosystem (shared controllers, easier hardware swaps, standard launch patterns) and removes a class of bringup-time wiring bugs.
