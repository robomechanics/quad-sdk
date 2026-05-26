---
title: FAQ
tags:
  - faq
  - troubleshooting
---

# FAQ

## Does Quad-SDK support bipeds, hexapods, or other morphologies?

Quad-SDK is designed for quadrupeds — specifically platforms with quasi-direct-drive actuators and lightweight 3-DOF legs. Many of the algorithms (RRT-Connect global planning, NMPC body planning, Pinocchio kinematics) generalize, but the per-package config schema, joint mappings, and the `nmpc_controller` codegen all assume four 3-DOF legs. Community ports for other morphologies are welcome.

## Will Quad-SDK get *X* feature?

The Robomechanics Lab maintains the repo alongside its research projects. We try to keep upstream useful, but feature requests aren't all on the roadmap. PRs are the most reliable path — see [Contributing](contributing.md).

## What's the joint and leg numbering convention?

Joint arrays use indices 0–11 in this order:

```
abad0, hip0, knee0,    # leg 0 — front left
abad1, hip1, knee1,    # leg 1 — back  left
abad2, hip2, knee2,    # leg 2 — front right
abad3, hip3, knee3,    # leg 3 — back  right
```

So `state_msg.joints.positions[4]` is leg 1's hip position (back left).

## Why three simulators (Gazebo, MuJoCo, IsaacSim)?

Gazebo Harmonic gives us first-class sensor and contact plugins, mature tooling, and a worldfile/SDF ecosystem, and it covers every supported platform. MuJoCo is fast, deterministic, renders without a GPU, and is the better fit for headless training rigs. NVIDIA IsaacSim 5.1 (**beta**, separate [IsaacLab conda install](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)) adds high-fidelity, GPU-accelerated rendering for the Spirit 40 and Go2 — see [Running in IsaacSim](tutorials/isaac-sim.md). The controller and planning stacks run unmodified across all three — see the [simulator support matrix](platforms.md#simulator-support).

## Why Pinocchio over RBDL?

See [Pinocchio integration](architecture/pinocchio-integration.md). Short version: active maintenance, first-class CasADi auto-diff (which the NMPC re-uses), and a healthier user community in the legged-robotics space.

## Why does my new robot drift sideways on stand?

Almost always a joint sign/offset error in the per-robot YAML. See [Adding a new robot](tutorials/adding-a-robot.md#step-2-per-robot-yaml). Verify by commanding each joint individually with the manufacturer SDK and translating the polarity into the YAML.

## NMPC reports `Restoration_Failed` — what now?

The reference is infeasible. Common causes:

1. Global plan jumps past kinematic limits (`cmd_vel_scale` too high)
2. Friction `mu` doesn't match the simulation surface
3. NMPC GRF bounds (`u_lb`/`u_ub`) are too tight after a per-robot config edit

Bisect by reducing `cmd_vel` to zero — if the NMPC stabilizes, the reference is the issue, not the model.

## Catkin vs. colcon

Quad-SDK is a **ROS 2** workspace and uses `colcon` + `ament_cmake`. ROS 1 / catkin commands in older material no longer apply.

## Building runs out of RAM

Lower parallelism:

```bash
colcon build --parallel-workers 4
# or, for an even tighter build
colcon build --executor sequential
```

Most often the offender is the NMPC code-gen stage — build that package in isolation with reduced parallelism.

## Gazebo crashes on startup

- Confirm Gazebo runs standalone: `gz sim`. If that fails, it's a system / driver issue, not a Quad-SDK issue.
- Kill stale instances: `killall -9 gzserver gzclient`.
- On NVIDIA: update Mesa drivers, or use MuJoCo for headless work.

## RViz crashes when I use *Publish Point*

Known interaction between NVIDIA drivers and RViz interactive markers. Workaround: switch to software rendering (`LIBGL_ALWAYS_SOFTWARE=1`) for visualization-only sessions, or set the goal via the parameter API:

```bash
ros2 param set /global_body_planner global_body_planner.goal_state "[5.0, 0.0]"
```

## How do I see ROS 1 docs?

Use the version selector in the top-right of this site to switch to the **`ros1`** version. The ROS 1 stack is preserved but no longer actively developed.

## How do I report a bug or request a feature?

[GitHub issue tracker](https://github.com/robomechanics/quad-sdk/issues). Please include: ROS distro, robot type, full launch command, and the relevant log lines. Bag attachments are welcome.
