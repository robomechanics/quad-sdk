---
title: Architecture Overview
tags:
  - architecture
  - design
---

# Architecture Overview

Quad-SDK is a layered, message-driven stack. Each layer is a ROS 2 package with clear input/output topics, so individual layers can be swapped or studied in isolation.

![Quad-SDK architecture](../assets/quad_sdk_architecture.png){ .hero-image }

## The hierarchy

```mermaid
flowchart TB
  subgraph PERCEPTION[Perception]
    TM[terrain_map]
  end

  subgraph PLAN[Planning]
    GP[global_body_planner<br/>RRT-Connect + leaps]
    LP[local_planner<br/>NMPC body + Raibert footstep]
    NMPC[(nmpc_controller<br/>library)]
  end

  subgraph CONTROL[Control + estimation]
    RD[robot_driver<br/>control loop]
    EKF[state_estimator<br/>EKF]
    BFE[body_force_estimator]
  end

  subgraph EXEC[Execution]
    HW[hardware_interface]
    SIM[Gazebo / MuJoCo]
  end

  TM --> GP
  TM --> LP
  GP -- global_plan --> LP
  LP -- local_plan --> RD
  LP <-.uses.-> NMPC
  RD -- joint_cmd --> HW
  RD -- joint_cmd --> SIM
  HW -- joint_state, imu --> EKF
  SIM -- joint_state, imu --> EKF
  EKF -- state/ground_truth --> LP
  EKF -- state/ground_truth --> GP
  EKF -- state/ground_truth --> RD
  EKF -- state/ground_truth --> BFE
  RD -- joint_cmd --> BFE
  BFE -- external_wrench --> LP
```

## Layer summary

| Layer | Frequency | What it produces | Where it lives |
|---|---|---|---|
| **Multi-robot coordination** *(optional)* | per goal change | Conflict-free per-robot global plans | [`conflict_based_search`](../packages/conflict_based_search.md) |
| **Global plan** | 5–20 Hz | Path of body states + GRFs to a goal | [`global_body_planner`](../packages/global_body_planner.md) |
| **Local plan** | 333 Hz | 26-step NMPC body plan + footstep schedule | [`local_planner`](../packages/local_planner.md) |
| **NMPC** | called per local-plan tick | OCP solution (states, GRFs) | [`nmpc_controller`](../packages/nmpc_controller.md) |
| **Control** | 1 kHz | Per-joint torque/PD commands | [`robot_driver`](../packages/robot_driver.md) |
| **Estimation** | 1 kHz | Body state + foot contacts | [`robot_driver`](../packages/robot_driver.md) |
| **External wrench** | 100 Hz | Disturbance estimate | [`body_force_estimator`](../packages/body_force_estimator.md) |
| **Simulation** | 1–2 kHz | Joint states, IMU, contacts | [`quad_simulator`](../packages/quad_simulator.md) |

For multi-robot scenarios, the [`conflict_based_search`](../packages/conflict_based_search.md) node sits *above* the per-robot global planners and coordinates them via the `plan_with_constraints` service. See the [Multi-Robot tutorial](../tutorials/multi-robot.md).

## Topic conventions

All robot-scoped topics are namespaced as `/robot_<id>/<topic>`. Multi-robot configs (see CBS demos) replicate the full stack under each namespace.

| Topic | Type | Producer → Consumer |
|---|---|---|
| `state/ground_truth` | `quad_msgs/RobotState` | estimator → planners, driver |
| `global_plan` | `quad_msgs/RobotPlan` | global → local planner |
| `local_plan` | `quad_msgs/LocalPlan` | local planner → driver |
| `foot_plan_continuous` | `quad_msgs/MultiFootPlanContinuous` | local planner → driver |
| `terrain_map` | `grid_map_msgs/GridMap` | perception → planners |
| `cmd_vel` | `geometry_msgs/Twist` | user → local planner |
| `control/mode` | `std_msgs/UInt8` | user → driver |

Full message definitions: [`quad_msgs`](../packages/quad_msgs.md).

## Launch composition

The runtime is composed by launch files under `quad_utils/launch/` rather than monolithic config:

```
# Single-robot
quad_gazebo.py ─┐
                ├── robot_bringup.py ── robot_driver.py
quad_mujoco.py ─┘                       (per-robot)

quad_plan.py ──── planning.py ──── global_body_planner_node
                                ├── local_planner_node
                                ├── body_force_estimator (optional)
                                └── logging (optional)

# Multi-robot
quad_multi_gazebo.py ── robot_bringup.py × N      (one per robot)

multi_robot.py  ──── planning.py × N              (per-robot stacks)
                ├── conflict_based_search_node    (central coordinator)
                └── logging_cbs.py × N            (optional, --logging_cbs:=true)
```

Single-robot scenarios pass a `robot_configs` JSON list with one entry; multi-robot scenarios pass an N-robot list with each robot's `goal_state` field set. `robot_bringup.py` iterates and stamps namespaces. The CBS coordinator only runs in the multi-robot path. See:

- `quad_utils/launch/quad_gazebo.py` (single)
- `quad_utils/launch/quad_multi_gazebo.py` (multi)
- `quad_utils/launch/multi_robot.py` (per-robot planning + CBS)
- `quad_utils/launch/logging_cbs.py` (per-robot CBS-debug bagging)

## Module deep-dives

- [ROS 2 migration](ros2-migration.md) — what changed from ROS 1, what to update if you're porting code.
- [Pinocchio integration](pinocchio-integration.md) — how the kinematics/dynamics layer was rewritten.

## Design choices worth knowing

!!! info "Why NMPC as a library?"
    `nmpc_controller` builds a CasADi/IPOPT solver once at startup and exposes a synchronous `solve()`. The local planner calls it directly so we avoid the latency and brittleness of a separate ROS node round-trip per control tick.

!!! info "Why split global vs. local planning?"
    Global search (~hundreds of milliseconds, RRT-Connect) and local refinement (~3 ms, NMPC) live on different timescales. Splitting them lets each be tuned and replaced independently.

!!! info "Why include both Gazebo and MuJoCo?"
    Gazebo is feature-rich (sensors, physics plugins, fluid contact). MuJoCo is fast, deterministic, and renders without a GPU — ideal for headless training rigs and perf tests.
