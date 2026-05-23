---
title: Supported platforms
tags:
  - hardware
  - platforms
---

# Supported platforms

Quad-SDK ships configs and hardware interfaces for the following quadrupeds. **Hardware tested** means the lab has run the full planning + control stack on the physical robot — currently the **Ghost Robotics Spirit 40** and the **Unitree Go2**. Every listed platform has an in-tree config and runs in simulation; see the [simulator support matrix](#simulator-support) below for which backend covers which robot.

## At a glance

| Platform | Manufacturer | Mass | Hardware status |
|---|---|---|---|
| **Spirit 40** | Ghost Robotics | 12 kg | :material-check-bold: Tested on hardware |
| **Go2** | Unitree | 15 kg | :material-check-bold: Tested on hardware |
| **Go1** | Unitree | 12 kg | :material-check: Simulation / config-ready |
| **A1** | Unitree | 12 kg | :material-check: Simulation / config-ready |
| **B2** | Unitree | 60 kg | :material-check: Simulation / config-ready |
| **Spot** | Boston Dynamics | 32.5 kg | :material-check: Simulation / config-ready |
| **Vision60** | Ghost Robotics | 50 kg | :material-check: Simulation / config-ready |
| **Go2-W** (wheeled) | Unitree | 18 kg | :material-check: Simulation / config-ready |

## Simulator support

Quad-SDK runs in three simulators: **Gazebo Harmonic**, **MuJoCo**, and **NVIDIA Isaac Sim 5.1**. Each robot's verified coverage:

| Robot | Gazebo | MuJoCo | Isaac Sim |
|---|:---:|:---:|:---:|
| **Spirit 40** | :material-check: | :material-check: | :material-check: |
| **Go2** | :material-check: | :material-check: | :material-check: |
| **Go1** | :material-check: | :material-check: | :material-close: |
| **A1** | :material-check: | :material-close: | :material-close: |
| **B2** | :material-check: | :material-check: | :material-close: |
| **Spot** | :material-check: | :material-check: | :material-close: |
| **Vision60** | :material-check: | :material-check: | :material-close: |
| **Go2-W** | :material-check: | :material-check: | :material-close: |

- **Gazebo Harmonic** — every platform.
- **MuJoCo** — every platform except the A1.
- **Isaac Sim 5.1** *(beta)* — Spirit 40 and Go2 (the robots in the Isaac bridge `ROBOT_REGISTRY`). Requires a separate IsaacLab conda install ([install guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)). See [Running in Isaac Sim](tutorials/isaac-sim.md) for setup and run instructions.

## Adding a new platform

If your robot isn't listed, see [Adding a new robot](tutorials/adding-a-robot.md) for the full step-by-step. We accept upstream contributions — file a PR with the description package and per-robot YAML.

## Performance reference

Approximate steady-state CPU on the planner stack with the default 1 kHz control loop and 333 Hz local planner:

| Platform tier | Onboard CPU | NMPC step | Local planner | Notes |
|---|---|---|---|---|
| **Tier 1** (Go2, Spirit, Go1) | i7 / Ryzen 7 + 16 GB | ~2.8 ms | ~3.0 ms | Fits comfortably |
| **Tier 2** (Jetson Orin) | 8-core Cortex-A78 | ~5.5 ms | ~6.0 ms | Tighten horizon for headroom |
| **Tier 3** (Jetson Xavier NX) | 6-core Carmel | ~9 ms | ~10 ms | Drop horizon to 16, raise dt |

These numbers are eyeballed from CI bench reports — the [perf-tests package](packages/quad_perf_tests.md) reproduces them.
