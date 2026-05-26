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

Quad-SDK runs in three simulators: **Gazebo Harmonic**, **MuJoCo**, and **NVIDIA IsaacSim 5.1**. Each robot's verified coverage:

| Robot | Gazebo | MuJoCo | IsaacSim |
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
- **IsaacSim 5.1** *(beta)* — Spirit 40 and Go2 (the robots in the Isaac bridge `ROBOT_REGISTRY`). Requires a separate IsaacLab conda install ([install guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)). See [Running in IsaacSim](tutorials/isaac-sim.md) for setup and run instructions.

## Adding a new platform

If your robot isn't listed, see [Adding a new robot](tutorials/adding-a-robot.md) for the full step-by-step. We accept upstream contributions — file a PR with the description package and per-robot YAML.
