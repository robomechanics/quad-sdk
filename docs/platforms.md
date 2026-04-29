---
title: Supported platforms
tags:
  - hardware
  - platforms
---

# Supported platforms

Quad-SDK ships configs and hardware interfaces for the following quadrupeds. **Tested** means the lab has run the full planning + control stack on real hardware. **Config-ready** means simulation works and the config is in-tree but hardware has not been verified yet.

## At a glance

| Platform | Manufacturer | Status | Mass | Sim backend |
|---|---|---|---|---|
| **Spirit 40** | Ghost Robotics | :material-check-bold: Tested | 12 kg | Gazebo + MuJoCo |
| **Go2** | Unitree | :material-check-bold: Tested | 15 kg | Gazebo + MuJoCo |
| **Go1** | Unitree | :material-check-bold: Tested | 12 kg | Gazebo + MuJoCo |
| **A1** | Unitree | :material-check-bold: Tested | 12 kg | Gazebo + MuJoCo |
| **B2** | Unitree | :material-check-bold: Tested | 60 kg | Gazebo |
| **Spot** | Boston Dynamics | :material-check-bold: Tested | 32.5 kg | Gazebo (joint API) |
| **Vision60** | Ghost Robotics | :material-check-bold: Tested | 50 kg | Gazebo |
| **Go2-W** (wheeled) | Unitree | :material-check: Config-ready | 18 kg | Gazebo |

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
