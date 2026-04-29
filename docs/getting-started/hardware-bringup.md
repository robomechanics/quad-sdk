---
title: Hardware Bringup
tags:
  - hardware
  - getting-started
---

# Hardware Bringup

This page covers the **common path** to bring up Quad-SDK on a real quadruped. Per-platform notes (Spirit, Go1, Go2, B2, Spot, Vision60, Underbrush) are linked at the bottom.

!!! danger "Safety first"
    Real hardware is unforgiving. **Always use a safety harness or hoist for first-time bringup.** Validate every step in simulation with the same robot config before running on metal. Keep an e-stop within arm's reach.

## Hardware checklist

Before you power on the robot:

- [ ] Manufacturer SDK installed and verified independently of Quad-SDK (you can read joint state and command zero torque from a vendor demo)
- [ ] Battery is charged and seated correctly
- [ ] Low-level motor calibration is current
- [ ] Safety harness or hoist attached
- [ ] Quad-SDK builds clean on the on-board computer (or you've configured a remote driver — see below)
- [ ] Network: on-board computer and laptop on the same subnet, low latency, ROS 2 discovery working both ways

## On-board vs. remote driver

Quad-SDK runs the control loop in the `robot_driver` node. You can run it:

| Mode | Where `robot_driver` runs | When to pick this |
|---|---|---|
| **On-board** | On the robot's computer | Lowest latency, hardware deployment |
| **Remote** | On a tethered laptop | Bringup, debugging, perf-test rigs |

For remote, see `quad_utils/launch/remote_driver.py`.

## Per-robot configs

Robot-specific parameters live in `quad_utils/config/<robot>.yaml` and are selected by the `robot_type` launch arg. Examples:

```bash
# Go2 hardware bringup
ros2 launch quad_utils robot_driver.py robot_type:=go2 hardware:=true

# Spirit hardware bringup
ros2 launch quad_utils robot_driver.py robot_type:=spirit hardware:=true
```

The YAML files set: gait, control gains, kinematic limits, IMU bias, contact thresholds, NMPC weights, and motor mapping. **Always start from the upstream config and only override what's required for your unit.**

## Bringup sequence

1. **Power on** the robot, wait for the manufacturer SDK to settle.
2. **Source** the workspace on the on-board computer.
3. **Launch the driver** (it should report `mode: 0` / SAFETY).
   ```bash
   ros2 launch quad_utils robot_driver.py robot_type:=<your_robot> hardware:=true
   ```
4. **Verify topics** from a remote terminal:
   ```bash
   ros2 topic hz /robot_1/state/ground_truth     # state estimator
   ros2 topic hz /robot_1/joint_states            # raw joint feedback
   ros2 topic echo /robot_1/control/mode --once   # confirm mode
   ```
5. **Stand** the robot:
   ```bash
   ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 1" --once
   ```
6. **Plan** (optional, for autonomous walking):
   ```bash
   ros2 launch quad_utils quad_plan.py
   ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 2" --once
   ```
7. **Cut control** at any time:
   ```bash
   ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 0" --once
   ```

## State estimation

The default estimator is an **EKF** that fuses IMU, contact-aware kinematics, and (if available) motion-capture or VIO. Configuration lives under `robot_driver/include/robot_driver/estimators/`. See the [Robot Driver](../packages/robot_driver.md) reference for tuning.

For initial hardware bringup, **motion capture** is the easiest ground truth — the EKF can be configured to trust mocap aggressively while you tune contact estimation.

## Common bringup gotchas

??? warning "Robot drifts laterally on stand"
    Almost always an IMU bias issue. Run the calibration script at rest and confirm `/robot_1/imu/bias` looks stable.

??? warning "NMPC throws "Restoration_Failed""
    Indicates infeasible reference. Confirm the global plan is reachable, friction `mu` matches your floor, and `cmd_vel_scale` isn't pushing the robot past leg-velocity limits.

??? warning "Joints buzz / oscillate"
    Control gains are too high for the actuator bandwidth. Drop the YAML `kp_*` gains by 30% and re-tune.

??? warning "Robot collapses when entering WALK"
    The contact schedule and stance pose disagree. Confirm `t_stance_min/max` and the body-plan timestep match what the local planner expects.

## Per-platform notes

The platforms listed below have known-good configs and per-robot quirks:

- Spirit
- Unitree Go1
- Unitree Go2 / Go2-W
- Unitree B2
- Boston Dynamics Spot (joint API)
- Ghost Robotics Vision60
- Underbrush research platform

Per-robot pages will be expanded as the platform list matures — check each platform's `quad_utils/config/<robot>.yaml` for the canonical reference.

[:octicons-arrow-right-24: Architecture overview](../architecture/overview.md)
