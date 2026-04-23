# Quad Perf Tests

## Overview

This package provides performance-testing utilities for Quad-SDK. The primary tool is a configurable `cmd_vel` publisher that drives the robot through deterministic or randomized velocity trajectories — useful for benchmarking controller tracking, stress-testing the planning stack, and collecting reproducible training rollouts.

### License

The source code is released under a [MIT License](../LICENSE).

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

Tested under [ROS2] Jazzy on Ubuntu 24.04.

## Build

```bash
colcon build --packages-select quad_perf_tests
```

## Usage

Run the publisher against a running stack:

```bash
ros2 run quad_perf_tests cmd_vel_publisher_node --ros-args \
    --params-file install/quad_perf_tests/share/quad_perf_tests/config/cmd_vel_publisher.yaml \
    --params-file install/quad_perf_tests/share/quad_perf_tests/config/cmd_vel_publisher_topics.yaml
```

## Config

* **`config/cmd_vel_publisher.yaml`** — mode, velocity bounds, duration, seed.
* **`config/cmd_vel_publisher_topics.yaml`** — topic remappings.

## Nodes

### cmd_vel_publisher_node

Publishes a `geometry_msgs/Twist` command on the `cmd_vel` topic. Three modes are supported:

- **`single`** — sample one random velocity within the configured bounds and hold it for `test_duration` seconds.
- **`timer`** — resample a new random velocity every `resample_sec` seconds.
- **`off`** — publish zero velocity (useful as a keep-alive).

#### Published Topics

* **`cmd_vel`** ([geometry_msgs/Twist])

	Commanded body twist.

#### Parameters

* **`cmd_vel_publisher.update_rate`** (double, default: `500.0`) — publish rate in Hz. Must match `robot_driver.update_rate`.
* **`cmd_vel_publisher.mode`** (string, default: `single`) — one of `single`, `timer`, `off`.
* **`cmd_vel_publisher.resample_sec`** (double, default: `3.0`) — resample interval for `timer` mode.
* **`cmd_vel_publisher.bounds.x_{min,max}`** (double) — forward velocity bounds in m/s.
* **`cmd_vel_publisher.bounds.y_{min,max}`** (double) — lateral velocity bounds in m/s.
* **`cmd_vel_publisher.bounds.yaw_{min,max}`** (double) — yaw-rate bounds in rad/s.
* **`cmd_vel_publisher.test_duration`** (double, default: `10.0`) — duration for `single` mode in seconds.
* **`cmd_vel_publisher.seed`** (int, default: `1`) — RNG seed for reproducibility.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ROS2]: https://docs.ros.org/en/jazzy/
[geometry_msgs/Twist]: https://docs.ros.org/en/jazzy/p/geometry_msgs/
