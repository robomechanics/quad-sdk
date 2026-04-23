# Force Applicator

## Overview

This package applies external wrenches (forces and torques) to a simulated robot at runtime. It is used for robustness testing and disturbance rejection experiments — e.g., pushing the robot during locomotion to evaluate controller recovery. The applicator supports fixed, random, and periodic disturbance profiles, and interfaces with Gazebo's `ApplyLinkWrench` service via `ros_gz_interfaces`.

### License

The source code is released under a [MIT License](../LICENSE).

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

Tested under [ROS2] Jazzy on Ubuntu 24.04 with Gazebo Harmonic.

## Build

```bash
colcon build --packages-select force_applicator
```

## Usage

Launch the applicator alongside a running Gazebo simulation:

```bash
ros2 launch quad_utils force_applicator.py
```

For single-robot configurations:

```bash
ros2 launch quad_utils force_app_single.py
```

## Config

* **`config/force_applicator.yaml`** — wrench magnitudes, disturbance mode, update rate, RNG seed.
* **`config/force_applicator_topics.yaml`** — topic remappings.

## Nodes

### force_applicator_node

Periodically selects a wrench (fixed, random, or periodic) and applies it to the robot trunk link via Gazebo service calls. Also publishes a visualization marker that shows the current applied wrench in RViz.

#### Subscribed Topics

* **`state/ground_truth`** ([quad_msgs/RobotState])

	Current robot state, used for position-triggered disturbances.

#### Published Topics

* **`visualization/force_torque_markers`** ([visualization_msgs/MarkerArray])

	Arrow markers indicating the magnitude and direction of the applied wrench.

* **`wrench/persistent`** ([ros_gz_interfaces/EntityWrench])

	Persistent wrench request sent to the Gazebo bridge.

* **`wrench/clear`** ([ros_gz_interfaces/Entity])

	Clears any persistent wrench.

#### Key Parameters

* **`force_applicator.fixed.force.{x,y,z}`** (double) — fixed-mode force components in N.
* **`force_applicator.fixed.torque.{x,y,z}`** (double) — fixed-mode torque components in Nm.
* **`force_applicator.duration.dt`** (double, default: `0.5`) — duration of each applied wrench in seconds.
* **`force_applicator.duration.update_rate`** (double, default: `100.0`) — update rate in Hz.
* **`force_applicator.random.force_mag_{min,max}`** (double) — per-axis random force magnitude bounds in N.
* **`force_applicator.random.torque_mag_{min,max}`** (double) — per-axis random torque magnitude bounds in Nm.
* **`force_applicator.periodic.period`** (double, default: `5.0`) — period between disturbances in seconds.
* **`force_applicator.distance.distance_threshold`** (double, default: `1.0`) — trigger distance for position-based disturbances in m.
* **`force_applicator.seed`** (int, default: `1`) — RNG seed for reproducible random disturbances.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ROS2]: https://docs.ros.org/en/jazzy/
[quad_msgs/RobotState]: ../quad_msgs/msg/RobotState.msg
[visualization_msgs/MarkerArray]: https://docs.ros.org/en/jazzy/p/visualization_msgs/
[ros_gz_interfaces/EntityWrench]: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_interfaces
