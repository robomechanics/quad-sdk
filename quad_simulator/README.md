# Quad Simulator

## Overview

This directory is a meta-package that groups the Mujoco and Gazebo simulation infrastructure and all supported robot descriptions used by Quad-SDK. It contains:

- **`gazebo_plugins/`** — Gazebo system plugins for controller bridging and state estimation injection.
- **`gazebo_scripts/`** — terrain worlds, meshes, and bring-up scripts for Gazebo Harmonic.
- **`*_description/`** — URDF/meshes for each supported platform (see below).

The simulation side of Quad-SDK targets **Gazebo Harmonic** (via `ros_gz_bridge`) on ROS2 Jazzy; legacy Gazebo Classic support has been removed. A MuJoCo back-end is also available — see the top-level `quad_mujoco.py` launch file.

### License

The source code is released under a [MIT License](../LICENSE). The Spirit40 model was originally created by Ghost Robotics. The Gazebo bring-up structure is based on [kodlab_gazebo](https://github.com/KodlabPenn/kodlab_gazebo).

**Original `kodlab_gazebo` author:** Vasileios Vasilopoulos

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

**Maintainer:** Justin Yim (jkyim@andrew.cmu.edu)

Tested under [ROS2] Jazzy on Ubuntu 24.04 with Gazebo Harmonic.

## Supported Robots

| Package | Platform |
|---|---|
| `a1_description` | Unitree A1 |
| `b2_description` | Unitree B2 |
| `go1_description` | Unitree Go1 |
| `go2_description` | Unitree Go2 |
| `go2w_description` | Unitree Go2-W (wheeled) |
| `spirit_description` | Ghost Robotics Spirit40 |
| `spot_description` | Boston Dynamics Spot |
| `vision60_description` | Ghost Robotics Vision60 |

Each description package ships URDF/Xacro files and collision/visual meshes. Friction and inertial parameters for foot contact are tuned in the URDF.

## Subpackages

### `gazebo_plugins`

C++ Gazebo plugins:

* **`controller_plugin`** — bridges between the Quad-SDK control stack and Gazebo's actuator interface, applying joint torques computed by `robot_driver`.
* **`estimator_plugin`** — publishes ground-truth state from the simulator for use as `state/ground_truth` or as a reference for EKF evaluation. Sets `use_sim_time=true` via `parameter_overrides` at Node construction (so `node_->now()` returns sim time from the first call) and skips publishing while the rclcpp clock still reports wall-clock time.
* **`spawn_lock_plugin`** — pins the body link at its spawn pose for `<hold_duration>` seconds (default 8 s) while gz_ros2_control loads its joint controllers, then releases. Joints articulate freely throughout, so per-robot controllers can fold the legs into their commanded sit pose without the body tipping. Releases on whichever clock crosses `hold_duration` first (sim or wall) for resilience against per-instance sim-time stalls.

Plugins are registered via `controller_plugin.xml` and loaded into each simulated model by the URDF/SDF.

### `gazebo_scripts`

Terrain models, worlds, and utility scripts. Available terrain models (under `gazebo_scripts/models/`):

```
flat                 big_flat             slope_20             slope_20_hole
step_10cm            step_15cm_local_min  step_20cm            step_25cm
step_30cm            step_10cm_local_min
gap_20cm             gap_40cm             gap_40cm_local_min   gap_80cm
rough_25cm           rough_40cm_huge      parkour_local_min
```

## Build

```bash
colcon build --packages-select \
  gazebo_plugins gazebo_scripts \
  a1_description b2_description go1_description go2_description go2w_description \
  spirit_description spot_description underbrush_description vision60_description
```

## Usage

Launch Gazebo with the default robot (Go2) and flat terrain:

```bash
ros2 launch quad_utils quad_gazebo.py
```

Launch with MuJoCo:

```bash
ros2 launch quad_utils quad_mujoco.py
```

Robot and terrain selection are set via launch arguments — see `quad_utils/launch/quad_gazebo.py` for the full argument list.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ROS2]: https://docs.ros.org/en/jazzy/
