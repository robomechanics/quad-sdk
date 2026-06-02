# MuJoCo Plugins

## Overview

This package is the MuJoCo-side ground-truth state estimator for Quad-SDK — the MuJoCo counterpart to `gazebo_plugins`' `estimator_plugin`. It subscribes to the simulator's floating-base odometry and joint states (published by `mujoco_ros2_control`), computes world-frame and body-frame `RobotState` messages, and publishes them at 500 Hz on `/<ns>/state/ground_truth` and `/<ns>/state/ground_truth_body_frame`. Foot positions and velocities are derived via QuadKD2 forward kinematics from the latched `robot_description`.

The package contains a single executable, `mujoco_estimator`. It's launched once per robot by `quad_utils/launch/quad_mujoco_bringup.py`, with the per-robot joint mapping pulled from `quad_utils/config/<robot_type>.yaml`.

### License

The source code is released under a [MIT License](LICENSE).

**Author:** David Ologan

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

**Maintainer:** David Ologan (ologandavid@gmail.com)

Tested under [ROS2] Jazzy on Ubuntu 24.04 with `mujoco_vendor` + `mujoco_ros2_control` (apt-installed by `setup.sh`). This is research code; expect frequent changes and no fitness for any particular purpose.

## Build

```bash
colcon build --packages-select mujoco_plugins
```

Build deps: `rclcpp`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `quad_msgs`, `quad_utils`, and `pinocchio`. The `quad_utils` dep brings in `QuadKD2`, which builds the leg kinematics model from the latched `robot_description` topic at startup.

## Usage

The estimator is normally launched implicitly by the top-level MuJoCo bringup:

```bash
ros2 launch quad_utils quad_mujoco.py
```

`quad_mujoco_bringup.py` spawns one `mujoco_estimator` per robot under the robot's namespace. To run standalone:

```bash
ros2 run mujoco_plugins mujoco_estimator --ros-args \
    --params-file ~/ros2_ws/install/quad_utils/share/quad_utils/config/go2.yaml
```

## Architecture

```
┌─ MuJoCo simulator (mujoco_ros2_control / ros2_control_node) ─┐
│                                                              │
│   mjData ──► /robot_1/odom         (free-joint pose + twist) │
│          ──► /robot_1/joint_states (12-DOF)                  │
│          ──► /robot_1/robot_description (latched URDF)       │
└──────────────────────────┬───────────────────────────────────┘
                           ↓
                ┌──────────────────────┐
                │  mujoco_estimator    │
                │   500 Hz publish     │
                │   - QuadKD2 FK       │
                │   - body/world frame │
                │   - joint remap      │
                └──────────┬───────────┘
                           ↓
   /robot_1/state/ground_truth             (world-frame pose + twist, feet)
   /robot_1/state/ground_truth_body_frame  (body-frame velocities, feet)
```

The publisher gates on having received the latched `robot_description`, the most recent odometry, and the most recent joint state. QuadKD2 must be primed by `updateDynamics(state)` each tick before `fkRobotState(state)` — both are called inline in `publishState()`.

## Notes

- **Floating-base convention.** MuJoCo's free joint publishes world-frame linear velocity in `qvel[0:3]` and body-frame angular velocity in `qvel[3:6]`. The estimator preserves the Gazebo plugin's convention: on `/state/ground_truth`, `body.twist.linear` is world-frame, `body.twist.angular` is body-frame. The companion `_body_frame` topic puts both linear and angular into the body frame and zeroes the body pose.
- **Joint remap.** MuJoCo publishes joints in MJCF order; Quad-SDK consumes them in the order configured by `leg_<n>.joints.<slot>.name` in the per-robot YAML. The estimator builds this remap on construction. A fallback default order (`{8,0,1,9,2,3,10,4,5,11,6,7}`) covers robots whose YAML doesn't yet expose those param names.
- **Foot velocity.** Foot positions come from QuadKD2 FK; foot velocity comes from the Jacobian * `qd` chain. The body-frame publisher subtracts the body twist and `ω×r` so values are expressed relative to the moving body.
- **No contact reporting.** MuJoCo doesn't publish per-toe contact topics in the same shape as Gazebo, so the contact-state side of the stack continues to use `gazebo_scripts/contact_state_publisher_node` (a no-op on MuJoCo for now). Real per-toe GRF reporting on MuJoCo is a future item.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ROS2]: https://docs.ros.org/en/jazzy/
