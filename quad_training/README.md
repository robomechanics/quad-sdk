# Quad Training

## Overview

This package provides the data-collection infrastructure for training learned policies on Quad-SDK. It handles time-synchronization of asynchronous simulation topics, episode lifecycle management, and a ready-gate that ensures the robot is fully initialized before data recording begins. Together these produce clean (state, action) rollouts suitable for offline RL, behavioral cloning, or dynamics model learning.

### License

The source code is released under a [MIT License](../LICENSE).

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

Tested under [ROS2] Jazzy on Ubuntu 24.04.

## Build

This is an `ament_python` package.

```bash
colcon build --packages-select quad_training
```

## Usage

End-to-end data collection loop (spawns robot, waits for ready, issues commands, records synced bag, exits on termination):

```bash
ros2 launch quad_training data_collection.py
```

Lightweight variant that only launches the time-sync + bag recorder against an already-running stack:

```bash
ros2 launch quad_training training_logging.py
```

## Nodes

### time_sync_node

Subscribes to key simulation topics at their native rates and republishes aligned tuples on `/synced/<topic>` using `message_filters.ApproximateTimeSynchronizer`. The synchronized bundles are bagged as coherent (state, action) pairs for offline training.

Synchronized topics:

- `state/ground_truth` (`quad_msgs/RobotState`, ~500 Hz)
- `control/joint_command` (`quad_msgs/LegCommandArray`, ~500 Hz)
- `control/grfs` (`quad_msgs/GRFArray`, ~500 Hz)
- `local_plan` (`quad_msgs/RobotPlan`, ~333 Hz)
- `state/estimate` (`quad_msgs/RobotState`, ~500 Hz)

Bundle rate is bounded by the slowest topic and the configured slop tolerance.

### episode_monitor_node

Watches for three termination conditions and cleanly exits the launch context when any fire:

1. **Goal reached** — boolean on `goal_reached`.
2. **Planner failure** — boolean on `planner_failed`.
3. **Body collision** — roll/pitch exceeds a configured threshold.

After a condition triggers, the node waits a brief settle period to flush the bag, then raises `SystemExit` so the parent `ros2 launch` shuts down the full stack.

### wait_for_robot_node

Blocks until the simulated robot is fully ready, so downstream data collection doesn't capture spawn-in artifacts. Exits only when **all** of the following hold for a configured consecutive-message count:

1. `state/ground_truth` is being published (Gazebo + driver up).
2. `state/joints` is being published (controllers activated).
3. Robot is upright (roll/pitch within threshold).
4. Robot has landed (vertical velocity near zero).

## Launch Files

* **`launch/data_collection.py`** — full rollout: bring up sim, wait for ready, start sync + bag, monitor episode, shutdown on termination.
* **`launch/training_logging.py`** — sync + bag only, for attaching to an existing stack.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ROS2]: https://docs.ros.org/en/jazzy/
