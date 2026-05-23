# Quad Perf Tests

## Overview

This package provides performance- and regression-testing utilities for Quad-SDK. There are two complementary tools:

1. **`cmd_vel` publisher** — a configurable C++ node that drives the robot through deterministic or randomized velocity trajectories, for benchmarking controller tracking and stress-testing the planning stack.
2. **Batch iteration runner** — a headless launch + Python driver that brings the full sim/plan/control stack up, runs it to a goal (or a timeout), records bags, tears everything down, and repeats N times. Useful for reproducible, automated testing across many randomized runs.

> The batch iteration tooling was previously the standalone `quad_training` package. It was folded in here and repurposed for general iteration testing — it is **not** for generating learned-policy training data.

### License

The source code is released under a [MIT License](../LICENSE).

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

Tested under [ROS2] Jazzy on Ubuntu 24.04.

## Build

```bash
colcon build --packages-select quad_perf_tests
```

The C++ `cmd_vel_publisher_node` and the Python iteration tooling are both built and installed by this single `ament_cmake` package.

## cmd_vel publisher

Run the publisher against a running stack:

```bash
ros2 run quad_perf_tests cmd_vel_publisher_node --ros-args \
    --params-file install/quad_perf_tests/share/quad_perf_tests/config/cmd_vel_publisher.yaml \
    --params-file install/quad_perf_tests/share/quad_perf_tests/config/cmd_vel_publisher_topics.yaml
```

### Config

* **`config/cmd_vel_publisher.yaml`** — mode, velocity bounds, duration, seed.
* **`config/cmd_vel_publisher_topics.yaml`** — topic remappings.

### cmd_vel_publisher_node

Publishes a `geometry_msgs/Twist` command on the `cmd_vel` topic. Three modes are supported:

- **`single`** — sample one random velocity within the configured bounds and hold it for `test_duration` seconds.
- **`timer`** — resample a new random velocity every `resample_sec` seconds.
- **`off`** — publish zero velocity (useful as a keep-alive).

#### Published Topics

* **`cmd_vel`** ([geometry_msgs/Twist]) — commanded body twist.

#### Parameters

* **`cmd_vel_publisher.update_rate`** (double, default: `500.0`) — publish rate in Hz. Must match `robot_driver.update_rate`.
* **`cmd_vel_publisher.mode`** (string, default: `single`) — one of `single`, `timer`, `off`.
* **`cmd_vel_publisher.resample_sec`** (double, default: `3.0`) — resample interval for `timer` mode.
* **`cmd_vel_publisher.bounds.x_{min,max}`** (double) — forward velocity bounds in m/s.
* **`cmd_vel_publisher.bounds.y_{min,max}`** (double) — lateral velocity bounds in m/s.
* **`cmd_vel_publisher.bounds.yaw_{min,max}`** (double) — yaw-rate bounds in rad/s.
* **`cmd_vel_publisher.test_duration`** (double, default: `10.0`) — duration for `single` mode in seconds.
* **`cmd_vel_publisher.seed`** (int, default: `1`) — RNG seed for reproducibility.

## Batch iteration testing

A single headless iteration brings up Gazebo, waits for the robot to settle, stands it, launches the planning stack, records bags, and auto-terminates on goal / planner failure / collision:

```bash
ros2 launch quad_perf_tests run_iteration.py
ros2 launch quad_perf_tests run_iteration.py robot_type:=spirit world:=rough.sdf
```

To run many iterations back-to-back (each fully isolated — processes killed and the ROS 2 daemon reset between runs):

```bash
# 10 iterations of the default robot
ros2 run quad_perf_tests run_iterations --num_episodes 10

# Custom robot/world, 5-minute per-iteration safety timeout
ros2 run quad_perf_tests run_iterations --num_episodes 5 \
    --robot_type spirit --world rough.sdf --timeout 300 --output_dir /data/bags
```

Bags are written to `./iteration_bags` by default (`--output_dir` to override). For each iteration two bags are recorded: a time-synced bundle (`*_synced`) and a full-rate raw bag (`*_raw`).

To record bags against an already-running stack without the full orchestration:

```bash
ros2 launch quad_perf_tests iteration_logging.py
```

### Launch / script files

* **`launch/run_iteration.py`** — one headless iteration: bring up sim, wait for ready, stand, plan, sync + bag, monitor, shut down on termination.
* **`launch/iteration_logging.py`** — sync + bag recording only, for attaching to an existing stack.
* **`scripts/run_iterations.py`** (`ros2 run quad_perf_tests run_iterations`) — runs `run_iteration.py` N times with cleanup and retries between runs.

### Nodes

#### time_sync_node

Subscribes to key simulation topics at their native rates and republishes aligned tuples on `/<ns>/synced/<topic>` using `message_filters.ApproximateTimeSynchronizer`, so they can be bagged as coherent (state, action) bundles. Synchronized topics:

- `state/ground_truth` (`quad_msgs/RobotState`, ~500 Hz)
- `control/joint_command` (`quad_msgs/LegCommandArray`, ~500 Hz)
- `control/grfs` (`quad_msgs/GRFArray`, ~500 Hz)
- `local_plan` (`quad_msgs/RobotPlan`, ~333 Hz)
- `state/estimate` (`quad_msgs/RobotState`, ~500 Hz)

Bundle rate is bounded by the slowest topic and the configured slop tolerance.

#### episode_monitor_node

Watches for three termination conditions and cleanly exits the launch context when any fire:

1. **Goal reached** — boolean on `goal_reached`.
2. **Planner failure** — boolean on `planner_failed`.
3. **Body collision** — roll/pitch exceeds a configured threshold.

After a condition triggers, the node waits a brief settle period to flush the bag, then raises `SystemExit` so the parent `ros2 launch` shuts down the full stack.

#### wait_for_robot_node

Blocks until the simulated robot is fully ready, so an iteration doesn't start capturing spawn-in artifacts. Exits only when **all** of the following hold for a configured consecutive-message count:

1. `state/ground_truth` is being published (Gazebo + driver up).
2. `state/joints` is being published (controllers activated).
3. Robot is upright (roll/pitch within threshold).
4. Robot has landed (vertical velocity near zero).

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ROS2]: https://docs.ros.org/en/jazzy/
[geometry_msgs/Twist]: https://docs.ros.org/en/jazzy/p/geometry_msgs/
