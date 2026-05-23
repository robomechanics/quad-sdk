---
title: Batch Iteration Testing
tags:
  - tutorial
  - testing
  - simulation
---

# Batch Iteration Testing

`quad_perf_tests` ships a headless batch runner for executing the full sim → plan → control stack many times in a row — useful for regression and robustness testing across randomized runs. Each iteration brings everything up, drives the robot to a goal, records bags, tears the stack down, and the runner moves on to the next.

!!! note "Repurposed from quad_training"
    This tooling was previously the standalone `quad_training` package. It now lives in [`quad_perf_tests`](../packages/quad_perf_tests.md) and is meant for **iteration / regression testing**, not for generating learned-policy training data.

## What one iteration does

`launch/run_iteration.py` orchestrates a single headless run with event-driven sequencing:

1. **Gazebo (headless)** + robot spawn in a sitting pose.
2. **`wait_for_robot_node`** blocks until the robot is up, controllers are active, and it has settled upright.
3. **Stand** command (`control/mode = 1`).
4. **Planning stack** (`quad_plan.py`: global planner, local planner / NMPC, body force estimator).
5. **`time_sync_node`** + **bag recording** (`iteration_logging.py`).
6. **`episode_monitor_node`** auto-terminates the run on goal reached, planner failure, or a body collision, then shuts the whole launch down cleanly.

Run a single iteration directly:

```bash
ros2 launch quad_perf_tests run_iteration.py
ros2 launch quad_perf_tests run_iteration.py robot_type:=spirit world:=rough.sdf
```

| Arg | Default | Purpose |
|---|---|---|
| `robot_type` | `go2` | Robot config (`go2`, `spirit`, `a1`, …) |
| `world` | `flat.sdf` | Gazebo world |
| `plan_delay` | `5.0` | Seconds after stand before launching the planners |
| `output_dir` | `./iteration_bags` | Where bags are written |

## Running many iterations

`scripts/run_iterations.py` drives `run_iteration.py` N times, fully isolating each run — it kills lingering processes and resets the ROS 2 daemon between iterations, and retries failed runs.

```bash
# 10 iterations of the default robot
ros2 run quad_perf_tests run_iterations --num_episodes 10

# Custom robot/world with a 5-minute per-iteration safety timeout
ros2 run quad_perf_tests run_iterations --num_episodes 5 \
    --robot_type spirit --world rough.sdf --timeout 300 --output_dir /data/bags
```

| Flag | Default | Purpose |
|---|---|---|
| `--num_episodes` | *(required)* | Number of iterations to run |
| `--timeout` | `300` | Per-iteration safety timeout (s) |
| `--robot_type` | `go2` | Robot config |
| `--world` | `flat.sdf` | Gazebo world |
| `--plan_delay` | `5.0` | Seconds after stand before planning |
| `--output_dir` | `./iteration_bags` | Bag output directory |
| `--max_retries` | `3` | Retries per iteration before skipping |

## What gets recorded

Each iteration produces two bags under `output_dir`:

- **`*_synced`** — time-aligned bundles from `time_sync_node` (`state/ground_truth`, `control/joint_command`, `control/grfs`, `local_plan`, `state/estimate`).
- **`*_raw`** — all full-rate topics (states, plans, foot plans, body-force, `cmd_vel`, terrain map) for richer post-hoc inspection.

Post-process with the [Quad Logger](../packages/quad_logger.md) tools.

To record against an already-running stack without the full orchestration:

```bash
ros2 launch quad_perf_tests iteration_logging.py
```

## Driving with the cmd_vel publisher

For tracking/stress tests that don't need the goal-seeking planner, the `cmd_vel_publisher_node` feeds deterministic or randomized velocity commands instead. See the [`quad_perf_tests`](../packages/quad_perf_tests.md) reference.

## Troubleshooting

??? warning "Iterations hang on startup or controllers never activate"
    Stale Gazebo transport state from a previous run. The runner clears `/tmp/gz-*` and restarts the ROS 2 daemon between iterations — if you launched `run_iteration.py` directly, kill leftover `gz sim` processes manually first.

??? warning "Every iteration times out"
    The robot isn't reaching the goal. Check `plan_delay` is long enough for the planners to come up, and confirm the world/robot combination is feasible by running it once interactively with a GUI.

??? warning "`run_iterations` counts a failed run as done"
    After `--max_retries` attempts an iteration is counted anyway to avoid an infinite loop. Inspect that run's logs/bags to see why it failed.
