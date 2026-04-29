---
title: Logging & Playback
tags:
  - tutorial
  - logging
  - bag
---

# Logging & Playback

How to record, replay, and post-process robot data with `quad_logger` and `rosbag2`.

## Recording

Add `logging:=true` to any of the canonical launch files:

```bash
ros2 launch quad_utils quad_gazebo.py logging:=true
ros2 launch quad_utils quad_plan.py logging:=true
```

Bags are written to `quad_logger/bags/` as **rosbag2 mcap** directories (not single `.bag` files — that's a ROS 1 holdover).

The default record set covers:

- `/robot_*/state/ground_truth`
- `/robot_*/joint_states`
- `/robot_*/global_plan`
- `/robot_*/local_plan`
- `/robot_*/foot_plan_*`
- `/robot_*/imu/data`
- `/robot_*/cmd_vel`
- `/robot_*/control/mode`
- `/clock`

Tweak the record set in `quad_utils/launch/logging.py`.

## Replay

```bash
ros2 launch quad_utils quad_visualization.py
ros2 bag play quad_logger/bags/<run_name>
```

In RViz, use **Reset** to restart the playback display. Use `--rate 0.5` to slow down playback for diagnostics.

## MATLAB post-processing

Quad-SDK ships a MATLAB analysis pipeline under `quad_logger/scripts/` that ingests rosbag2 directories and produces trajectory/contact/torque plots.

**One-time message generation:**

```matlab
ros2genmsg('~/ros2_ws/src/quad-sdk')
rehash toolboxcache
clear
```

**Run the analysis:**

```matlab
% In quad_logger/scripts/
processLog(<bag_name>)
```

Or interactively edit `processLog.m`:

| Variable | Purpose |
|---|---|
| `trialName` | Bag directory; leave blank for a file picker |
| `namespace` | `robot_1` (or whatever you set) |
| `bAnimate` | Toggle 3-D body-frame animation |
| `bSave` | Save data + figures into `quad_logger/logs/` |

For headless runs:

```bash
matlab -nodesktop -batch "processLog('<bag_name>')"
```

## Python post-processing

For lighter-weight inspection, the Python helpers under `quad_logger/scripts/` extract a tidy `pandas.DataFrame` per topic — handy for notebooks and quick plots.

## Blender visualization

The `scripts/` package converts a bag into an animated Blender scene + MP4. Useful for paper figures and demo videos. See the [scripts](../packages/scripts.md) page.

## Bag layout

A typical run produces:

```
quad_logger/bags/<run_name>/
├── metadata.yaml
└── <run_name>_0.mcap
```

`mcap` is the default storage format — readable by `ros2 bag info`, `mcap-cli`, Foxglove, PlotJuggler.

## Tips

!!! tip "Disk-conscious recording"
    Long hardware runs balloon quickly. Strip the heaviest topics (e.g. raw camera) by editing `quad_utils/launch/logging.py`.

!!! tip "Sim time vs. wall time"
    With `use_sim_time:=true` the bag carries Gazebo time — replay must use `--clock` if downstream nodes assume sim clock.

!!! warning "ROS 1 bag compatibility"
    Old `.bag` files from the ROS 1 stack are not directly playable. Convert with `rosbags-convert` if you need to revisit them.
