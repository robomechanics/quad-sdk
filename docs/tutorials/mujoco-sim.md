---
title: Running in MuJoCo
tags:
  - tutorial
  - simulation
  - mujoco
---

# Running in MuJoCo

MuJoCo is a first-class simulation backend for Quad-SDK alongside Gazebo Harmonic. The controller and planning stacks run **unmodified** — `mujoco_ros2_control` exposes the same `controller_manager` interface Gazebo does, and the [`mujoco_estimator`](../packages/mujoco_plugins.md) node publishes the same `RobotState` topics the rest of the stack expects.

## Requirements

MuJoCo support is installed by Quad-SDK's `setup.sh` (apt-installs `ros-jazzy-mujoco-vendor` and `ros-jazzy-mujoco-ros2-control`). No conda env, no extra repo.

- **ROS 2 Jazzy** on Ubuntu 24.04
- A Quad-SDK workspace built with `colcon build` (in particular `mujoco_plugins`, `quad_sim_scripts`, and `quad_utils`)

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select quad_sim_scripts mujoco_plugins quad_utils
source install/setup.bash
```

## Running

A single launch file starts the MuJoCo viewer, the per-robot controller manager, robot driver, terrain-perception bridge, and ground-truth estimator:

```bash
ros2 launch quad_utils quad_mujoco.py
```

Stand and walk (in a second terminal):

```bash
ros2 topic pub --once /robot_1/control/mode std_msgs/msg/UInt8 'data: 1'
sleep 3
ros2 topic pub --once /robot_1/control/mode std_msgs/msg/UInt8 'data: 2'
ros2 launch quad_utils quad_plan.py
```

## Launch arguments

| Arg | Default | Purpose |
|---|---|---|
| `world` | `flat.xml` | MJCF world file from `quad_sim_scripts/worlds`. Procedural worlds are generated from `<world>.xml.xacro` templates with the per-robot MJCF / meshes / terrain stitched in at launch time. |
| `gui` | `true` | Launch the MuJoCo viewer (set `false` for headless) |
| `paused` | `false` | Start the sim paused |
| `robot_configs` | Single Go2 at `[0, 0, 5]` | JSON list of robots and their configs (see [Multi-robot](#multi-robot) below) |
| `recording` | `false` | Record an MP4 of the scene to `quad_logger/logs/mujoco_<robot>_<stamp>.mp4` via the offscreen `mujoco_recorder` node. Requires `ffmpeg`. |
| `live_plot` | `false` | Auto-launch PlotJuggler |
| `dash` | `false` | Auto-launch the rqt dashboard |
| `use_sim_time` | `true` | Use the MuJoCo clock for all nodes |

## Supported robots and terrains

- **Robots** — `spirit`, `spirit_rotors`, `a1`, `a2`, `go1`, `go2`, `go2w`, `b2`, `spot` (see the `_ROBOT_FILES` table in `quad_mujoco_bringup.py`).
- **Terrains** — any world with a matching `<world>.xml.xacro` template under `quad_sim_scripts/worlds/`. Heightmap-based worlds (`rough_*`, `slope_*`) consume an optional `<world>.{bin,png}` from the matching `models/<world>/meshes/` directory; flat-style worlds ignore it.

## Multi-robot

```bash
ros2 launch quad_utils quad_mujoco.py robot_configs:='[
  {"name": "robot_1", "type": "spirit", "controller": "inverse_dynamics", "init_pose": "-x 0.0 -y 0.0 -z 5"},
  {"name": "robot_2", "type": "go2",    "controller": "inverse_dynamics", "init_pose": "-x 2.0 -y 0.0 -z 5"}
]'
```

Each robot gets its own namespace, its own `controller_manager`, and its own `mujoco_estimator`. The first robot's MJCF is the one stitched into the world; multiple robot bodies in a single MuJoCo instance require a custom world template.

## Architecture

```
┌─ MuJoCo + mujoco_ros2_control ───────────────────────────────┐
│   ros2_control_node hosts the simulator                       │
│   - loads <world>.xml (xacro-processed from <world>.xml.xacro)│
│   - loads per-robot <robot>.xml as <include>                  │
│   - exposes joint_state_broadcaster + joint_controller        │
└────────────────────────────┬──────────────────────────────────┘
                             │ /tf, /joint_states, /odom
                             ↓
┌─ Per-robot ROS bringup (quad_mujoco_bringup.py) ─────────────┐
│   robot_state_publisher    URDF → /tf                         │
│   robot_driver_node        leg-command controller             │
│   mujoco_estimator         /odom + /joint_states → /state/gt  │
│   visualization_plugins                                       │
└──────────────────────────────────────────────────────────────┘
┌─ Terrain perception (mujoco_mapping.py) ─────────────────────┐
│   mjcf_to_grid_map_node    parses MJCF terrain → GridMap      │
│   grid_map_filters_demo    derived layers                     │
└──────────────────────────────────────────────────────────────┘
```

The world MJCF is generated per-launch by `prepare_world` in `quad_mujoco.py`: xacro is invoked with the robot's MJCF include, its mesh dir, and the terrain mesh / heightmap path, producing a single `/tmp/_quad_world_*.xml` that `mujoco_ros2_control` loads. Spawn pose comes from the robot config's `init_pose` — `_patch_mjcf_keyframe` rewrites the `<key name="home">` `qpos` of the per-robot MJCF so the controller initial state matches.

For implementation details of the ground-truth state path see the [`mujoco_plugins` package page](../packages/mujoco_plugins.md).

## Recording

```bash
ros2 launch quad_utils quad_mujoco.py recording:=true
```

The offscreen `mujoco_recorder` node shadow-loads the same MJCF, mirrors live state from `/<ns>/odom` and `/<ns>/joint_states` into its own `mjData`, and encodes frames directly to MP4 via `libmujoco` + GLFW + ffmpeg. No screen capture, no display window, no race with the live viewer. Output lands at `<quad_logger src>/logs/mujoco_<robot>_<timestamp>.mp4`.

## Next steps

- :material-rocket: [First Simulation Run](first-run.md) — the side-by-side Gazebo / MuJoCo walkthrough
- :material-table: [Simulator support matrix](../platforms.md#simulator-support) — which robot runs in which backend
- :material-package: [`mujoco_plugins` package page](../packages/mujoco_plugins.md) — estimator implementation details
