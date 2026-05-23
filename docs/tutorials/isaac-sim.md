---
title: Running in Isaac Sim
tags:
  - tutorial
  - simulation
  - isaac
---

# Running in Isaac Sim

NVIDIA Isaac Sim 5.1 is an alternative simulation backend to Gazebo and MuJoCo, adding high-fidelity rendering and GPU-accelerated PhysX dynamics. The controller and planning stacks run **unmodified** — the bridge just exchanges the same ROS 2 messages over DDS.

!!! warning "Beta integration"
    The Isaac Sim backend is **still in beta**. It currently supports the **Spirit 40** and **Go2** only, contact reporting is a heuristic, and PhysX gains may need scaling (`--tau-scale`). Expect rough edges and frequent changes. For production runs use Gazebo or MuJoCo. Full status, known limitations, and roadmap live in [`quad_simulator/isaac_plugins/README.md`](https://github.com/robomechanics/quad-sdk/blob/main/quad_simulator/isaac_plugins/README.md).

## Requirements

Isaac Sim is **not** part of the Quad-SDK Docker images — it needs a separate install.

- **Isaac Sim 5.1 + IsaacLab** in a conda environment named `isaaclab` (Python 3.11). Follow the official [IsaacLab installation guide :octicons-link-external-16:](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html).
- **ROS 2 Jazzy** (system Python 3.12) — the rest of the Quad-SDK stack, as usual.
- Ubuntu 24.04, an NVIDIA GPU with current drivers.

!!! note "Two Python runtimes, on purpose"
    The bridge runs inside Isaac's conda env (Python 3.11) while the controller/planner run on system ROS 2 Jazzy (Python 3.12). They talk over DDS. **Do not** `source /opt/ros/jazzy/setup.bash` in the same shell as the bridge — it auto-discovers Isaac's bundled rclpy and DDS libs.

## Build

The package itself only installs Python scripts:

```bash
cd ~/ros2_ws
colcon build --packages-select isaac_plugins
source install/setup.bash
```

## Running

Three terminals. `--terrain` and `--robot` must match between the bridge (terminal 1) and the bringup (terminal 2).

=== "1. Isaac sim + bridge"

    Conda `isaaclab` env (Python 3.11) — **no** ROS 2 sourced here:

    ```bash
    conda activate isaaclab
    ~/ros2_ws/src/quad-sdk/quad_simulator/isaac_plugins/scripts/run_isaac_bridge.sh \
        --robot spirit --terrain rough_25cm --spawn-z 0.7
    ```

=== "2. ROS bringup"

    System ROS 2 Jazzy (Python 3.12) — terrain heightmap + robot_state_publisher + robot_driver + RViz:

    ```bash
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch quad_utils quad_isaac_bringup.py \
        namespace:=robot_1 robot_type:=spirit terrain:=rough_25cm rviz:=true
    ```

=== "3. Planning"

    Global + local planner + body force estimator:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch quad_utils quad_isaac_planning.py \
        namespace:=robot_1 robot_type:=spirit
    ```

Then sit → stand → walk:

```bash
# Stand
ros2 topic pub --once /robot_1/control/control_mode std_msgs/msg/UInt8 'data: 1'
sleep 3
# Walk
ros2 topic pub --once /robot_1/control/control_mode std_msgs/msg/UInt8 'data: 2'
# Drive
ros2 topic pub --rate 10 /robot_1/control/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.3}, angular: {z: 0.0}}'
```

## Supported robots and terrains

- **Robots** — `spirit` and `go2` (the entries in the bridge `ROBOT_REGISTRY`).
- **Terrains** — any name with a matching mesh under `gazebo_scripts/models/<name>/meshes/<name>.{stl,ply}`, e.g. `flat`, `step_20cm`, `gap_40cm`, `rough_25cm`. With `--terrain` unset, a flat `GridMap` fallback is published.

## Useful bridge flags

| Flag | Effect |
|---|---|
| `--cinematic` | Follow camera + three-point lighting |
| `--scene underbrush` | Four-cord vine entanglement scenario |
| `--physx-gpu` | GPU dynamics (off by default) |
| `--tau-scale 0.5` | Halve applied torques — helps when gains are tuned for hardware over-drive in PhysX |

Full list: `run_isaac_bridge.sh --help`.

## Known limitations (beta)

- **Go2 torque saturation** — hardware-tuned gains (`kp=60`) over-drive on PhysX. Workaround: `--tau-scale 0.5`, or temporarily lower `stance_kp`/`swing_kp` in `quad_utils/config/go2.yaml` to ~30 for sim-only runs.
- **Contact reporting** — a `foot_z < 0.04` heuristic, not real per-toe GRFs.
- **Auto-record** (`--record`) no-ops on Isaac 5.1's capture API; use OBS or the Isaac GUI's Movie Capture.

See the [package README](https://github.com/robomechanics/quad-sdk/blob/main/quad_simulator/isaac_plugins/README.md) for the full physics deep-dive (Gazebo ODE vs. Isaac PhysX) and roadmap.

## Next steps

- :material-table: [Simulator support matrix](../platforms.md#simulator-support) — which robot runs in which backend
- :material-rocket: [First Simulation Run](first-run.md) — the Gazebo/MuJoCo walkthrough
