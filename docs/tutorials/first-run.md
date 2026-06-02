---
title: First Simulation Run
tags:
  - tutorial
  - simulation
  - getting-started
---

# First Simulation Run

End-to-end walkthrough: simulator → stand → plan → walk to a clicked goal. Should take ~10 minutes from a clean install.

## What you'll see

By the end of this tutorial:

- A Go2 (or Spirit) standing in Gazebo
- A planning stack producing a global path you can re-target with a click in RViz
- The local planner producing a 26-step body plan + footstep schedule
- The robot walking along the path

## 0. Prep

You should have completed [Installation](../getting-started/installation.md) and `source install/setup.bash` in every terminal.

## 1. Launch the simulator

=== "Gazebo Harmonic"

    ```bash
    ros2 launch quad_utils quad_gazebo.py gui:=true
    ```

=== "MuJoCo"

    ```bash
    ros2 launch quad_utils quad_mujoco.py
    ```

The `quad_gazebo.py` launch file accepts these parameters:

| Param | Default | Purpose |
|---|---|---|
| `robot_configs` | Single Go2 at origin | JSON list of robots and their configs |
| `gui` | `false` | Show the Gazebo viewer |
| `paused` | `false` | Start paused |
| `world` | `flat.sdf` | Pick a world from `quad_simulator/quad_sim_scripts/worlds` |
| `live_plot` | `false` | Auto-launch PlotJuggler |
| `dash` | `false` | Auto-launch the rqt dashboard |
| `logging` | `false` | Record a bag |
| `obstacles` | `[]` | JSON list of static obstacles |
| `scenario` | `none` | Named obstacle scenario |
| `use_sim_time` | `true` | Use Gazebo clock |

Inspect the running processes:

```bash
ros2 node list | grep robot_1
```

## 2. Stand the robot

```bash
ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 1" --once
```

The robot ramps to its nominal stance. If it doesn't:

- Check `ros2 topic echo /robot_1/state/ground_truth --once` — there should be a quaternion + 12 joint angles
- Check `ros2 topic hz /robot_1/joint_states` — should report ≥ 500 Hz
- Drop `stand_kp` in `quad_utils/config/<robot>.yaml` if the robot oscillates

## 3. Launch the planning stack

```bash
ros2 launch quad_utils quad_plan.py
```

Key params:

| Param | Default | Purpose |
|---|---|---|
| `reference` | `gbpl` | Plan source — `gbpl` for global planner, `twist` for `cmd_vel` only |
| `twist_input` | `none` | `keyboard`, `joy`, or none |
| `leaping` | `true` | Allow flight phases in the global plan |
| `logging` | `false` | Record a bag |

You should see in `ros2 topic list`:

- `/robot_1/global_plan`
- `/robot_1/local_plan`
- `/robot_1/foot_plan_continuous`
- `/robot_1/foot_plan_discrete`

## 4. Set a goal

Open RViz (auto-launched by `quad_visualization.py` or run manually with the provided RViz config), then use the **Publish Point** tool. Click somewhere on the terrain and the global planner picks it up via `/clicked_point`.

Alternatively, set a default goal via param:

```bash
ros2 param set /global_body_planner global_body_planner.goal_state "[5.0, 1.0]"
```

## 5. Walk

```bash
ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 2" --once
```

The robot tracks the local plan. To redirect, click a new goal in RViz and the global planner will replan toward it.

## 6. Drive with cmd_vel instead

For a hand-driven test:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/robot_1/cmd_vel
```

Teleop keys:

```
   u i o
   j k l
   m , .
```

- `q`/`z` — increase/decrease speed scale 10%
- `w`/`x` — adjust linear only
- `e`/`c` — adjust angular only

## Troubleshooting

??? warning "Robot collapses on stand"
    The most common cause is gain mismatch. Reduce `stand_kp` in the per-robot YAML. If joints jitter, reduce `stand_kd`.

??? warning "NMPC reports `Restoration_Failed`"
    Reference is infeasible. Check the global plan is reachable, friction `mu` matches the simulation surface, and `cmd_vel_scale` isn't pushing past kinematic limits.

??? warning "RViz crashes when using Publish Point"
    Known NVIDIA driver / interactive-marker issue. Use `xrdp` or set `LIBGL_ALWAYS_SOFTWARE=1` for visualization-only sessions.

??? warning "Robot drifts laterally on stand"
    IMU bias. With the robot at rest, run the calibration step in your platform's bringup sequence.

## Next steps

- [Logging & Playback](logging.md) — record runs, replay, post-process
- [Adding a new robot](adding-a-robot.md) — extend the SDK to a new platform
- [Writing your own controller](writing-controller.md) — drop in a custom `LegController`
