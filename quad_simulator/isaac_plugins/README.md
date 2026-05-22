# Isaac Plugins

## Overview

This package is the Isaac Sim 5.1 integration for Quad-SDK — an alternative to the Gazebo simulator used elsewhere in the stack. It contains the Isaac-side simulation loop (`isaac_bridge.py`), the URDF prep pipeline that adapts Quad-SDK robot descriptions for Isaac's URDF importer, the four-cord underbrush scenario, and one-shot URDF→USD conversion tooling. The bridge exchanges ROS 2 messages with the rest of Quad-SDK using Isaac Sim's bundled Jazzy rclpy, so the controller and planning stacks run unmodified.

Supported robots are listed in `isaac_bridge.py`'s `ROBOT_REGISTRY` — currently `spirit` and `go2`. Supported terrains are any name with matching `gazebo_scripts/models/<name>/meshes/<name>.{stl,ply}` (e.g. `flat`, `step_20cm`, `gap_40cm`, `rough_25cm`).

### License

The source code is released under a [BSD-3-Clause License](LICENSE).

**Author:** David Ologan

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

**Maintainer:** David Ologan (ologandavid@gmail.com)

Tested under Isaac Sim 5.1 / [IsaacLab] (Python 3.11, conda) and [ROS2] Jazzy (Python 3.12, system) on Ubuntu 24.04. This is research code; expect frequent changes and no fitness for any particular purpose.

## Build

```bash
colcon build --packages-select isaac_plugins
```

The package itself only installs Python scripts. Isaac Sim 5.1 and IsaacLab must be installed separately in a conda environment named `isaaclab`; see the [IsaacLab installation guide][IsaacLab-install]. The bridge auto-discovers Isaac's bundled rclpy and prepends the bundled DDS libs to `LD_LIBRARY_PATH` at startup — do *not* source `/opt/ros/jazzy/setup.bash` in the same shell as the bridge.

## Usage

Three terminals; `--terrain` and `--robot` must match between the bridge and the bringup.

```bash
# Terminal 1 — Isaac sim + bridge (conda isaaclab env, Python 3.11)
conda activate isaaclab
~/ros2_ws/src/quad-sdk/quad_simulator/isaac_plugins/scripts/run_isaac_bridge.sh \
    --robot spirit --terrain rough_25cm --spawn-z 0.7

# Terminal 2 — ROS bringup (terrain heightmap + rsp + robot_driver + RViz)
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch quad_utils quad_isaac_bringup.py \
    namespace:=robot_1 robot_type:=spirit terrain:=rough_25cm rviz:=true

# Terminal 3 — planning (gbpl + local_planner + body_force_estimator)
ros2 launch quad_utils quad_isaac_planning.py \
    namespace:=robot_1 robot_type:=spirit

# Sit → stand → walk
ros2 topic pub --once /robot_1/control/control_mode std_msgs/msg/UInt8 'data: 1'
sleep 3
ros2 topic pub --once /robot_1/control/control_mode std_msgs/msg/UInt8 'data: 2'
ros2 topic pub --rate 10 /robot_1/control/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.3}, angular: {z: 0.0}}'
```

Common bridge flags: `--cinematic` (follow camera + three-point lighting), `--scene underbrush` (four-cord vine scenario), `--physx-gpu` (GPU dynamics — off by default), `--tau-scale 0.5` (halve applied torques; useful when gains tuned for hardware over-drive in PhysX). Full list: `run_isaac_bridge.sh --help`.

## Architecture

```
┌─ Terminal 1: Isaac + bridge (conda Python 3.11) ─────────┐
│   isaac_bridge.py                                         │
│   ├─ Isaac Sim 5.1 physics @ 1000 Hz                      │
│   ├─ Robot URDF imported at runtime                       │
│   ├─ Terrain STL with SDF pose applied                    │
│   └─ rclpy node using Isaac's bundled Jazzy               │
└──────────────────────────────────────────────────────────┘
                          ↑ /<ns>/state/ground_truth
                          ↓ /<ns>/control/joint_command
                          (DDS / FastRTPS)
┌─ Terminal 2: ROS bringup (system Python 3.12, ROS2 Jazzy)┐
│   robot_driver_node       — leg-command controller        │
│   robot_state_publisher   — kinematic TF                  │
│   mesh_to_grid_map_node   — PLY → /mapping/terrain_map_*  │
│   grid_map_filters_demo   — derived layers (z_inpainted…) │
│   rviz_interface_node     — RobotState → TF + joint_viz   │
│   ground_truth_state_publisher                            │
│   terrain_map relay       — /mapping/* → /<ns>/*          │
│   (optional) rviz2                                        │
└──────────────────────────────────────────────────────────┘
┌─ Terminal 3: planning (system Python 3.12) ──────────────┐
│   global_body_planner_node                                │
│   local_planner_node                                      │
│   body_force_estimator_node                               │
└──────────────────────────────────────────────────────────┘
```

The bridge and the rest of the stack run in different Python versions (3.11 inside Isaac's conda env vs system 3.12) but exchange ROS 2 messages over DDS. `quad_msgs` Python bindings are pure-Python and are ABI-compatible across both.

## Scripts

| Script | Role |
|---|---|
| `scripts/isaac_bridge.py` | Isaac-side bridge — sim loop, URDF import, articulation setup, ROS 2 pub/sub. |
| `scripts/run_isaac_bridge.sh` | Env wrapper — strips system ROS 2 from `LD_LIBRARY_PATH`, prepends Isaac's bundled DDS libs, then `exec`s `isaaclab.sh -p isaac_bridge.py`. |
| `scripts/resolve_package_urls.py` | URDF prep — expand `package://` URIs to absolute paths. |
| `scripts/rename_joints.py` | URDF prep — rename numeric joint names to `j_{abad,hip,knee}_{0..3}`. |
| `scripts/strip_gazebo_tags.py` | URDF prep — drop `<gazebo>`, `<transmission>`, `<ros2_control>`, repair missing/zero-mass `<inertial>` blocks, strip `dont_collapse`. |
| `scripts/spawn_vines.py` | Compliant-cord vine spawner for the underbrush scenario (chains of spherical-joint cylinders pinned to kinematic anchors). |
| `scripts/flat_terrain_publisher.py` | Fallback flat `GridMap` publisher used when `--terrain` is unset. |
| `scripts/convert_urdf_to_usd.py`, `scripts/import_urdf_to_usd.py` | One-shot URDF→USD conversion tools (not used at runtime). |

The companion launch files in `quad_utils` are `quad_isaac_bringup.py` (terrain + robot_state_publisher + robot_driver + viz) and `quad_isaac_planning.py` (global / local planner + BFE).

## Status

Implemented:

- Bridge runs at 1000 Hz with PhysX TGS, 64/16 solver iters.
- Multi-robot via `ROBOT_REGISTRY` (currently `spirit`, `go2`).
- Multi-terrain via `--terrain <name>` matching `gazebo_scripts/models/`.
- Terrain pose pulled from matching world `.sdf` `<include><pose>`.
- Real heightmap to the planner via `mesh_to_grid_map_node` (same path Gazebo uses).
- TF chain populated by `rviz_interface_node` + ground-truth `robot_state_publisher` consuming the renamed clean URDF.
- Body angular velocity rotated into body frame (matches Gazebo's convention).
- Sit pose authored via `PhysicsJointStateAPI` pre-`world.reset()`; spawn translate applied to the articulation root prim before reset.
- Toe contact heuristic and foot states wired into `FootState`.
- RViz toggle on the bringup (`rviz:=true`); underbrush via `--scene underbrush`.

Known limitations:

- `go2` controller saturates motor envelopes during stance/swing on PhysX with hardware-tuned gains (kp=60). Workaround: pass `--tau-scale 0.5` to the bridge, or temporarily lower `stance_kp` / `swing_kp` in `quad_utils/config/go2.yaml` to ~30 for sim-only runs. Proper fix is on the roadmap (joint armature + gain re-tune).
- Auto-recording (`--record`) silently no-ops on Isaac 5.1's `omni.kit.capture.viewport` API. Use OBS or Movie Capture from the Isaac GUI instead.
- Toe contact reporting is a `foot_z < 0.04` heuristic; the controller side sees no real per-toe GRFs in sim.

## Roadmap

Open items in rough priority order. Each is well-scoped enough to be a single PR.

| # | Task | Effort | Notes |
|---|---|---:|---|
| 1 | Joint armature in Isaac (reflected motor inertia) | ~25 LOC | Walk imported joints post-import, apply `PhysxSchema.PhysxJointAPI.armature`. Per-robot table in the `ARMATURE_TABLE` of `convert_urdf_to_usd.py`. Likely fixes the controller over-driving torques on PhysX. |
| 2 | Real contact reporting | ~½ day | Replace the `foot_z < 0.04` heuristic with `omni.physx.get_physx_interface().get_contact_pairs()` per toe. Publish proper `GRFArray` on `/<ns>/state/grfs`. |
| 3 | Tune controller gains for PhysX | ~1 day | Inverse-dynamics over-drives ~2× on PhysX vs ODE. Re-tune `stand_kp/kd`, `stance_kp/kd` per robot after #1 lands. |
| 4 | Hip-effort sign flip (paired report + apply) | ~10 LOC | Match Gazebo's `estimator_plugin.cpp` `case 1: -torque_msg.y()` convention. Only revisit once #1/#3 are in. |
| 5 | Portable paths | ~2 hrs | `run_isaac_bridge.sh` hardcodes `/home/rml/anaconda3/envs/isaaclab/...`; replace with programmatic discovery. Same for `WORKSPACE_INSTALL` in the bridge. |
| 6 | Procedural / additional vine fields | ~1 day | Generalize `spawn_vines.py` beyond the 4-cord ROS1 scenario. |
| 7 | Auto-record viewport (MP4) | ~1 day | Re-plumb against a working capture API. |
| 8 | Isaac 6 port | weeks | Isaac 6's `URDFImporter` is class-based, no `omni.kit.commands` registration, no per-import physics knobs; physics setup moves to a post-import USD pass. |

## Background — Gazebo vs Isaac physics

The robot stands and walks on rough terrains in Gazebo. The same world, robot, and controller can saturate on Isaac. Ranked causes, most → least likely:

1. **Mesh-collider behavior.** ODE silently falls back to convex hulls / heightfields on concave meshes; PhysX honors every triangle. The bridge sets `MeshCollisionAPI.approximationAttr = 'none'` (faithful); convex hull spans baseplate-to-peak, decomposition was no better. A heightfield collider is the proper fix.
2. **Missing joint armature.** Reflected motor inertia `J_rotor * N²` (~4.4e-3 kg·m² abad/hip, 1.77e-2 kg·m² knee for spirit). Gazebo reads URDF `<armature>`; Isaac's runtime importer doesn't. Inverse-dynamics M(q) is off by this amount and feed-forward overshoots.
3. **Physics step + solver iters.** Gazebo ODE runs 300 iters; Isaac PhysX 64/16 (raised from defaults 16/4). Closes most of the gap.
4. **Contact reporting.** Gazebo's `ContactStatePublisher` emits real per-toe GRFs; Isaac uses a `foot_z < 0.04` heuristic that's wrong on non-flat ground.
5. **PhysX contact / rest offset.** PhysX adds a ~2 cm skin around shapes; on bumpy meshes the effective contact height shifts upward. ODE has no equivalent.
6. **Joint damping defaults.** Unspecified URDF `<dynamics damping>` defaults to 0 in ODE, ~0.1 in PhysX. Matters for swing-phase PD.
7. **Hip-effort sign convention.** Gazebo's plugin negates the hip y-axis torque on state report; the bridge currently publishes raw URDF-axis values.
8. **Friction combine mode.** ODE uses `min(μ₁, μ₂)`. PhysX defaults to `average`; IsaacLab uses `multiply`. Negligible at μ=100.

Items 1–4 are the live ones; 5–8 are kept here as a checklist for future regressions.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ROS2]: https://docs.ros.org/en/jazzy/
[IsaacLab]: https://isaac-sim.github.io/IsaacLab/
[IsaacLab-install]: https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html
