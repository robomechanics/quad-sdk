---
title: Training Data Collection
tags:
  - tutorial
  - training
  - reinforcement-learning
---

# Training Data Collection

`quad_training` provides time-synchronized episode management for reinforcement-learning and behavior-cloning pipelines. This page covers the data-collection rig; policy deployment is covered under [Specialized Controllers](specialized-controllers.md).

## What it does

- **Episode boundaries** — automatic reset when the robot falls, exits the world, or hits a step budget
- **Time-synced channels** — joint states, body state, contacts, IMU, and command interleaved at a fixed cadence
- **Curriculum hooks** — start poses, target velocities, and terrain are sampled per episode from a YAML schedule
- **Headless-friendly** — designed for the MuJoCo backend on a GPU box

## A minimal collection run

```bash
ros2 launch quad_utils quad_mujoco.py
ros2 launch quad_training collect.py \
  episodes:=1000 \
  curriculum:=stand_walk_basic \
  output_dir:=$HOME/datasets/quad_walk_v1
```

Output is a directory of `.parquet` shards plus a `manifest.json` that maps episode ID → terrain seed → outcome. Loaders for PyTorch and JAX live under `quad_training/loaders/`.

## What gets logged per step

| Channel | Shape | Notes |
|---|---|---|
| `obs.body_state` | (13,) | pos (3) + quat (4) + lin_vel (3) + ang_vel (3) |
| `obs.joint_state` | (24,) | pos (12) + vel (12) |
| `obs.contacts` | (4,) | bool per leg |
| `obs.cmd_vel` | (3,) | linear x/y, angular z |
| `action` | (12,) | joint commands as logged by the controller |
| `reward` | (1,) | from `reward_fn` in the curriculum YAML |
| `done` | (1,) | episode-terminal flag |

## Curricula

Curriculum YAMLs live in `quad_training/config/curricula/`. Each is a list of stages with sampling rules. Example:

```yaml
- name: stand
  duration_eps: 200
  init_pose:
    sampler: nominal
  cmd_vel:
    sampler: zero
  terrain: flat
  reward: stand_upright

- name: walk_flat
  duration_eps: 500
  init_pose:
    sampler: nominal_jitter
    jitter: 0.05
  cmd_vel:
    sampler: uniform
    range: { vx: [-0.5, 1.0], vy: [-0.3, 0.3], wz: [-0.5, 0.5] }
  terrain: flat
  reward: track_cmd_vel
```

## Reproducibility

Each episode logs:

- Curriculum stage + step within stage
- RNG seed (used for terrain, init pose, perturbations)
- Robot URDF SHA + per-robot YAML SHA
- Quad-SDK git SHA

Together these let you re-run a problematic episode bit-for-bit on the MuJoCo backend.

## Running on a cluster

The `quad_training` rig spins up one MuJoCo instance per worker; on a 16-core box you can pack ~12 collectors and saturate I/O before CPU. There's a `--launch-mode=detached` flag that supervises children and writes shards as they complete (so a crash loses at most one episode).

## Troubleshooting

??? warning "Episodes look correct, model trains poorly"
    Check the `reward` curve in the manifest. If it's noisier than expected, your reset condition may be triggering early — inspect `done` flag distribution per stage.

??? warning "Workers desync after a few hours"
    Long collection runs occasionally drift on heavily-loaded shared boxes. Pin workers with `taskset` or move to a dedicated machine.

??? warning "Loader complains about schema mismatch"
    Curriculum changes that add/remove channels invalidate older shards. The manifest carries a schema hash — gate loaders on it.
