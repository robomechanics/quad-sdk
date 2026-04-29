---
title: Creating Custom Terrain Maps
tags:
  - tutorial
  - simulation
  - terrain
---

# Creating Custom Terrain Maps

Generate a Gazebo world + GridMap-compatible 2.5D terrain layer from a CAD part. Useful for benchmarking the planner on repeatable obstacles.

## Overview

The pipeline is:

1. **CAD** — design the part in your CAD package
2. **Export** — `.stl` (mesh) + `.ply` (point cloud) at metric scale
3. **Wrap** — drop a `model.config` and `<world>.world` next to the meshes
4. **Install** — copy into `quad_simulator/gazebo_scripts/worlds`
5. **Verify** — spawn and confirm the robot lands on the surface

## Step 1 — CAD

Model the part in Solidworks (or Fusion / FreeCAD / etc.). Origin convention: place the part origin where you want the world origin. Quad-SDK's planners assume +Z is up.

## Step 2 — Export

Save **two** files with these settings:

- **`.stl` (binary)** — units **meters**, custom resolution, **maximum facet size 0.20 m**
- **`.ply` (binary)** — same units and resolution

Why both: Gazebo physics consumes the STL (heightfield collision), the GridMap layer consumes the PLY (perception terrain).

## Step 3 — Wrap

Create a folder named after the part:

```
<part_name>/
├── <part_name>.stl
├── <part_name>.ply
├── <part_name>.world
└── model.config
```

Use `flat.world` as a template for the `.world` file. Two lines must change:

| Line in `flat.world` | Update |
|---|---|
| ~26 | mesh path → `model://<part_name>/<part_name>.stl` |
| ~31 | model name |

`model.config` follows the standard Gazebo schema — see the [Gazebo model docs](https://classic.gazebosim.org/tutorials?tut=model_structure&cat=build_robot).

If you also keep the original `.sldprt` (or `.f3d`, etc.) in the folder, reproducibility is much easier when someone wants to re-mesh.

## Step 4 — Install

Copy the folder into:

```
quad-sdk/quad_simulator/gazebo_scripts/worlds/<part_name>/
```

## Step 5 — Verify

Spawn with `gui:=true` and confirm the robot lands on the surface, not through it:

```bash
ros2 launch quad_utils quad_gazebo.py \
  world:=<part_name>.world gui:=true \
  robot_configs:='[{"name":"robot_1","type":"go2","controller":"inverse_dynamics","init_pose":"-x 0 -y 0 -z 0.6"}]'
```

If the robot starts inside the geometry, raise the `init_pose -z` value. If it falls through, the STL units are probably wrong (export was in mm).

## Tips

!!! tip "Facet size matters"
    Going below 0.20 m blows up collision-mesh memory and slows physics. Going above produces visible terrain steps. 0.20 m is a good default for typical lab obstacles.

!!! tip "Heightfield vs. mesh"
    For purely 2.5D terrain (no overhangs), a heightfield is faster than a mesh. The PLY → heightfield path is more involved but worth it for large maps.

!!! warning "Friction is set in the world file"
    Don't expect the planner's `mu` to match if the world file's friction differs. Keep them in sync, especially when comparing sim vs. real.
