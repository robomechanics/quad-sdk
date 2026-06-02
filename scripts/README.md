# Scripts

Utility scripts for Quad-SDK development. Two unrelated workflows live here:

- **Blender Bag Visualization** — generate animated scenes and MP4 videos from recorded ROS2 bag files.
- **Linting** — run `cpplint` over the C++ source tree (`lint_soft.sh`, `lint_hard.sh`).

## Blender Bag Visualization

Generate animated Blender scenes and MP4 videos from recorded ROS2 bag files.

## Prerequisites

- ROS2 Jazzy workspace built with `quad_msgs`
- Blender 4.0+ (installed at `/usr/bin/blender`)
- A recorded bag file containing `quad_msgs/msg/RobotState` on a ground truth or state estimate topic

## Quick Start

From the workspace root (`ros2_ws/`):

```bash
# 1. Source the workspace
source install/setup.bash

# 2. Extract bag data to JSON
python3 src/quad-sdk/scripts/extract_bag.py <bag_directory> -o recording.json

# 3. Generate .blend file and render MP4
blender --background --python src/quad-sdk/scripts/bag_to_blender.py -- \
    --json recording.json \
    --urdf_meshes src/quad-sdk/quad_simulator/go2_description/models/go2/meshes \
    --output recording.blend \
    --render output_frames/
```

## Step-by-Step

### 1. Extract bag data

`extract_bag.py` reads a ROS2 bag (MCAP format) and writes robot body pose + joint angles to a JSON file.

```bash
source install/setup.bash
python3 src/quad-sdk/scripts/extract_bag.py training_bags/robot_1_training_go2_20260406_180302_raw
```

Options:

| Flag | Default | Description |
|------|---------|-------------|
| `-o`, `--output` | `<bag_name>.json` | Output JSON path |
| `-t`, `--topic` | `/robot_1/state/ground_truth` | RobotState topic to extract |

Example with a different topic:

```bash
python3 src/quad-sdk/scripts/extract_bag.py my_bag -t /robot_1/state/estimate -o estimate.json
```

### 2. Generate Blender animation

`bag_to_blender.py` runs inside Blender's Python environment. It imports the Go2 DAE meshes, builds the kinematic chain, and keyframes every frame from the JSON.

```bash
blender --background --python src/quad-sdk/scripts/bag_to_blender.py -- \
    --json recording.json \
    --urdf_meshes src/quad-sdk/quad_simulator/go2_description/models/go2/meshes \
    --output my_scene.blend
```

Options:

| Flag | Default | Description |
|------|---------|-------------|
| `--json` | *(required)* | Path to JSON from extract_bag.py |
| `--urdf_meshes` | *(required)* | Path to Go2 meshes directory (contains trunk.dae, hip.dae, etc.) |
| `--terrain` | none | Optional terrain STL file |
| `--output` | `go2_sim.blend` | Output .blend file path |
| `--render` | none | If set, render PNG frames to this directory |

### 3. Export MP4 video

There are two ways to get an MP4:

**Option A: Render directly from the command line**

```bash
blender --background my_scene.blend --python-expr "
import bpy

scene = bpy.context.scene
scene.render.resolution_x = 1920
scene.render.resolution_y = 1080
scene.render.fps = 60
scene.frame_start = 1
scene.frame_end = scene.frame_end  # full recording, or set a specific end frame
scene.frame_step = 8               # skip frames (bag is ~500 Hz, step=8 gives ~60 fps playback)

scene.render.image_settings.file_format = 'FFMPEG'
scene.render.ffmpeg.format = 'MPEG4'
scene.render.ffmpeg.codec = 'H264'
scene.render.ffmpeg.constant_rate_factor = 'MEDIUM'
scene.render.filepath = '/path/to/output.mp4'

bpy.ops.render.render(animation=True)
"
```

**Option B: Open in Blender GUI and render interactively**

```bash
blender my_scene.blend
```

1. Open **Output Properties** (printer icon in the right panel)
2. Set format to **FFmpeg Video**, container **MPEG-4**, codec **H.264**
3. Set output path
4. Adjust frame range and resolution as needed
5. **Render > Render Animation** (or Ctrl+F12)

### 4. Add terrain

Pass a terrain STL from the simulator models:

```bash
blender --background --python src/quad-sdk/scripts/bag_to_blender.py -- \
    --json recording.json \
    --urdf_meshes src/quad-sdk/quad_simulator/go2_description/models/go2/meshes \
    --terrain src/quad-sdk/quad_simulator/quad_sim_scripts/models/step_20cm/meshes/step_20cm.stl \
    --output with_terrain.blend
```

Available terrains in `quad_simulator/quad_sim_scripts/models/`:

```
flat/           slope_20/       step_10cm/      step_20cm/
step_25cm/      step_30cm/      gap_20cm/       gap_40cm/
gap_80cm/       rough_25cm/     rough_40cm_huge/  parkour_local_min/
```

## Full Example

```bash
# Source workspace
source install/setup.bash

# Extract 
python3 src/quad-sdk/scripts/extract_bag.py \
    training_bags/robot_1_training_go2_20260406_180302_raw \
    -o go2_walk.json

# Build scene with flat terrain
blender --background --python src/quad-sdk/scripts/bag_to_blender.py -- \
    --json go2_walk.json \
    --urdf_meshes src/quad-sdk/quad_simulator/go2_description/models/go2/meshes \
    --terrain src/quad-sdk/quad_simulator/quad_sim_scripts/models/flat/meshes/flat.stl \
    --output go2_walk.blend

# Render MP4 (first 5 seconds at 60fps, bag is ~500Hz so step=8)
blender --background go2_walk.blend --python-expr "
import bpy
s = bpy.context.scene
s.render.resolution_x = 1920
s.render.resolution_y = 1080
s.render.fps = 60
s.frame_start = 1
s.frame_end = 2500
s.frame_step = 8
s.render.image_settings.file_format = 'FFMPEG'
s.render.ffmpeg.format = 'MPEG4'
s.render.ffmpeg.codec = 'H264'
s.render.ffmpeg.constant_rate_factor = 'MEDIUM'
s.render.filepath = 'go2_walk.mp4'
bpy.ops.render.render(animation=True)
"
```

## Notes

- Bag files at ~500 Hz produce thousands of frames. Use `frame_step` to downsample for rendering (step=8 maps ~500 Hz to ~60 fps playback).
- The camera automatically tracks the robot's XY position during playback.
- To adjust the camera angle, open the .blend file in Blender and reposition the `Camera` object or its `CameraTracker` parent.
- The script currently supports the Go2 robot. For other robots, update the kinematic constants and mesh paths in `bag_to_blender.py`.

## Linting

`lint_soft.sh` and `lint_hard.sh` run `cpplint` over the C++ source tree to catch style and basic-quality issues before pushing. Both require `cpplint` on `$PATH` (`pip install --user cpplint`).

| Script | Behavior |
|---|---|
| `lint_soft.sh` | Suppresses copyright/runtime/build/casting/function-size categories; useful as a quick pre-commit gate. Excludes `external/`, generated NMPC headers, and `matlab_msg_gen/`. |
| `lint_hard.sh` | Runs `cpplint` with the default category set from the parent directory — closer to the CI lint. Excludes only `external/` and the generated NMPC headers. |

Run from the repo root:

```bash
./scripts/lint_soft.sh    # fast, permissive — daily-use gate
./scripts/lint_hard.sh    # full categories — CI-parity check
```

Either script exits non-zero on the first violation (`set -e`), so wire them into pre-push hooks if desired.
