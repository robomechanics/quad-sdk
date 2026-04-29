---
title: Installation
tags:
  - getting-started
  - install
---

# Installation

Quad-SDK is developed and tested on **Ubuntu 24.04** with **ROS 2 Jazzy**. Three Docker containers are provided for the most common workflows — see [Docker install](#docker-install) below.

## Prerequisites

- Ubuntu 24.04 LTS (real or VM/container)
- ROS 2 Jazzy ([install instructions](https://docs.ros.org/en/jazzy/Installation.html))
- ~10 GB free disk space for the workspace, dependencies, and simulation assets
- For hardware: a supported quadruped (Spirit, Go1, Go2, Go2-W, A1, B2, Spot, Vision60) and the manufacturer SDK installed and tested independently
- **HSL solver for IPOPT** — the NMPC controller requires the HSL routines (free for academic use). Request access at <https://licences.stfc.ac.uk/product/coin-hsl>, then drop the source tarball next to the IPOPT submodule (see [HSL setup](#hsl-solver-setup) below).

!!! tip "Don't have hardware?"
    Quad-SDK runs end-to-end in **Gazebo Harmonic** or **MuJoCo** with no hardware attached. Use the [Quickstart](quickstart.md) once installed.

## Docker install

Three containers are provided, each tuned for a different stage of the workflow:

| Container | Use it for | Where it runs |
|---|---|---|
| **`x86`** | Local development — simulation, learned-policy training, NMPC iteration. Includes CUDA 12.8, ONNX Runtime, GPU passthrough. | Your workstation/laptop (x86_64) |
| **`arm-mpc`** | Hardware deployment of the **MPC** stack on the robot's onboard computer. ROS 2 only — no ONNX, smaller image. | Robot (Jetson / arm64) |
| **`arm-learned`** | Hardware deployment of **learned policies** on the robot. ARM build of ONNX Runtime + CUDA + TensorRT. | Robot Jetson with JetPack 6.x + NVIDIA Container Runtime |

Pick **`x86`** for everything you do at your desk. Use **`arm-mpc`** or **`arm-learned`** when you SSH into the robot and run on real hardware.

### HSL solver setup

The IPOPT solver inside `nmpc_controller` needs CoinHSL. After cloning the repo:

```bash
# Download CoinHSL from https://licences.stfc.ac.uk/product/coin-hsl
# (academic use is free but requires registration)

cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/robomechanics/quad-sdk.git
cd quad-sdk

# Drop the unpacked CoinHSL source into the IPOPT submodule:
cp -r /path/to/coinhsl ./external/ipopt/coinhsl
```

Without this step, `colcon build --packages-select nmpc_controller` will fail at link time.

### VS Code (devcontainer)

```bash
git clone --recurse-submodules https://github.com/robomechanics/quad-sdk.git
cd quad-sdk
git checkout devel_ros2

# Add the HSL source as above
cp -r /path/to/coinhsl ./external/ipopt/coinhsl

# Allow X11 forwarding for Gazebo / RViz
xhost +local:docker

code .
# Command Palette → "Dev Containers: Reopen in Container"
# Pick the container that matches your workflow (x86 / arm-mpc / arm-learned)
```

Inside the container:

```bash
sudo apt-get update && sudo apt-get upgrade -y
chmod +x setup.sh && ./setup.sh
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### CLI

```bash
cd ~/ros2_ws/src/quad-sdk/.devcontainer

# Build the image you need
docker compose build x86          # or: arm-mpc / arm-learned
# Equivalent shortcut: docker build -t quad-sdk:latest .

# Run the container with X11 + GPU passthrough
xhost +local:docker
./run.sh quad-sdk quad-sdk:latest
```

Persistent containers can be re-entered without rebuilding:

```bash
docker start -i quad-sdk
```

## Standard install (no Docker)

If you prefer a host install:

```bash
# 1. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 2. Clone with submodules
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/robomechanics/quad-sdk.git

# 3. Add CoinHSL — see "HSL solver setup" above
cp -r /path/to/coinhsl quad-sdk/external/ipopt/coinhsl

# 4. Install dependencies (idempotent)
cd quad-sdk && ./setup.sh

# 5. Build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

The `setup.sh` script installs:

- `doxygen`, `libeigen3-dev`, `python3-pip`, `colcon`, `rosdep`, `python3-colcon-clean`
- `ament_cmake` and other ROS-side build tooling
- per-package dependencies declared in each `setup_deps.sh`
- everything in `package.xml` via `rosdep`

!!! note "Submodules matter"
    Always clone with `--recurse-submodules`. Several third-party libs (e.g. IPOPT) live as submodules under `external/`. If you forgot, run `git submodule update --init --recursive` from the repo root.

## Building on robot hardware (Unitree)

NatNet — a dependency of the mocap driver — is **x86-only**. When building on a Jetson or other arm64 robot, skip the optitrack driver:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip mocap4r2_optitrack_driver
```

On the remote workstation (x86_64) you can build everything normally:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Verify

After `source install/setup.bash`:

```bash
ros2 pkg list | grep quad_
```

You should see `quad_msgs`, `quad_utils`, `global_body_planner`, `local_planner`, `nmpc_controller`, `robot_driver`, `quad_simulator`, and friends.

## Troubleshooting

??? warning "`rosdep` reports unresolved keys"
    Run `rosdep update` and re-run `setup.sh`. If a key is genuinely unavailable for Jazzy, file an [issue](https://github.com/robomechanics/quad-sdk/issues).

??? warning "IPOPT / CoinHSL link errors"
    Verify `external/ipopt/coinhsl/` exists and is non-empty. Re-run `colcon build --packages-up-to nmpc_controller`.

??? warning "Build fails on `mocap4r2_optitrack_driver` (arm64)"
    NatNet is x86-only. Add `--packages-skip mocap4r2_optitrack_driver` to your `colcon build`.

??? warning "Gazebo Harmonic plugin not found"
    Source `install/setup.bash` *after* the build completes — the simulator plugins live in `quad_simulator` and are exposed via the install space.

??? warning "Graphics issues in WSL or remote sessions"
    Gazebo Harmonic's `gz sim` requires a working OpenGL context. For headless or remote use, prefer **MuJoCo** (`ros2 launch quad_utils quad_mujoco.py`) which renders software-fallback friendly.

[:octicons-arrow-right-24: Continue to Quickstart](quickstart.md)
