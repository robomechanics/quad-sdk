# Quad-SDK MuJoCo Simulation

[![Rdev](https://build.ros2.org/job/Rdev__mujoco_ros2_control__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mujoco_ros2_control__ubuntu_noble_amd64/) [![Kdev](https://build.ros2.org/job/Kdev__mujoco_ros2_control__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mujoco_ros2_control__ubuntu_noble_amd64/) [![Jdev](https://build.ros2.org/job/Jdev__mujoco_ros2_control__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mujoco_ros2_control__ubuntu_noble_amd64/) [![Hdev](https://build.ros2.org/job/Hdev__mujoco_ros2_control__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mujoco_ros2_control__ubuntu_jammy_amd64/) [![CI](https://github.com/ros-controls/mujoco_ros2_control/actions/workflows/ci.yaml/badge.svg)](https://github.com/ros-controls/mujoco_ros2_control/actions/workflows/ci.yaml) ![License](https://img.shields.io/github/license/ros-controls/mujoco_ros2_control) [![Codecov](https://codecov.io/gh/ros-controls/mujoco_ros2_control/branch/main/graph/badge.svg)](https://codecov.io/gh/ros-controls/mujoco_ros2_control)

This repository provides a ros2_control system interface and supporting packages to run the Unitree Go2 quadrupedal robot against the MuJoCo physics simulator.

This project wraps MuJoCo as a hardware/system interface so you can use the ros2_control stack (controller manager, controllers, controller interfaces) against simulated robots based on MJCF or generated from URDF.

### Contents

- `mujoco_ros2_control` - core system interface plugin and resources
- `mujoco_vendor` - Provides the base MuJoCo installation 
- `quad_utils` - core launch logic and simulation scripts
- `go2_description` - URDF and meshes for the Unitree Go2
- `mujoco_scripts` - MJCF/XML world files and robot definitions

### Key features

- Full ros2_control SystemInterface plugin for MuJoCo
- URDF to MuJoCo actuator mapping for Unitree Go2 hardware
- Support for quadrupedal locomotion within the ROS 2 ecosystem

## Quick start

The simulation is designed to run within a Conda environment. Ensure your environment is properly configured before launching.

- Environment Setup

  1. Activate the designated Conda environment:
  ```bash
  conda activate mujoco
  
  2. Source the ROS 2 installation and your workspace:
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
  
  3. Launch the MuJoCo simulator with the robot driver:
  ros2 launch quad_utils quad_mujoco.py
