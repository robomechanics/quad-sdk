---
title: ROS 2 Migration
tags:
  - architecture
  - ros2
  - migration
---

# ROS 2 Migration

Quad-SDK now targets **ROS 2 Jazzy on Ubuntu 24.04**. This page captures what changed, why, and what to update if you're porting downstream code from a ROS 1 fork.

!!! info "Looking for ROS 1 docs?"
    Use the version selector in the top-right of this site to switch to the **`ros1`** docs. The ROS 1 stack is no longer actively developed — we recommend new work targets ROS 2.

## What changed at a glance

| Area | ROS 1 | ROS 2 (current) |
|---|---|---|
| Build system | `catkin` / `catkin_make` | `colcon` + `ament_cmake` / `ament_python` |
| Launch | XML `*.launch` | Python `*.py` (Launch description API) |
| Node API | `roscpp` / `rospy` | `rclcpp` / `rclpy` |
| Time | `ros::Time::now()` | `node->now()` (clock-aware) |
| Params | `<param>` in launch | `ros__parameters` YAML, namespace wildcards |
| Kinematics/dynamics | RBDL (`QuadKD`) | Pinocchio (`QuadKD2`) |
| Simulation | Gazebo Classic | **Gazebo Harmonic** (`gz sim`) + **MuJoCo** |
| ROS bridge for Gazebo | `gazebo_ros` | `ros_gz_bridge` |
| Robot description | URDF + xacro | URDF + xacro + SDF (Gazebo) |
| Bag format | `.bag` | rosbag2 (mcap by default) |

## What you need to update if porting

### Launch files

ROS 1:
```xml
<launch>
  <include file="$(find quad_utils)/launch/planning.launch"/>
  <node pkg="local_planner" type="local_planner_node" name="local_planner"/>
</launch>
```

ROS 2 equivalent:
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            FindPackageShare("quad_utils").find("planning.py"),
        ),
        Node(package="local_planner", executable="local_planner_node"),
    ])
```

See `quad_utils/launch/` for canonical examples — favor `OpaqueFunction` and `FindPackageShare`.

### Parameter loading

Drop `<param>` and `<rosparam>`. Put per-robot params in `quad_utils/config/<robot>.yaml`:

```yaml
/**:
  ros__parameters:
    local_planner:
      update_rate: 333.0
      timestep: 0.03
```

Namespace-wildcard nodes (`/**:`) make params apply across the multi-robot tree.

### Topic API

ROS 1 → ROS 2 cheat sheet for the bits that bite:

```cpp
// Publisher
auto pub = node->create_publisher<quad_msgs::msg::RobotPlan>("global_plan", 10);

// Subscription
auto sub = node->create_subscription<quad_msgs::msg::RobotState>(
    "state/ground_truth", 10,
    [this](const quad_msgs::msg::RobotState::SharedPtr msg) { /* ... */ });

// Timer
auto t = node->create_wall_timer(33ms, [this]() { update(); });
```

### Time

`ros::Time::now()` → `node->now()`. Always pass the node's clock (or use `rclcpp::Clock`) — never wallclock — so sim-time and bag playback work.

### Kinematics

If your code calls `QuadKD::*FK*()`, the same call is generally available on `QuadKD2`, but **build a `Pinocchio::Model` from the URDF once** and call `updateFromPinocchio()` per control loop before any FK/Jacobian evaluation. Joint ordering follows URDF order; use the supplied mappers when crossing the API boundary. See [Pinocchio integration](pinocchio-integration.md).

### Bags

`.bag` → rosbag2 (mcap). Replay:

```bash
ros2 bag play <bag_directory>
```

For MATLAB post-processing, regenerate Quad-SDK message bindings once:
```matlab
ros2genmsg('~/ros2_ws/src/quad-sdk')
rehash toolboxcache; clear
```

### Tests

`catkin_add_gtest` → `ament_add_gtest`. See `quad_utils/test/` for templates.

## What didn't change

- Topic names (`global_plan`, `local_plan`, `state/ground_truth`, etc.)
- Message structure (`quad_msgs::*` are unchanged in shape, only the namespace path is `quad_msgs::msg::*` in ROS 2)
- High-level architecture, the planning/control split, and the runtime loop rates
- The per-package config scheme — same YAML keys, just under `ros__parameters`

## Removed in the ROS 2 build

- ROS 1 launch XML files
- RBDL build path (the source files remain in-tree but are not linked)
- Gazebo Classic worlds (replaced with Gazebo Harmonic SDF + MuJoCo MJCF)

The legacy `underbrush` controller is still available in the ROS 2 build, but it currently only runs on the **Ghost Robotics Spirit 40** — see [Specialized Controllers](../tutorials/specialized-controllers.md) for usage and porting notes.

[:octicons-arrow-right-24: Pinocchio integration](pinocchio-integration.md)
