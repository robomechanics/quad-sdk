---
title: Specialized Controllers
tags:
  - tutorial
  - control
  - learned-policies
---

# Specialized Controllers

Quad-SDK ships hooks for two specialized controller families: **learned policies** (ONNX) and the **leg-disentanglement (Underbrush)** controller for cluttered environments.

## Learned controllers (ONNX)

### Overview

The runtime can execute neural-network policies trained in **IsaacLab** or **MuJoCo** via the **ONNX Runtime**. Custom policy parameterizations can be created by subclassing `LearnedController` (which itself implements the `LegController` interface — see [Writing your own controller](writing-controller.md)).

### Prerequisites

- C++ ONNXRuntime installed locally, **or** the Quad-SDK devcontainer (which bundles it)
- A `.onnx` model file with the expected input/output schema (see `LearnedController` header for the contract)

### Switching to a learned controller

1. In `quad_gazebo.py` and `quad_plan.py`, set `controller_mode` to `learned` in the robot config.
2. Copy your `.onnx` weights into `robot_driver/include/robot_driver/models/`.
3. Update the model path in `robot_driver/config/robot_driver.yaml`.

### Run

```bash
ros2 launch quad_utils quad_gazebo.py
ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 1" --once

ros2 launch quad_utils quad_plan.py \
  robot_configs:='[{"name":"robot_1","type":"go2","controller_mode":"learned","reference":"twist","twist_input":"keyboard"}]'
```

### Tips

!!! tip "Match the training observation order"
    The most common bug is observation-vector ordering disagreeing between training and deployment. Bake the schema into a header (or load it from JSON next to the `.onnx`) rather than relying on convention.

!!! warning "Don't allocate per tick"
    Pre-allocate the input/output tensors in the constructor; reuse them every call.

## Leg-disentanglement (Underbrush)

### Overview

A reactive swing-phase controller that prevents leg entanglements in cluttered natural and human-made environments using proprioceptive sensing only — no extra cameras or contact sensors required.

In benchmark trials reported in the publication, the controller succeeded in 14 of 16 lab trials.

### Status

- **Available on**: ROS 1 (`devel` branch only)
- **Not yet ported** to ROS 2

### Citation

> Yim, J. K., Ren, J., Ologan, D., Gonzalez, S. G., & Johnson, A. M. *Proprioception and reaction for walking among entanglements*. IEEE/RSJ IROS, 2023.

### Usage (ROS 1)

```bash
git checkout devel
roslaunch quad_utils underbrush_gazebo.launch
rostopic pub /robot_1/control/mode std_msgs/UInt8 "data:1"
roslaunch quad_utils quad_plan.launch reference:=twist logging:=true
rosrun body_force_estimator path_following.py
```

For a ROS 2 port, the swing-phase reactive logic in `body_force_estimator/src/path_following.py` is the most useful starting point.
