---
title: Adding a New Robot
tags:
  - tutorial
  - hardware
  - robot
---

# Adding a New Robot

End-to-end recipe for getting a new quadruped running in Quad-SDK on ROS 2. Allow a few days the first time — the NMPC code-generation step is the largest chunk.

## Prerequisites

- A URDF for your robot (with valid inertial parameters — verify in `urdf-viz` first)
- Manufacturer SDK installed and tested independently
- MATLAB with the Symbolic Math Toolbox, plus CasADi 3.5.5 (for the NMPC code-gen step)

## Step 1 — Robot description

Add a new ROS 2 package under `quad_simulator/`:

```
quad_simulator/<robot>_description/
├── models/
│   └── <robot>/
│       ├── dae/
│       ├── meshes/
│       ├── urdf/
│       │   └── <robot>.urdf.xacro
│       ├── xacro/
│       ├── <robot>.sdf.xacro
│       └── model.config
├── CMakeLists.txt
├── LICENSE
└── package.xml
```

Generate the SDF file:

```bash
ros2 run xacro xacro models/<robot>/<robot>.sdf.xacro > <robot>.sdf
```

Append the model path to `quad_simulator/setup_deps.sh`:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(...)/<robot>_description/models
```

## Step 2 — Per-robot YAML

Copy `quad_utils/config/go2.yaml` to `quad_utils/config/<robot>.yaml`. Edit:

- **Body:** `h_nom`, `desired_height`, `h_min`, `h_max`
- **Gait:** `period`, `ground_clearance`, `duty_cycles`, `phase_offsets`
- **NMPC:** `nmpc_controller.body.u_lb/u_ub`, `nmpc_controller.feet.u_lb/u_ub`, `nmpc_controller.joints.x_lb/x_ub` (must match URDF)
- **Gains:** `stand_kp/kd`, `stance_kp/kd`, `swing_kp/kd`
- **Pose:** `stand_joint_angles`, `sit_joint_angles`
- **Limits:** `motor_limits.torque`, `motor_limits.speed`
- **Joint mapping:** the per-leg `name` / `sign` / `offset` table — *the most error-prone step, see the warning below*

!!! warning "Sign and offset mistakes"
    A wrong joint sign manifests as a robot that drifts on stand or refuses to track. Verify the mapping by commanding each joint individually with the manufacturer SDK first, then translate that polarity into the YAML.

## Step 3 — Wire launch files

Edit:

- `quad_utils/launch/robot_bringup.py` and `quad_utils/launch/planning.py`: add a case for `<robot>` setting:
    - `desc_pkg = "<robot>_description"`
    - `urdf_file = "<robot>.urdf.xacro"`
    - `sdf_file = "<robot>.sdf.xacro"`
    - `config_file = "<robot>.yaml"`
- `quad_utils/launch/robot_driver.py` and `quad_utils/launch/remote_driver.py`: add `<robot>` to the description-package map

Smoke test:

```bash
ros2 launch quad_utils quad_gazebo.py \
  robot_configs:='[{"name":"robot_1","type":"<robot>","controller":"inverse_dynamics","init_pose":"-x 0 -y 0 -z 0.5"}]'
```

The robot should spawn and report joint state. It won't stand yet — we still need the NMPC files.

## Step 4 — Generate NMPC code

The local planner uses a robot-specific NMPC compiled from CasADi expressions. Generate them once per robot:

1. From `quad-sdk/nmpc_controller/scripts/`, edit `main.m`:
   - `parameter.name = '<robot>'`
   - `parameter.physics.sim2real_scale_factor`
   - `parameter.physics.mass_body_body`, `mass_body_leg`
   - `parameter.physics.hip_offset`
   - `parameter.physics.inertia_body`
2. Run the script. It produces:
   - `eval_g_<robot>.cpp`, `eval_hess_g_<robot>.cpp`, `eval_jack_g_<robot>.cpp`
   - Matching `.h` files

Copy outputs:

- `.cpp` → `nmpc_controller/src/gen/`
- `.h` → `nmpc_controller/include/nmpc_controller/gen/`

## Step 5 — Wire the NMPC

Edit `nmpc_controller/include/nmpc_controller/quad_nlp.h`:

```cpp
#include "nmpc_controller/gen/eval_g_<robot>.h"
#include "nmpc_controller/gen/eval_hess_g_<robot>.h"
#include "nmpc_controller/gen/eval_jack_g_<robot>.h"

enum SystemID {
    SPIRIT = 0,
    GO2,
    // ...
    <ROBOT>,
};
static const int num_sys_id_ = /* total entries */;
```

Edit `nmpc_controller/src/nmpc_controller.cpp`:

```cpp
switch (robot_id_) {
    // ...
    case <ROBOT>:
        // load the per-robot CasADi-emitted callbacks
        break;
}
```

And `nmpc_controller/src/quad_nlp_utils.cpp`: dispatch to the new leg-controller functions.

## Step 6 — Update the local planner

In `local_planner/src/local_planner.cpp`, add a case mapping `robot_name_ == "<robot>"` to the new `SystemID`.

## Step 7 — Hardware interface (skip if sim only)

Create:

- `robot_driver/include/robot_driver/hardware_interfaces/<robot>_interface.h`
- `robot_driver/src/hardware_interfaces/<robot>_interface.cpp`

Implement the manufacturer-SDK <-> ROS 2 glue. Use `go2_interface.cpp` as the template.

In `robot_driver/src/robot_driver.cpp` constructor, add a case:

```cpp
} else if (robot_type_ == "<robot>") {
    hardware_interface_ = std::make_shared<<Robot>Interface>();
}
```

Update `robot_driver/CMakeLists.txt` to compile the new file.

## Step 8 — Stand and tune

Build, source, and run the standing test:

```bash
colcon build --packages-up-to robot_driver local_planner nmpc_controller
source install/setup.bash
ros2 launch quad_utils quad_gazebo.py
ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 1" --once
```

If the robot stands cleanly, walk:

```bash
ros2 launch quad_utils quad_plan.py
ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 2" --once
```

Iterate on `<robot>.yaml` from there. Most tuning hours go into:

1. Stand gains (oscillation, sag)
2. NMPC GRF bounds (infeasible references)
3. Step timing (gait period, duty cycle)

[:octicons-arrow-right-24: Writing your own controller](writing-controller.md)
