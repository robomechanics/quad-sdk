---
title: Adding a New Robot
tags:
  - tutorial
  - hardware
  - robot
  - ros2
---

# Adding a New Robot

End-to-end recipe for getting a new quadruped running in Quad-SDK on ROS 2. Allow
a few days the first time — the NMPC code-generation step is the largest chunk.

On the `ros2_control` stack the work splits cleanly: a **description package**
(URDF + SDF, which select the hardware backend on a `use_sim` arg), a
**robot-specific YAML** (kinematics, gains, poses, planner params — *only* the
things that vary per robot), a **launch registration**, and a **hardware
interface**. The controller TYPES and the supervisor come from the shared
`quad_controllers/config/controllers.yaml`, so you never duplicate them. See the
[Control Stack architecture](../architecture/control-stack.md) for the big
picture.

## Prerequisites

- A URDF for your robot (with valid inertial parameters — verify in `urdf-viz` first)
- Manufacturer SDK installed and tested independently
- MATLAB with the Symbolic Math Toolbox, plus CasADi 3.5.5 (for the NMPC code-gen step)

## Step 1 — Robot description package

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

### The URDF xacro: switch the backend on `use_sim`

The URDF's `<ros2_control>` block selects the hardware-interface backend from a
`use_sim` xacro arg — Gazebo in sim, your `quad_hardware` `SystemInterface` on
hardware. Mirror `go2.urdf.xacro`:

```xml
<xacro:arg name="use_sim" default="true" />
<xacro:arg name="read_only" default="true" />      <!-- hardware: true commands no torque -->
<xacro:arg name="network_interface" default="eth0" />

<!-- Simulation: Gazebo, effort-only -->
<xacro:if value="$(arg use_sim)">
  <ros2_control name="GazeboSimSystem" type="system">
    <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>
    <joint name="0">
      <command_interface name="effort" />
      <state_interface name="velocity" />
      <state_interface name="position" />
    </joint>
    <!-- ... joints 1..11 ... -->
  </ros2_control>
</xacro:if>

<!-- Hardware: five command interfaces per joint (onboard motor PD) + IMU sensor -->
<xacro:unless value="$(arg use_sim)">
  <ros2_control name="UnitreeSystem" type="system">
    <hardware>
      <plugin>quad_hardware/UnitreeSystem</plugin>
      <param name="network_interface">$(arg network_interface)</param>
      <param name="read_only">$(arg read_only)</param>
    </hardware>
    <joint name="0">
      <command_interface name="position" /> <command_interface name="velocity" />
      <command_interface name="kp" /> <command_interface name="kd" />
      <command_interface name="effort" />
      <state_interface name="position" /> <state_interface name="velocity" />
      <state_interface name="effort" />
    </joint>
    <!-- ... joints 1..11 + <sensor name="imu"> ... -->
  </ros2_control>
</xacro:unless>
```

### The SDF xacro: load the merged config into the Gazebo plugin

The SDF embeds the `gz_ros2_control` plugin, which loads the **merged** param
file the launch produces (controllers + robot + sim), and the ground-truth
estimator plugin that publishes `state/ground_truth` in sim:

```xml
<xacro:arg name="controller_config_path" default="/path/<robot>.yaml" />

<plugin filename="libgz_ros2_control-system.so"
        name="gz_ros2_control::GazeboSimROS2ControlPlugin">
  <parameters>$(arg controller_config_path)</parameters>
</plugin>
<plugin filename="libground_truth_estimator.so" name="ground_truth_estimator">
  <parameters>$(arg controller_config_path)</parameters>
</plugin>
```

Append the model path to `quad_simulator/setup_deps.sh`:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(...)/<robot>_description/models
```

## Step 2 — Per-robot YAML (robot-specific params only)

Copy `quad_utils/config/go2.yaml` to `quad_utils/config/<robot>.yaml`. This file
holds **only** what varies per robot — it is merged with the shared
`quad_controllers/config/controllers.yaml` (controller types + supervisor) and a
context overlay (`sim.yaml` / `hardware.yaml`) at launch, so **do not** duplicate
the controller type map or the `mode_supervisor` block here.

Edit:

- **Kinematics** — the `leg_0`..`leg_3` blocks: per-joint `name` / `sign` / `offset` and the `frames` (hip/upper/lower/toe). The `body.frame`. *The most error-prone step — see the warning.*
- **Limits** — `motor_limits.torque` (also used as the effort-mode torque clamp), `motor_limits.speed`.
- **Per-controller gains/poses** under the `/**/locomotion_controller`, `/**/sit_controller`, `/**/safety_controller`, `/**/sit_to_ready_controller`, `/**/ready_to_sit_controller` keys: `stance_kp/kd`, `swing_kp/kd`, `stand_kp/kd`, `stand_joint_angles`, `sit`/transition angles and gains, and the law selection (`controller:`).
- **Planner** — `local_planner`, `nmpc_controller.*` params (body/feet/joints bounds; must match the URDF).

!!! warning "Sign and offset mistakes"
    A wrong joint sign manifests as a robot that drifts on stand or refuses to
    track. Verify the per-leg `name`/`sign`/`offset` mapping by commanding each
    joint individually with the manufacturer SDK first, then translate that
    polarity into the YAML.

!!! info "Where controller types live"
    The `controller_manager` `update_rate`, the controller TYPE map, the IMU
    broadcaster wiring, and the supervisor structure are robot-agnostic and stay
    in `quad_controllers/config/controllers.yaml`. Your `<robot>.yaml` only
    overrides the per-controller gains/poses under the same `/**/...` keys.

## Step 3 — Register the robot type in launch

Add a case for `<robot>` to the `robot_type` mapping in
`quad_utils/launch/robot_bringup.py` (and `quad_utils/launch/planning.py`):

```python
elif robot_type == '<robot>':
    desc_pkg = '<robot>_description'
    urdf_file = '<robot>.urdf.xacro'
    sdf_file = '<robot>.sdf.xacro'
    config_file = '<robot>.yaml'
```

The launch then deep-merges `controllers.yaml + <robot>.yaml + sim.yaml` into the
single config the Gazebo plugin and the controller spawners consume. Also add
`<robot>` to the description-package maps in `quad_utils/launch/robot_driver.py`
and `remote_driver.py` if you use the legacy `robot_driver` path.

Smoke test:

```bash
ros2 launch quad_utils quad_gazebo.py \
  robot_configs:='[{"name":"robot_1","type":"<robot>","controller":"inverse_dynamics","init_pose":"-x 0 -y 0 -z 0.5"}]'
```

The robot should spawn and report joint state. It won't stand yet — we still need
the NMPC files.

## Step 4 — Generate NMPC code

The local planner uses a robot-specific NMPC compiled from CasADi expressions.
Generate them once per robot:

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

And `nmpc_controller/src/quad_nlp_utils.cpp`: dispatch to the new leg-controller
functions.

## Step 6 — Update the local planner

In `local_planner/src/local_planner.cpp`, add a case mapping
`robot_name_ == "<robot>"` to the new `SystemID`.

## Step 7 — Hardware interface (skip if sim only)

You have two options:

- **Reuse an existing `SystemInterface`** — if your robot speaks Unitree DDS, point
  the URDF's hardware `<ros2_control>` block at `quad_hardware/UnitreeSystem`; if
  it speaks MBLink, use `quad_hardware/SpiritSystem`. No new code.
- **Add a new `SystemInterface`** — for a different bus, add a
  `hardware_interface::SystemInterface` plugin to `quad_hardware`, modeled on
  `unitree_system.cpp` / `spirit_system.cpp`:
    - Export five command interfaces (`position/velocity/kp/kd/effort`) + three
      state interfaces (`position/velocity/effort`) per joint, plus the `imu`
      sensor, to match the controllers' interface contract.
    - Honor the `read_only` param and the NaN-command guard so a read-only
      bring-up commands no torque.
    - Register it in `quad_hardware/quad_hardware_plugins.xml`, then reference
      the plugin name in the URDF's hardware `<ros2_control>` block.

Bring it up safely first (broadcasters only):

```bash
# adapt go2_hardware_readonly.launch.py for your robot/description package
ros2 launch quad_hardware <robot>_hardware_readonly.launch.py
ros2 topic echo /joint_states               # 12 joints, live encoder values
ros2 topic echo /imu_sensor_broadcaster/imu
```

(If you still use the legacy `robot_driver`, instead add a
`robot_driver/.../<robot>_interface.cpp` and a case in `robot_driver.cpp`.)

## Step 8 — Stand and tune

Build, source, and run the standing test:

```bash
colcon build --packages-up-to quad_controllers local_planner nmpc_controller
source install/setup.bash
ros2 launch quad_utils quad_gazebo.py control_stack:=ros2_control
ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 1" --once   # READY / stand
```

If the robot stands cleanly, add the planner and walk:

```bash
ros2 launch quad_utils quad_plan.py
ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 1" --once
```

Iterate on `<robot>.yaml` from there. Most tuning hours go into:

1. Stand gains (oscillation, sag)
2. NMPC GRF bounds (infeasible references)
3. Step timing (gait period, duty cycle)

[:octicons-arrow-right-24: Writing a control law](writing-controller.md) ·
[:octicons-arrow-right-24: Control Stack architecture](../architecture/control-stack.md)
