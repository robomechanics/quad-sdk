# quad_hardware

`ros2_control` hardware components for quad-sdk robots. These are
`hardware_interface::SystemInterface` plugins, so the `controller_manager` loop
drives a real robot the same way it drives Gazebo (`gz_ros2_control`). The
control (write) path is implemented; it is gated by `read_only` so a new robot
can be brought up safely (broadcasters only) before any torque is commanded.

> See the [Control Stack architecture page](https://github.com/robomechanics/quad-sdk/blob/main/docs/architecture/control-stack.md)
> for how these plugins fit the sim/hardware pipeline and how the
> `controller_manager` runs them.

## Contents

| Component | Plugin | Robot | Bus |
|---|---|---|---|
| `UnitreeSystem` | `quad_hardware/UnitreeSystem` | Go2 / Go2-W | Unitree SDK2 DDS (`rt/lowcmd`, `rt/lowstate`) |
| `SpiritSystem` | `quad_hardware/SpiritSystem` | Ghost Robotics Spirit 40 | MBLink mainboard |

Both are registered in `quad_hardware_plugins.xml`, expose the same interface
shape, share the same safety model, and mirror the legacy `robot_driver`
`UnitreeInterface` / `SpiritInterface` field-for-field.

### Interface shape (per joint)

Both robots' motors run an **onboard PD loop**:

```
tau_motor = kp * (position - q_meas) + kd * (velocity - dq_meas) + effort
```

So each of the 12 joints exposes **five command interfaces**
(`position`, `velocity`, `kp`, `kd`, `effort`) and **three state interfaces**
(`position`, `velocity`, `effort`). A controller in **motor mode** that writes
all five reproduces the legacy `LegCommandArray` → `send()` path exactly. Each
plugin also exports an **IMU sensor** (`imu`) with `orientation.{x,y,z,w}`,
`angular_velocity.{x,y,z}`, `linear_acceleration.{x,y,z}` state interfaces, which
the stock `imu_sensor_broadcaster` republishes as `sensor_msgs/Imu`.

`UnitreeSystem` resolves each URDF numeric joint name to a Unitree LowCmd/LowState
motor index; `SpiritSystem` maps each name to an MBLink `(leg, joint-in-leg)`
slot and a per-joint torque constant (knees `1.092`, others `0.546`).

## Safety model

`write()` sends a **zero-gain command (no torque)** whenever:

- the hardware parameter `read_only` is `true`, **or**
- any of a joint's command interfaces is NaN (no controller has written it).

So a broadcaster-only bring-up cannot command torque even though the
`controller_manager` calls `write()` every cycle. The hardware backend and
`read_only` are selected in the robot's URDF `<ros2_control>` block via the
`use_sim` / `read_only` xacro args.

## Configuration layering

The hardware launches compose the effective config from three files (later
overrides earlier):

| File | Scope | Holds |
|---|---|---|
| `quad_controllers/config/controllers.yaml` | node | `controller_manager` `update_rate`, controller TYPE map, `imu_sensor_broadcaster` wiring, `mode_supervisor` structure. |
| `quad_utils/config/<robot>.yaml` (e.g. `go2.yaml`) | robot | gains, poses, joint names, kinematics/limits, planners. |
| `quad_controllers/config/hardware.yaml` | context | `interface_mode: motor`, `is_hardware: true`. |

A **read-only** bring-up only needs `controllers.yaml + hardware.yaml`
(broadcasters claim no per-robot gains); the full stack adds `<robot>.yaml`.

## Launch files

| Launch | What it brings up |
|---|---|
| `go2_hardware_readonly.launch.py` | `controller_manager` + `UnitreeSystem` (read-only) + `joint_state_broadcaster` + `imu_sensor_broadcaster` only. No command interface is claimed — **safe first bench test.** |
| `spirit_hardware_readonly.launch.py` | Same, with `SpiritSystem`. |
| `go2_bringup.launch.py` | Full stack: `UnitreeSystem` (motor mode) + both broadcasters + the five switchable controllers (`sit_controller` active, the rest inactive) + `mode_supervisor`. |
| `go2_estimator.launch.py` | The hardware state estimator (`estimator:=ekf` \| `comp_filter`) from `quad_estimators`. |

### Test 1 — read-only bring-up (do this first)

**Go2** — run on the robot (the DDS link needs the robot's `eth0`):

```bash
ros2 launch quad_hardware go2_hardware_readonly.launch.py
# optional: network_interface:=<iface>
```

**Spirit 40** — run with the mainboard reachable over MBLink:

```bash
ros2 launch quad_hardware spirit_hardware_readonly.launch.py
```

Verify:

```bash
ros2 control list_hardware_components       # UnitreeSystem / SpiritSystem -> active
ros2 control list_controllers               # joint_state_broadcaster, imu_sensor_broadcaster -> active
ros2 topic echo /joint_states               # 12 joints, live encoder positions/velocities
ros2 topic echo /imu_sensor_broadcaster/imu # live orientation / angular velocity / accel
```

**What this proves:** the SystemInterface's bus wiring, the joint↔motor index
map, and the state/sign conventions are correct — with zero risk to the robot.
Compare `/joint_states` against the legacy `robot_driver` output for the same
pose to confirm parity.

### Test 2 — full bring-up

```bash
ros2 launch quad_hardware go2_bringup.launch.py
ros2 launch quad_hardware go2_estimator.launch.py estimator:=ekf   # state/ground_truth
ros2 topic pub /control/mode std_msgs/UInt8 "data: 1" --once       # READY (0 SIT, 4 SAFETY)
```

Choose the READY-mode law via `locomotion_controller`'s `controller` param in
`quad_utils/config/go2.yaml`. Locomotion also needs the planner (`local_plan`)
and a state estimate. Validate on a stand first.

## Build

The package links the Unitree SDK2 from `quad-sdk/external/unitree_sdk2`
(the same tree `robot_driver` uses) and MBLink for Spirit:

```bash
colcon build --packages-select quad_hardware
```

## License

MIT. **Maintainer:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/),
Carnegie Mellon University.
