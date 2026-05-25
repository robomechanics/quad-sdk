---
title: Control Stack (ros2_control)
tags:
  - architecture
  - ros2
  - control
  - estimation
---

# Control Stack (ros2_control)

Quad-SDK's on-robot control runs on **`ros2_control`**. The same control laws, estimator, and planner run in **Gazebo simulation** and on **hardware** — the only things that change are the *hardware interface backend* and a two-line run-context overlay. This page explains the data pipeline, how the `controller_manager` loads and switches controllers, how everything talks to the hardware/sim, and exactly how the sim and hardware pipelines differ.

!!! info "Relationship to `robot_driver`"
    The legacy monolithic [`robot_driver`](../packages/robot_driver.md) node still exists and is still the default (`control_stack:=robot_driver`). The ros2_control stack (`control_stack:=ros2_control`) is the modern, switchable-controller path that reuses the *same* hardware-tested control laws. Both are faithful to the original logic; pick one at launch.

## Players at a glance

| Component | Package | Type | Role |
|---|---|---|---|
| `LegControlController` | `quad_controllers` | controller (plugin) | READY mode — runs a selectable leg-control **law**, falls back to a stand pose |
| `PoseController` | `quad_controllers` | controller (plugin) | SIT mode — holds a fixed joint pose |
| `SafetyController` | `quad_controllers` | controller (plugin) | SAFETY mode — damping-only PD |
| `TransitionController` | `quad_controllers` | controller (plugin) | SIT↔READY interpolation |
| control **laws** | `quad_controllers/laws/` | C++ classes | `inverse_dynamics`, `grf_pid`, `joint`, `underbrush`, `inertia_estimation`, `learned` — verbatim ports of the `LegController` hierarchy |
| `mode_supervisor` | `quad_supervisor` | node | the FSM — switches which controller is active via the `controller_manager` |
| `ekf` / `comp_filter` estimator | `quad_estimators` | node | hardware state estimation → `state/ground_truth` |
| `UnitreeSystem` / `SpiritSystem` | `quad_hardware` | hardware interface (plugin) | talks to the real robot (DDS / MBLink) |
| `GazeboSimSystem` | `gz_ros2_control` (upstream) | hardware interface (plugin) | talks to Gazebo |
| `QuadKD2` | `quad_utils` | library | Pinocchio kinematics/dynamics, instantiated by anything that needs it |

## The data pipeline

The control state flows on **one topic in both sim and hardware**: `state/ground_truth`. Whatever produces state publishes there, so everything downstream is source-agnostic. (`state/estimate` exists too, but it is **debug-only** — nothing in the control path reads it.)

```
                    ┌──────────────────────── state/ground_truth ───────────────────────┐
                    │                          (RobotState, the control state)           │
                    ▼                                                                     │
            ┌───────────────┐   local_plan    ┌──────────────────────┐  joint cmd        │
 cmd_vel ─► │ local_planner │ ──────────────► │ locomotion_controller│ ─► hardware iface ─┤─► actuators
 (goal) ──► │  + nmpc       │ ◄────────────── │  (law + QuadKD2)      │   (effort | motor) │
            └───────────────┘  state feedback └──────────────────────┘                    │
                    ▲                                   ▲                                  │
                    │                          /control/mode                              │
                    │                                   │                                  │
              terrain_map                      ┌─────────────────┐  switch_controller      │
                                               │ mode_supervisor │ ───────────────────────►│ (CM)
                                               └─────────────────┘
                                                                                           │
  sensors ──► estimator/plugin ───────────────────────────────────────────────────────────┘
```

Topic by topic (all relative names, so they auto-namespace per robot — see [Multi-robot](#multi-robot-namespacing)):

| Topic | Type | From → To |
|---|---|---|
| `state/ground_truth` | `quad_msgs/RobotState` | estimator (hw) / `estimator_plugin` (sim) → planner + controllers |
| `state/estimate` | `quad_msgs/RobotState` | estimator → *(debug only)* |
| `body_plan` | `quad_msgs/RobotPlan` | `global_body_planner` → `local_planner` |
| `local_plan` | `quad_msgs/RobotPlan` | `local_planner` → `locomotion_controller` |
| `cmd_vel` | `geometry_msgs/Twist` | teleop/goal → `local_planner` + law |
| `control/mode` | `std_msgs/UInt8` | operator → `mode_supervisor` |
| `control/joint_command` | `quad_msgs/LegCommandArray` | controller → *(echo/debug; the controller writes torques straight to the command interfaces)* |
| `imu` (hw) | `sensor_msgs/Imu` | `imu_sensor_broadcaster` → estimator |

The controller does **not** publish torques on a topic for someone else to apply — it writes them directly to the hardware interface's command interfaces (that's the whole point of ros2_control). `control/joint_command` is published only as an observable echo.

### QuadKD2 must be primed every cycle

The law calls `quadKD_->computeInverseDynamics(...)` and `getJacobianBodyAngVel(...)`, both of which read `QuadKD2`'s internal Pinocchio configuration. The owning controller refreshes that configuration **before** running the law each tick:

```cpp
quad_utils::updateDynamics(*quadKD_, *last_robot_state_msg_);  // sets M, N, J at the current state
```

Skipping this leaves Pinocchio at a stale/default configuration → singular dynamics → zeroed torque → the robot drops. (`robot_driver` does the identical step in `testDynamics()`.)

## How `controller_manager` runs everything

`controller_manager` (CM) is the heart of ros2_control. It owns the real-time update loop, loads the **hardware interface**, and loads/activates **controllers** that read and write that hardware's interfaces.

### 1. Interfaces are the contract

The robot's URDF `<ros2_control>` block declares, per joint, the **command interfaces** a controller may write and the **state interfaces** it may read, plus any **sensors**:

```xml
<ros2_control name="UnitreeSystem" type="system">
  <hardware><plugin>quad_hardware/UnitreeSystem</plugin></hardware>
  <joint name="0">
    <command_interface name="position"/> <command_interface name="velocity"/>
    <command_interface name="kp"/> <command_interface name="kd"/> <command_interface name="effort"/>
    <state_interface name="position"/> <state_interface name="velocity"/> <state_interface name="effort"/>
  </joint>
  ...
  <sensor name="imu">
    <state_interface name="orientation.x"/> ... <state_interface name="linear_acceleration.z"/>
  </sensor>
</ros2_control>
```

The hardware-interface plugin (`UnitreeSystem`, `GazeboSimSystem`, …) implements `export_command_interfaces()` / `export_state_interfaces()` to back each declared interface with a memory location, and `read()`/`write()` to move data to/from the robot or sim each cycle. The CM registers them all by name (`"0/position"`, `"imu/orientation.x"`, …) in its resource manager.

### 2. The update loop

Every cycle (`update_rate: 500` Hz, set in `controllers.yaml`):

```
hardware.read()  →  for each active controller: controller.update(time, period)  →  hardware.write()
```

`time` here is the **authoritative control clock** the CM provides — sim time under `gz_ros2_control`, the steady clock on hardware.

### 3. Controllers claim interfaces

Each controller declares which interfaces it needs:

- `QuadControllerBase::command_interface_configuration()` claims the joint command interfaces — in **effort mode** just `<joint>/effort`; in **motor mode** `position/velocity/kp/kd/effort`.
- `state_interface_configuration()` claims `<joint>/position` and `/velocity` for PD feedback.
- The stock `imu_sensor_broadcaster` claims `imu/*` and republishes them as `sensor_msgs/Imu`.

A broadcaster is just a controller that only reads. **It is not created by the hardware interface** — the CM loads it separately (via a spawner) and wires it to the hardware's exported interfaces by name. The hardware exports data; the broadcaster publishes it; the CM is the matchmaker.

### 4. Spawning

A *spawner* is a small process that asks the CM (over its services) to load, configure, and (optionally) activate a controller by name, pulling its parameters from the `--param-file`(s) you pass. The launch files spawn:

- `joint_state_broadcaster` (always),
- `imu_sensor_broadcaster` (hardware only — sim gets IMU/state from the `estimator_plugin`),
- `sit_controller` **active** (so the robot holds sit from spawn, matching `robot_driver`'s `control_mode_ = SIT`),
- `locomotion_controller`, `safety_controller`, `sit_to_ready_controller`, `ready_to_sit_controller` **inactive**,
- `mode_supervisor` (a normal node, not a controller).

### 5. Mode switching

All five mode controllers are loaded at once but only **one is active at a time**. The `mode_supervisor` is the FSM (`SIT=0, READY=1, SIT_TO_READY=2, READY_TO_SIT=3, SAFETY=4`). On a `control/mode` command (or a safety trip) it calls the CM's `switch_controller` service to **activate the target and deactivate the others**, and auto-advances the transition controllers (sit→ready→locomotion). Because only one controller holds the command interfaces at a time, there is never a write conflict.

```
control/mode=1 (READY)
   └─ supervisor: switch_controller(activate=[sit_to_ready], deactivate=[sit])
        └─ (after duration) switch_controller(activate=[locomotion], deactivate=[sit_to_ready])
```

!!! tip "Inspecting live state"
    ```bash
    ros2 control list_controllers -v          # which controllers are loaded/active + their interfaces
    ros2 control list_hardware_interfaces      # every command/state interface and who claims it
    ```

## Sim vs hardware: the only differences

Everything above is identical in both worlds. Three things change, and they are isolated:

| Axis | Simulation | Hardware |
|---|---|---|
| **Hardware interface** | `gz_ros2_control/GazeboSimSystem` | `quad_hardware/UnitreeSystem` (Go2) / `SpiritSystem` |
| **Selected by** | `use_sim:=true` in the URDF xacro (picks the `<ros2_control>` backend) | `use_sim:=false` |
| **`interface_mode`** | `effort` — Gazebo joints have *no* onboard PD, so the controller collapses `τ_ff + kp·e + kd·ė` into a single torque written to `effort` | `motor` — the motors have an onboard PD loop, so the controller writes `position/velocity/kp/kd/effort` and the motor closes PD at ~kHz |
| **State source** | Gazebo `estimator_plugin` publishes `state/ground_truth` directly | `quad_estimators` (`ekf`/`comp_filter`) node fuses IMU + joints + mocap → `state/ground_truth` |
| **CM is started by** | the `gz_ros2_control` plugin *inside* the Gazebo process (params via the SDF's `<parameters>` tag) | a standalone `ros2_control_node` process (params via its `parameters=[...]`) |
| **`is_hardware`** | `false` (supervisor sim branch) | `true` (arms heartbeat/timeout safety) |

```
SIM                                   HARDWARE
Gazebo estimator_plugin               IMU + joints + mocap
        │                                     │
        ▼                                     ▼
   state/ground_truth   ◄── same topic ──►  ekf/comp_filter ─► state/ground_truth
        │                                     │  └─► state/estimate (debug)
        ▼                                     ▼
   planner + controllers (identical)     planner + controllers (identical)
        │                                     │
        ▼                                     ▼
   GazeboSimSystem (effort)              UnitreeSystem (motor)
        │                                     │
        ▼                                     ▼
   Gazebo physics                        Go2 actuators (onboard PD)
```

The `read_only` flag on the hardware plugin gives a **safe bench bring-up**: the interface reads state but rejects all writes, so no torque is ever commanded. Use the read-only launch first on any new hardware.

## Configuration layering

Parameters are split by *what varies*, then merged at launch (later files override earlier):

| File | Scope | Holds |
|---|---|---|
| `quad_controllers/config/controllers.yaml` | **node** (every robot, sim+hw) | CM `update_rate` + controller type map, broadcaster wiring, `mode_supervisor` structure, fixed `robot_state_topic` |
| `quad_utils/config/<robot>.yaml` (e.g. `go2.yaml`) | **robot** | gains, poses, joint names, kinematics/limits, planners |
| `quad_controllers/config/sim.yaml` | **context** | `interface_mode: effort`, `is_hardware: false` |
| `quad_controllers/config/hardware.yaml` | **context** | `interface_mode: motor`, `is_hardware: true` |

In sim the launch **deep-merges** `controllers + <robot> + sim` into the single file the Gazebo plugin loads; on hardware the `ros2_control_node` is simply handed the three files as a list. Either way, controller gains live in exactly one place per robot, and switching sim↔hardware is a one-overlay change.

## Multi-robot namespacing

Every controller/estimator topic and param key is **relative** or `/**/`-wildcarded, so launching under a namespace (`robot_1`, `robot_2`, …) gives each robot an isolated stack: `/robot_1/state/ground_truth`, `/robot_1/controller_manager`, etc. The same `controllers.yaml` composes cleanly under any namespace. See the [Multi-Robot tutorial](../tutorials/multi-robot.md).

## Where to go next

- [Writing a control law](../tutorials/writing-controller.md) — add a new `LegController` law and select it.
- [Adding a robot](../tutorials/adding-a-robot.md) — description, config, and hardware interface for a new platform.
- [`quad_controllers`](../packages/quad_controllers.md), [`quad_estimators`](../packages/quad_estimators.md), [`quad_supervisor`](../packages/quad_supervisor.md), [`quad_hardware`](../packages/quad_hardware.md) package references.
