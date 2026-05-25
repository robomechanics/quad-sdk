# quad_supervisor

The behavior-mode finite state machine for the switchable-controller ("Option B")
quad-sdk stack. A single `mode_supervisor` node reproduces `robot_driver`'s
control-mode FSM, but realizes each mode by **switching `ros2_control`
controllers** through the `controller_manager` instead of branching inside one
node.

> See the [Control Stack architecture page](https://github.com/robomechanics/quad-sdk/blob/main/docs/architecture/control-stack.md#mode-switching)
> for how mode switching fits the overall pipeline.

## Contents

`mode_supervisor` (`src/mode_supervisor.cpp`) — a plain `rclcpp::Node` (not a
controller). It is a faithful port of `robot_driver`'s `controlModeCallback` +
`checkMessagesForSafety` + the transition auto-advance.

### FSM

States (the same numeric mapping as `robot_driver`):

| State | Value | Active controller |
|---|---|---|
| `SIT` | 0 | `sit_controller` (`PoseController`) |
| `READY` | 1 | `locomotion_controller` (`LegControlController`, selected law) |
| `SIT_TO_READY` | 2 | `sit_to_ready_controller` (`TransitionController`, sit→stand) |
| `READY_TO_SIT` | 3 | `ready_to_sit_controller` (`TransitionController`, stand→sit) |
| `SAFETY` | 4 | `safety_controller` (pos-0 PD, safety gains) |

The node **starts in `SIT`** (matching `robot_driver`'s `control_mode_ = SIT`),
tracking `sit_controller` as the active one — the launch spawns it active so the
robot holds the sit pose from boot.

`switchTo()` calls the `controller_manager`'s `switch_controller` service to
**activate** the target controller and **deactivate every other** mode
controller (they all claim the same command interfaces, so only one may be active
at a time). It uses `BEST_EFFORT` strictness, so deactivating a controller that
isn't currently active is harmless. Transitions auto-advance after
`transition_duration` (sit→ready→locomotion, and ready→sit), exactly like
`robot_driver`.

## Key parameters

Controller-name mappings and timings come from `quad_controllers/config/controllers.yaml`;
`is_hardware` comes from the context overlay.

* `controller_manager` (string, default `/controller_manager`) — service namespace.
* `sit_controller` (string, default `sit_controller`)
* `locomotion_controller` (string, default `locomotion_controller`)
* `safety_controller` (string, default `safety_controller`)
* `sit_to_ready_controller` (string, default `sit_to_ready_controller`)
* `ready_to_sit_controller` (string, default `ready_to_sit_controller`)
* `transition_duration` (double, default `1.0`) — s before a transition auto-advances.
* `heartbeat_timeout` (double, default `0.2`) — s without a heartbeat before SAFETY.
* `state_timeout` (double, default `0.1`) — s without a state message before SAFETY (sim only — gated by `!is_hardware`).
* `is_hardware` (bool, default `false`) — arms the state-timeout safety branch only in sim.

## Topics

### Subscribed

* `control/mode` ([std_msgs/UInt8]) — commanded behavior (`0` SIT, `1` READY, `4` SAFETY; mid-transition commands are ignored).
* `heartbeat` ([std_msgs/Header]) — remote operator heartbeat; loss triggers SAFETY.
* `state/ground_truth` ([quad_msgs/RobotState]) — liveness signal for the (sim-only) state-timeout check.

### Service client

* `<controller_manager>/switch_controller` ([controller_manager_msgs/SwitchController]) — activates/deactivates mode controllers.

## How it's launched

Not run standalone — it is started alongside the controllers, after they spawn:

```bash
# Hardware (Go2) — supervisor started ~4 s after the controller_manager
ros2 launch quad_hardware go2_bringup.launch.py

# Simulation
ros2 launch quad_utils robot_bringup.py control_stack:=ros2_control
```

Drive it exactly like `robot_driver`:

```bash
ros2 topic pub /control/mode std_msgs/UInt8 "data: 1" --once   # READY
ros2 topic pub /control/mode std_msgs/UInt8 "data: 0" --once   # SIT
ros2 topic pub /control/mode std_msgs/UInt8 "data: 4" --once   # SAFETY
```

## Build

```bash
colcon build --packages-select quad_supervisor
```

## License

MIT. **Maintainer:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/),
Carnegie Mellon University.

[std_msgs/UInt8]: https://docs.ros.org/en/jazzy/p/std_msgs/
[std_msgs/Header]: https://docs.ros.org/en/jazzy/p/std_msgs/
[quad_msgs/RobotState]: ../quad_msgs/msg/RobotState.msg
[controller_manager_msgs/SwitchController]: https://docs.ros.org/en/jazzy/p/controller_manager_msgs/
