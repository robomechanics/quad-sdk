# quad_estimators

Standalone state-estimator nodes used on **hardware**, ported out of
`robot_driver`. Each fuses IMU + joint encoders + mocap into the control state
the planner and controllers read. In simulation these are not used — Gazebo's
`ground_truth_estimator` plugin publishes the state instead.

> See the [Control Stack architecture page](https://github.com/robomechanics/quad-sdk/blob/main/docs/architecture/control-stack.md#sim-vs-hardware-the-only-differences)
> for where these sit in the sim-vs-hardware pipeline.

## Contents

| Node | Executable | Wraps | Estimator |
|---|---|---|---|
| `ekf_estimator_node` | `ekf_estimator_node` | `quad_estimators::EKFEstimator` | Contact-aided extended Kalman filter (uses `QuadKD2` for FK/Jacobians). |
| `comp_filter_estimator_node` | `comp_filter_estimator_node` | `quad_estimators::CompFilterEstimator` | Complementary filter fusing IMU with mocap, with a mocap-dropout gate. |

Both mirror how `robot_driver` fed its estimators: `loadSensorMsg` /
`loadMocapMsg` then `updateOnce`, driven by a fixed-rate timer (`update_rate`,
default 500 Hz). Each builds a `QuadKD2` on its own node (so it needs
`robot_description` + the leg/frame params, or it fatals).

### Why nodes, not ros2_control broadcasters

These estimators are **topic-driven** (mocap, GRF, contact) rather than
hardware-interface-driven, so they don't fit the broadcaster model. The
`imu_sensor_broadcaster` republishes the hardware IMU as a topic; these nodes
then consume that topic (plus joints and mocap) and produce the fused state.
A broadcaster only reads exported hardware interfaces — it has no path to mocap.

## Inputs / outputs

### Subscribed topics

* `joint_states` ([sensor_msgs/JointState]) — joint encoders (from `joint_state_broadcaster`).
* `imu` ([sensor_msgs/Imu]) — remapped to `/imu_sensor_broadcaster/imu` at launch.
* `mocap` ([geometry_msgs/PoseStamped]) — optional motion-capture pose.
* GRF and contact topics — subscribed internally by the estimator's `init()` (as in `robot_driver`).

### Published topics

* `state/ground_truth` ([quad_msgs/RobotState]) — **the control state** the planner and controllers read; the hardware counterpart to Gazebo's `estimator_plugin`.
* `state/estimate` ([quad_msgs/RobotState]) — debug copy.

!!! info "One topic, two sources"
    Everything downstream reads `state/ground_truth`. On hardware this node fills
    it; in sim the Gazebo plugin fills it. `state/estimate` is debug-only.

## Key parameters

* `robot_ns` (string, default `robot_1`) — namespace for the robot-specific (leg/frame) params.
* `update_rate` (double, default `500.0`) — estimator loop rate in Hz.
* `robot_description` (string) — URDF, required by `QuadKD2`.

`comp_filter_estimator_node` only — the mocap-dropout gate:

* `mocap_rate` (double, default `360.0`) — expected mocap frame rate (must match the source).
* `mocap_dropout_threshold` (double, default `0.035`) — s; if the inter-message interval deviates from `1/mocap_rate` by more than this, the mocap velocity helper is skipped (a dropped frame won't corrupt the velocity estimate). The latest pose is always cached.

EKF/filter tuning (noise, bias, filter coefficients) is supplied via
`robot_driver/config/robot_driver.yaml` at launch.

## How it's launched

Hardware-only. The launch feeds `robot_description` + the robot leg config
(`go2.yaml`) + estimator tuning (`robot_driver.yaml`) and remaps `imu` to the IMU
broadcaster topic:

```bash
ros2 launch quad_hardware go2_estimator.launch.py estimator:=ekf         # or estimator:=comp_filter
```

## Build

```bash
colcon build --packages-select quad_estimators
```

## License

MIT. **Maintainer:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/),
Carnegie Mellon University.

[sensor_msgs/JointState]: https://docs.ros.org/en/jazzy/p/sensor_msgs/
[sensor_msgs/Imu]: https://docs.ros.org/en/jazzy/p/sensor_msgs/
[geometry_msgs/PoseStamped]: https://docs.ros.org/en/jazzy/p/geometry_msgs/
[quad_msgs/RobotState]: ../quad_msgs/msg/RobotState.msg
