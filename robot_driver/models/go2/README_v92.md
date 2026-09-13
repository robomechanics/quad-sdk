# Underbrush V92 testing controller

Controller ID: `underbrush_v92`. `underbrush_v90` (alias `underbrush_learned`) retains the
10-value per-leg interface used by V90. Both use `UnderbrushPolicy`; V92
selects 13-value per-leg buffers and appends previous raw actions in
hip/thigh/calf order. Body input remains 21 values, hidden states remain
four [1, 1, 64] tensors, and output remains 12 raw actions.

V92 per-leg input: q-relative (3), velocity (3), received joint effort (3),
binary foot contact (1), previous raw action for this leg (3).
Isaac action indices are [leg, 4+leg, 8+leg], with FL/FR/RL/RR leg order.
Initial action history is zero. The body still receives all 12 previous
raw actions. Existing PD tracking, gains 25/0.5, action scale 0.5,
command gating, and state lifecycle are inherited.
No reset calls or additional warmup behavior are introduced.

Initialization validates the ONNX input/output names, types, and shapes;
a V90 model cannot be used with the V92 mode (or vice versa).

## Build and select

Build the updated `robot_driver` and `quad_utils` packages in the deployment
workspace and source its install/setup.bash before testing. This change was
compiled in an isolated /tmp build; no installation or robot launch was run.

Hardware launch, when ready to execute:

```bash
ros2 launch quad_utils robot_driver.py robot_type:=go2 controller:=underbrush_v92 model_path:=/absolute/path/to/v92_model.onnx
```

The `model_path` launch argument now overrides the parameter actually read
by the driver (`robot_driver.model_path`). Leaving it empty preserves the
YAML setting. For a planning launch, use `controller_mode: underbrush_v92`
in its robot configuration; this suppresses the MPC local planner just as
for other learned controllers. Select the matching driver controller too.

No V92 checkpoint was found under the local training log directory. Export
one when available, using the CPU environment used for V90:

```bash
CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONPATH=/home/rml/underbrush/source/underbrush \
/home/rml/anaconda3/envs/isaaclab/bin/python /home/rml/underbrush/scripts/tools/onnx_export/export_underbrush_onnx.py \
  --checkpoint /absolute/path/to/v92/model_N.pt \
  --output /absolute/path/to/v92_model.onnx --actor-type gru --per-leg-obs-dim 13
```

## V90 export produced alongside this change

`v90_model_42900.onnx` came from the latest completed checkpoint selected at
export start: `vinewalk_gru_v90/2026-09-11_17-08-07_v90_widepatch_3d_handoff1s_s42_20260911/model_42900.pt`.
Use controller `underbrush_v90` (or `underbrush_learned`) with this file.

ONNX checker passed. Over 100 sequential random-input steps, ONNX Runtime
CPU actions and all four hidden states matched PyTorch within rtol=2e-4,
atol=2e-5; maximum action absolute error was 2.6226043701171875e-06.
The exporter omits training-only auxiliary prediction heads and exploration
standard deviation, and requires every inference-path weight to match.
This validates export equivalence, not hardware locomotion performance.

## Simulator and hardware input routing

`is_hardware:=false`: estimator_plugin publishes RobotState on
`state/ground_truth`; RobotDriver receives q, dq, orientation, body angular
velocity, and `joints.effort` from that message. The existing flat-ground
contact_state_publisher publishes `state/grfs`; RobotDriver thresholds each
force-vector norm at >5 N and passes the resulting four contact bits to the
policy. Simulator plugins and their existing torque signs are unchanged.

`is_hardware:=true`: UnitreeInterface fills JointState.effort from
motor_state.tau_est and obtains binary contacts from its foot-force readings
using the configured hardware threshold (default >30 raw units). RobotDriver
copies those joints into RobotState and forwards contacts to the policy.

Both controller versions reorder Quad-SDK FL/RL/FR/RR into actor FL/FR/RL/RR,
scale received torque by 0.01, and use the received signal from the first
observation. The previously added PD-torque proxy is no longer the network
input. Raw hardware tau_est is not guaranteed numerically equivalent to the
Isaac actuator signal; this change implements the requested sensor routing.

For the planned flat-ground sim-to-sim test, select `underbrush_v90` or
`underbrush_v92` in the Gazebo robot configuration's `controller` field and
use the corresponding `controller_mode` for planning. Set the matching model
in robot_driver.yaml (`robot_driver.model_path`) or forward `model_path` to
the driver launch. `is_hardware=false` is supplied by robot_bringup.

Validation: isolated CPU build and observation test cover received effort
(including first inference), torque scaling, contact order, and the V92
previous-action fields. No simulator or hardware run was launched.

## Deployment debugging update (September 13, 16:43 bag)

The flat-ground bag reproduced the first ONNX action in CPU replay, ruling
out an export/action-order discrepancy for that sample. Changing only the
thigh torque sign did not remove the large first knee target.

Current fixes use CPU ONNX inference by default (optional provider=cuda),
and perform a disposable runtime preflight during initialization. Preflight
does not advance persistent hidden state or previous actions.

For explicit underbrush_v90/v92 simulation launches, robot_bringup enables
only additional actuator telemetry. The Gazebo controller publishes its
existing final motor torque on state/applied_joint_torques. RobotDriver uses
this received torque for the actor and checks timestamp and joint ordering.
The estimator continues publishing its original transmitted-wrench signal.
Hardware still uses Unitree tau_est.

Gazebo target clamping, PD error calculation, and torque limits are unchanged.
Standing targets remain sourced from the existing YAML. No hidden-state reset
calls or additional policy warmup/hold logic were added.

New bag topics: state/applied_joint_torques, policy/observations, and
policy/raw_actions. Observations are concatenated as body (21), then
FL/FR/RL/RR per-leg groups (10 or 13 each); raw actions retain Isaac
joint-type order. Existing command/clock topics provide timing context.
A fresh flat-ground bag is required to assess closed-loop behavior.
