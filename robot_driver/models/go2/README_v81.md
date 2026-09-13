# v81 ONNX sim-to-sim deployment

This directory contains the deterministic actor exported from the final v81
checkpoint:

- source version: **v81** (remote-consistent name)
- historical run directory: `vinewalk_gru_v75` (the run was completed before
  the local v75 name was renumbered to v81)
- checkpoint: `model_49999.pt`
- ONNX: `v81_policy_model_49999.onnx`
- SHA-256:
  `fb8b5ead7b2d6dbd83a87a7901dfc2f13b362f5343287cec766fcdc4acf2e007`

Do not identify this policy as remote v75. Remote-consistent v75 is a different
configuration.

## ONNX interface

The model has nine `float32` inputs and five `float32` outputs. Batch size one
is the intended deployment configuration.

| Name | Shape | Meaning |
|---|---:|---|
| `per_leg_FL` | `[1, 10]` | front-left leg observation |
| `per_leg_FR` | `[1, 10]` | front-right leg observation |
| `per_leg_RL` | `[1, 10]` | rear-left leg observation |
| `per_leg_RR` | `[1, 10]` | rear-right leg observation |
| `body` | `[1, 21]` | body and command observation |
| `h_FL`, `h_FR`, `h_RL`, `h_RR` | `[1, 1, 64]` each | input GRU states |
| `actions` | `[1, 12]` | deterministic raw policy actions |
| `h_FL_out`, `h_FR_out`, `h_RL_out`, `h_RR_out` | `[1, 1, 64]` each | next GRU states |

Feed each `h_*_out` back as the corresponding `h_*` on the next policy step.
Initialize all four states to zero at startup and reset them to zero on an
episode reset or controller-mode transition.

### Observation order and scaling

Each leg vector is:

```text
[q_rel hip/thigh/knee (3),
 dq hip/thigh/knee * 0.05 (3),
 measured torque hip/thigh/knee * 0.01 (3),
 binary foot contact (1)]
```

Leg order is `FL, FR, RL, RR`. In Quad-SDK, which stores legs as
`FL, RL, FR, RR`, the Isaac-to-Quad leg map is `{0, 2, 1, 3}`. Use Unitree
`motor_state[].tau_est` as measured torque. The contact input must be exactly
`0.0` or `1.0`; Isaac training used `foot force > 5 N`. Deployment may use the
already-thresholded Unitree contact state.

The body vector is:

```text
[base angular velocity xyz * 0.2 (3),
 projected gravity xyz in the body frame (3),
 commanded vx, vy, yaw rate (3),
 previous raw policy action in Isaac joint order (12)]
```

The command bounds used by the Quad-SDK implementation are `vx [-1, 1]`,
`vy [-0.4, 0.4]`, and yaw rate `[-1, 1]`. The action is a normalized joint
offset: multiply by `0.5`, add the nominal joint pose, and then reorder from
Isaac to Quad-SDK joint order. Keep the previous **raw** action—not the scaled
joint target—for the next body observation.

## Quad-SDK Gazebo sim-to-sim test

The matching runtime is `robot_driver/controllers/underbrush_policy.cpp` in
`~/ros2_ws/src/quad-sdk`.

1. Copy or symlink the model into Quad-SDK and set
   `robot_driver.model_path` in `robot_driver/config/robot_driver.yaml` to its
   absolute path.

2. Before building, change `UnderbrushPolicy::kGRUHidden` in
   `robot_driver/include/robot_driver/controllers/underbrush_policy.hpp` from
   its current value of `256` to **`64`**. The exported v81 checkpoint uses 64
   hidden units. Leaving it at 256 gives the ONNX runtime the wrong tensor
   shape and cannot produce a valid comparison.

3. Build the workspace with ONNX Runtime enabled, then source it:

   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install --packages-up-to robot_driver quad_utils
   source install/setup.bash
   ```

4. Select `underbrush_learned` as the robot's `controller_mode` in the Gazebo
   launch robot configuration, then launch simulation:

   ```bash
   ros2 launch quad_utils quad_gazebo.py
   ros2 topic pub /robot_1/control/mode std_msgs/msg/UInt8 "{data: 1}" --once
   ros2 launch quad_utils quad_plan.py \
     robot_configs:='[{"name":"robot_1","type":"go2","controller_mode":"underbrush_learned","reference":"twist","twist_input":"keyboard"}]'
   ```

5. Start with a zero command for two to three seconds so stance behavior and
   zero-state recurrent initialization can be checked before commanding
   forward motion. The policy inference rate should be **50 Hz**; the lower
   level PD command loop may remain at its normal higher rate.

For an actual sim-to-sim comparison, log at least command velocity, body pose
and twist, joint position/velocity/effort, binary contact, raw policy action,
post-processed joint target, controller mode, and termination/reset reason.

## Re-export

Run from the underbrush repository:

```bash
source ~/anaconda3/etc/profile.d/conda.sh
conda activate isaaclab
export PYTHONPATH="$PWD/source/underbrush:${PYTHONPATH:-}"
python scripts/tools/onnx_export/export_underbrush_onnx.py \
  --checkpoint scripts/rsl_rl/logs/rsl_rl/vinewalk_gru_v75/2026-08-29_19-07-39_v75_binary_contact_fromscratch_20260829_1845/model_49999.pt \
  --output deploy_onnx/v81/v81_policy_model_49999.onnx \
  --actor-type gru --gru-hidden-dim 64
```

The exporter deliberately loads the actor without starting Isaac Sim. It also
uses explicit ONNX hidden-state inputs and outputs; older exports from this
script could incorrectly capture mutable PyTorch GRU state and omit all four
`h_*` inputs.

## Validation performed

The file passes `onnx.checker.check_model`. Its graph exposes all nine required
inputs and five outputs. A three-step recurrent comparison against the loaded
PyTorch actor, feeding each step's returned hidden states into the next step,
had a maximum absolute difference of `1.39e-6`.
