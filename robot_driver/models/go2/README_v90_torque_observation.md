> Superseded for the current controller: at the user's request, the network
> now consumes RobotState.joints.effort from the simulator or Unitree interface,
> scaled by 0.01. The PD proxy described below is no longer used by the policy.
> See README_v92.md for current routing.

# V90 torque observation curve

The existing `UnderbrushPolicy::computeObservations()` PD-torque proxy now
uses V90's Go2HV envelope instead of V51's linear DC-motor curve:

```
requested = kp * (previous_active_joint_target - q) - kd * dq
limit = 20.2 if requested * dq > 0 else 23.4
if abs(dq) >= 13.5:
    limit *= max(0, (30 - abs(dq)) / 16.5)
observation = 0.01 * clamp(requested, -limit, limit)
```

The gains are the same stored gains used by the base controller (25/0.5).
The base class still controls inference timing and sends the processed joint
target, zero desired velocity, zero feedforward torque, and PD gains to the
motors. Observation calculation occurs before inference replaces the target.
First-inference torque stays zero, as it was before this change.

There is no added history buffer, artificial delay, reset behavior, or
controller-loop override. `resetHiddenStates()` has its pre-change behavior;
no calls were added. Raw Unitree `tau_est` logging is unchanged. The proxy
needs encoder position/velocity, not the raw effort array.

This matches the actuator formula, not necessarily the exact sample phase of
Isaac's last physics step. No hardware-transfer claim follows from this edit.
Other model, observation, action and standing settings remain as they were;
this does not select a V90 checkpoint or add V92's extra action inputs.

The CPU-only `underbrush_torque_observation_test` checks eleven analytic
points covering motoring, braking, both velocity signs, the taper boundary,
and zero torque at/above the no-load speed. Validation builds are isolated;
no driver was installed or launched and training was not restarted.
