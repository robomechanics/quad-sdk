# Go2 Sim-to-Real Debug Notes

Distilled from a debugging session on the Go2 + quad-sdk stack. Symptom: works in
Gazebo, fails on hardware — robot falls behind the plan, legs don't cycle,
NMPC eventually starts failing.

## TL;DR

The dominant root cause is **URDF mass error (13.7 kg modeled, 16.3 kg actual,
~19% under)**. Every downstream symptom traces back to this through one of two
paths: (a) systematic FF gravity/inertia under-commanding, or (b) NMPC
warm-start divergence as planned vs actual state grow apart.

Bound and gain edits in `go2.yaml` only mask or trade off the symptoms; they
do not fix the source.

## Symptoms observed (in order)

1. Hip motor (motor index 1) FF effort exceeds 23.7 Nm continuous limit during
   stance — `[robot_driver]: Leg N motor 1: ff effort = 27..30 Nm exceeds threshold of 23.700 Nm`.
2. Robot falls behind the planned trajectory. Moving the modeled CoM forward
   in stand pose helps slightly but doesn't close the gap.
3. NMPC solve times start at 1–5 ms, grow over a few seconds (17, 33, 70, 118 ms),
   then `[local_planner]: NMPC solving fail`.
4. Once NMPC fails repeatedly, legs stop cycling — robot holds stand pose
   while rviz shows the local plan still moving forward.

## Where parameters get injected into the control stack

### `go2.yaml` consumers

| YAML key | Read by | Used for |
|---|---|---|
| `motor_limits.torque/speed` | `robot_driver.cpp:79`, `controller_plugin.cpp:75-76` | Static τ clamp ([robot_driver.cpp:776](../../robot_driver/src/robot_driver.cpp#L776), [controller_plugin.cpp:388](../../../quad_simulator/gazebo_plugins/src/controller_plugin.cpp#L388)). Speed limits **only** enter the optional motor model in the Gazebo plugin which is **hardcoded off** ([controller_plugin.cpp:390](../../../quad_simulator/gazebo_plugins/src/controller_plugin.cpp#L390)). |
| `nmpc_controller.body.u_lb/u_ub` | `nmpc_controller.cpp:88-106`, `quad_nlp.cpp:179-180` | GRF bounds (z is index 2/5/8/11). Body NMPC plans GRFs that map to joint τ via J^T. |
| `nmpc_controller.joints.x_lb/x_ub` | same | First 12 elements = joint angles, last 12 = joint velocities (rad/s). |
| `nmpc_controller.body.x_weights[6:9]` | same | Linear velocity tracking weights (low → planner tolerates velocity lag). |
| `robot_driver.stance_kp/kd, swing_kp/kd` | `robot_driver.cpp:66-69` → `inverse_dynamics_controller.cpp:164-176` | Per-mode joint-space PD on top of FF τ. |
| `mass: 15.0` | `global_body_planner` only | Kinodynamic GRF unit conversion. **Does not enter Pinocchio.** |

### URDF (`robot_description` parameter) consumers

| Reader | Path | Effect |
|---|---|---|
| local_planner | `local_planner.cpp:14` → `quad_kd2.cpp:23` (`pinocchio::urdf::buildModelFromXML`) | NMPC body dynamics constraints (M from `crba`, gravity/Coriolis from `nle`). |
| robot_driver | `robot_driver.cpp:16` → `QuadKD` | FF τ in `inverse_dynamics_controller.cpp:164-176`. |
| Gazebo plugins | `estimator_plugin.cpp:163`, `controller_plugin.cpp:197` | Sim physics inertials. |

**Implication:** the URDF mass is in the joint-space mass matrix `M`
([quad_kd2.cpp:336](../../src/quad_kd2.cpp#L336)) and the gravity term `nle`
([quad_kd2.cpp:340](../../src/quad_kd2.cpp#L340)). Both flow into NMPC
constraints and robot_driver FF τ. Sim looks fine because Gazebo loads the
**same wrong URDF** — the controller's model and the simulated body are wrong
identically and the errors cancel.

## Why the failure cascades

1. URDF mass is 19% low → FF τ for gravity and body acceleration is 19% short.
2. Real robot sags / lags relative to NMPC's expectations.
3. NMPC re-anchors at the (lagged) measured state. Warm-start from previous
   solution is now further from feasibility w.r.t. new initial state.
4. Each solve takes longer. Solve time exceeds local_planner publish period.
5. `InverseDynamicsController::computeLegCommandArray`
   ([inverse_dynamics_controller.cpp:12-15](../../../robot_driver/src/controllers/inverse_dynamics_controller.cpp#L12-L15))
   rejects plans older than **100 ms** → robot_driver falls back to
   `stand_joint_angles` ([robot_driver.cpp:666-674](../../../robot_driver/src/robot_driver.cpp#L666-L674)).
6. NMPC eventually returns infeasible. No new plan → permanent fallback to
   stand → "legs don't cycle" while plan in rviz keeps trotting.

A second gate ([line 64-67](../../../robot_driver/src/controllers/inverse_dynamics_controller.cpp#L64-L67))
prints `"ID node couldn't find the correct ref state!"` (1 Hz throttled) when
plan time window is missed. Absence of that log means gate 2 isn't firing —
gate 1 (100 ms staleness) silently triggers without a log.

## Spirit40 → Go2 differences that bit us

| | Spirit40 | Go2 |
|---|---|---|
| Mass (stock) | ~13 kg | 15 kg (spec); **16.3 kg as-built** with Jetson + sensors + markers |
| Hip τ continuous | ~21 Nm | 23.7 Nm |
| Hip τ peak | ~45 Nm | ~33.5 Nm |
| Knee speed limit | 25 rad/s | 15.7 rad/s |
| Hip speed limit | 30 rad/s | 30 rad/s |

The yaml shipped with Spirit-derived NMPC joint velocity bounds (37.71/25.11
rad/s) that exceed Go2 spec (30/15.7). Go2 also has ~25% less peak-torque
headroom and is heavier per leg. So bounds and force budgets that worked on
Spirit are **infeasible** on Go2.

## Edits applied during the session (in `go2.yaml`)

- **NMPC joint velocity bounds tightened** to match Go2 motor spec (30/30/15.7
  rad/s hard, 24/24/12.5 rad/s soft). Previous Spirit values preserved as
  comments above the active lines.
- **`local_planner.desired_height` 0.22 → 0.24, `global_body_planner.h_nom`
  0.25 → 0.26** to keep nominal pose less extended (smaller J^T moment arm
  at hip).
- **`stance_kp` 40 → 30** to avoid amplifying FF saturation via PD feedback.
- **Body z-GRF cap reverted to 150** after experimenting with 100 N. 100 N
  was too tight for 16.3 kg actual mass (each stance leg in trot needs ~80 N
  for gravity alone).

These changes reduce one symptom each but **none of them fix the URDF mass
error**, which remains the dominant cause of warm-start divergence.

## Diagnostic toolbox

```bash
# Is the local plan actually fresh on the wire?
ros2 topic echo /robot_1/local_plan --field header.stamp --once
date +%s.%N

# Is the controller falling back to stand silently?
ros2 launch ... 2>&1 | grep -i "ID node couldn't find"

# Are motors saturating?
ros2 launch ... 2>&1 | grep -i "exceeds threshold"

# Solve time progression (look for monotonic blowup → NMPC fail)
ros2 launch ... 2>&1 | grep "LocalPlanner took"

# Time-source sanity (use_sim_time should be false on hardware, /clock silent)
ros2 param get /robot_1/local_planner use_sim_time
ros2 param get /robot_1/robot_driver use_sim_time
ros2 topic hz /clock

# Confirm only one publisher on critical topics
ros2 topic info /robot_1/control/joint_command --verbose
```

## Open work

1. **Fix URDF inertial.** Update trunk mass to 16.3 kg (or add a `payload`
   link with mass 2.6 kg at the actual mounting location of Jetson + battery
   + markers). This single change propagates to: NMPC dynamics constraints
   in local_planner, FF τ in robot_driver, and Gazebo physics. Also bump
   `mass: 15.0 → 16.3` in `go2.yaml` for global planner consistency.
2. **Re-evaluate bound choices once URDF is correct.** With the right mass,
   NMPC's GRF demands change, and the right cap may differ from 150.
3. **Consider enabling motor speed-torque envelope** in robot_driver
   ([robot_driver.cpp:776](../../../robot_driver/src/robot_driver.cpp#L776))
   so commanded τ matches hardware capability. The Gazebo plugin has the
   same envelope at [controller_plugin.cpp:384-394](../../../quad_simulator/gazebo_plugins/src/controller_plugin.cpp#L384-L394)
   but `apply_motor_model = false` is hardcoded.
4. **State estimation latency.** Mocap pipeline adds 15–40 ms vs sim's 0 ms.
   Not yet quantified for this setup; worth measuring once URDF is fixed.

## Things to *not* do

- **Don't raise `motor_limits.torque[1]` from 23.7 to 33.5** to silence
  saturation warnings. 23.7 is continuous; 33.5 is peak. Sustained operation
  at peak overheats the motor or trips firmware current limits.
- **Don't enable the Gazebo motor model** to "expose" the saturation in sim.
  That makes sim worse without fixing real. Match real to the model, not the
  other way around.
- **Don't tighten NMPC bounds further to "force feasibility."** Tighter
  bounds cost solver headroom; once warm-start drifts (because of the model
  error), tighter bounds make solves fail faster.
