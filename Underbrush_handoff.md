# Underbrush Controller — Debug Handoff

**Date:** 2026-09-02 · **Branch:** `devel_ros2_sys_id` · **Robot:** Unitree Go2 (hardware, `unitree-jetson-payload`)
**Status:** Three bugs found and fixed. All changes **uncommitted**. Gait is walking normally again.
**Next task:** obtain an original ROS 1 bag from the IROS Underbrush work to explain *why these bugs did not
surface before* — see [Open Task](#open-task-ros-1-bag-analysis).

---

## 1. Background — the two controllers

At 500 Hz `robot_driver` converts a **local plan** (a ~0.78 s lookahead of robot states, each timestamped)
into per-motor commands: target angle, target speed, feedforward torque, `kp`, `kd`. Each of 4 legs has 3
motors — index 0 = **abad** (sideways), 1 = **hip** (fore/aft), 2 = **knee**. Each foot is in **stance** or
**swing**.

| controller | file |
|---|---|
| `InverseDynamicsController` (ID) | `robot_driver/src/controllers/inverse_dynamics_controller.cpp` |
| `UnderbrushInverseDynamicsController` (UB) | `robot_driver/src/controllers/underbrush_inverse_dynamics.cpp` |

**ID** tracks the plan directly. **UB** (Yim, Ren, Ologan, Gonzalez & Johnson, IROS 2023 — *Proprioception and
reaction for walking among entanglements*) adds reactive leg-disentanglement: it watches a **body force
estimator** for signs a leg has snagged and switches that leg into a RETRACT mode that drags it back over the
obstruction.

### `force_mode_` — the core state

Per-leg flag, `0` = NORMAL swing, `1` = RETRACT.

| joint | NORMAL | RETRACT |
|---|---|---|
| abad | tracks reference | still position-tracks (`swing_kp_[0]`) |
| hip | tracks reference | `kp = 0`, open-loop `vel_setpoint = -retract_vel_ * hip_retract_sign_` |
| knee | tracks reference | `kp = 0`, `kd = 0`, constant `torque_ff = -tau_push_` |

Transitions: `0→1` when estimated knee torque exceeds `tau_contact_start_` (or hip torque does and the leg has
been swinging longer than `t_up_`); `1→0` when knee torque falls below `tau_contact_end_`, or when
`t_TD_ - now < t_down_` (out of time, foot must land). `min_switch_` debounces; `last_mode_` latches retract
through the first `t_up_` of a new swing.

### USER SPEC (drove all of this)

> When `tau_contact_start` is set high so `force_mode_` can never activate, UB must behave **exactly** like ID.

This is the bisection baseline: if the two agree with obstruction disabled, any remaining difference is
attributable to the retract path alone. **The code did not satisfy this.** It does now.

---

## 2. Changes made — all in `robot_driver/src/controllers/underbrush_inverse_dynamics.cpp`

Four functional edits. Nothing else in the repo changed functionally.

### Fix 1 — Time-base mismatch (lines 333, 464) — THE REAL BUG

Both footfall-search loops compared a **plan-relative** time against an **absolute** ROS timestamp:

```cpp
// BEFORE
if (t_now < rclcpp::Time(states[j].header.stamp).seconds() && ...contact)
// t_now       = now() - plan->state_timestamp   -> ~0.0 to 0.5
// stamp       = absolute epoch seconds          -> ~1.788e9
// => always true; the time filter did nothing
```

The condition collapsed to just `...contact`, so the search returned **the first contact state anywhere in the
buffer**, scanning from index 0 — including states in the past. The interpolation loop ~90 lines above does it
correctly (it subtracts `t_first_state`); these two loops omitted that.

```cpp
// AFTER (both sites)
if (t_now < (rclcpp::Time(states[j].header.stamp) - t_first_state).seconds() && ...contact)
```

**Effect:** the plan buffer begins `t_now` seconds in the past (`t_now` = sensing + MPC solve latency). Right
after a leg lifted off, the buffer's leading states still showed it in stance, so the "next foothold" resolved
to the footstep it had **just left**. The foot was commanded backward and downward for the first ~30–100 ms of
every swing, then the target teleported ~20 cm forward once those states aged out.

```
 29 ms into swing:  footfall_err.x = -0.0053 m   (5 mm BEHIND the foot)
 98 ms into swing:  footfall_err.x = +0.1956 m   (196 mm AHEAD)   <- 20 cm jump
```

Knock-on: feet landed wrong → body displaced → stiff stance legs (`stance_kp = 60`) fought it, demanding up to
**80 Nm** against a **33.5 Nm** limit. This fix alone eliminated all **160** torque-saturation events.

### Fix 2 — Swing-mode state machine ran too late (line 219)

Original order: (1) interpolate plan → (2) rewrite swing reference → (3) commit → (4) inverse dynamics →
(5) command loop, **where `force_mode_` was decided**. Step 2 could not consult a flag decided in step 5.

Moved the ~70-line state machine out of the command loop to just after the touchdown scan, between steps 1
and 2. Added `if (contact) continue;` at the top, which exactly reproduces the old placement (it had lived
inside the `else` of `if (contact_mode[i])`, so it only ever ran on swing legs). All inputs — `t_now2`,
`t_switch_`, `t_LO_`, `t_TD_`, the force estimate, contact flags — are settled by that point. **No behavior
change alone**; it enables Fix 3.

### Fix 3a — Gate the rewrite on `force_mode_` (line 608)

The rewrite was conditioned only on `if (!ref_state_msg_.feet.feet.at(i).contact)` — "is this leg in the
air?" — never on "is it snagged?". Added, immediately before `ref_state_msg_ = ref_underbrush_msg;`:

```cpp
for (int i = 0; i < num_feet_; ++i) {
  if (force_mode_.at(i)) continue;              // retracting: keep underbrush motion
  ref_underbrush_msg.feet.feet.at(i) = ref_plan_msg.feet.feet.at(i);
  for (int j = 0; j < 3; ++j) {
    ref_underbrush_msg.joints.position.at(3 * i + j) = ref_plan_msg.joints.position.at(3 * i + j);
    ref_underbrush_msg.joints.velocity.at(3 * i + j) = ref_plan_msg.joints.velocity.at(3 * i + j);
  }
}
```

**Why a wholesale restore rather than four `if (force_mode_)` guards:** `quad_utils::ikRobotState()` runs on
the *entire* message, so even a leg whose foot reference was never touched had its joint angles recomputed
from its foot pose. Angles→position→angles is not perfectly reversible in floating point, and that round-trip
alone left untouched legs **~0.0016 rad** off the plan. Per-loop guards would not have caught it. This is why
the residual reached *exactly* zero rather than merely small.

### Fix 3b — Restore the swing feedforward (lines 690, 814)

Normal swing hard-set `torque_ff = 0`, discarding the inverse-dynamics feedforward that ID applies. Combined
with the rewrite pinning the reference foot position to the *actual* foot position (making joint tracking
error ~0 by construction, so `swing_kp = 10` produced ~0.02 Nm), **the swing leg was effectively limp** —
coasting on momentum, driven only by `swing_kd = 1`.

Added the `swing_cart_fb` computation (Jacobian-projected cartesian PD, mirroring ID — it did not exist in
this file at all), and:

```cpp
// AFTER — normal swing branch
.torque_ff = tau_array(joint_idx) + swing_cart_fb(joint_idx);
```

`swing_cart_fb` evaluates to exactly zero today (`swing_kp_cart`/`swing_kd_cart` are `[0,0,0]` in
`quad_utils/config/go2.yaml`) but is no longer structurally absent.

### Also done

- **Reverted** debug prints a prior session had added to `inverse_dynamics_controller.cpp`. That file is now
  byte-identical to `HEAD`. **Do not re-add prints there.**
- Added ~10 `[ub-diff]` instrumentation prints (see §4).
- **Did NOT touch** `robot_driver/config/robot_driver.yaml`, `body_force_estimator/config/body_force_estimator.yaml`,
  or `quad_utils/launch/robot_driver.py` — the latter two were already modified before this session.

---

## 3. Results

| metric | baseline | after Fix 1 | after Fixes 2+3 |
|---|---|---|---|
| stale early-swing targets | 7/11 | 0/5 | **0/32** |
| target jumps > 0.05 m | 3 (max +0.106) | 0 | **0** |
| swing `max\|dpos\|` | 1.05 rad | 0.375 rad | **0.0000** |
| swing `max\|dvel\|` | 11.8 rad/s | 7.64 rad/s | **0.0000** |
| stance `max\|dpos\|` | 0.0103 rad | 0.0016 rad | **0.0000** |
| torque saturations | 160 | 0 | **0** |

UB now produces **bit-identical** commands to ID when retract cannot fire. User spec satisfied.

Reference logs (may be rotated away):
`robot_driver_node_64883_1788314532426.log` (baseline) ·
`robot_driver_node_70964_1788315759116.log` (after Fix 1) ·
`robot_driver_node_79404_1788317144008.log` (final)

---

## 4. Instrumentation — how to verify

Ten tick-gated print sites tagged `[ub-diff][0:guard]` … `[9:cmd-retract]`, one at each point where UB
diverges from ID. Each reports **ID value | UB value | delta**, comparing against `ref_plan_msg` — a snapshot
of the untouched interpolated plan reference taken before UB overwrites `ref_state_msg_`.

- **Gate:** `UBDBG_EVERY_N = 100` at line 104 (~5 Hz at 500 Hz control rate). Lower to ~10–25 for denser
  coverage of early swing. A shared tick counter is used deliberately instead of `RCLCPP_*_THROTTLE`, which
  keeps a separate timer per call site and would let different legs through on different ticks.
- **Output goes to `~/.ros/log/robot_driver_node_<pid>_<ts>.log`** — NOT `~/.ros/log/latest/launch.log`, which
  holds only launch-level messages. No piping needed.
- **Checker:** `./check_ubdiff.py [logfile]` (repo root; defaults to `launch.log`, so pass the node log
  explicitly). Prints pass/fail for [A] early-swing footfall target, [B] mid-swing target discontinuity,
  [C] residual UB-vs-ID divergence (swing vs stance as a built-in control group), [D] retract path / guard /
  saturation counts.

---

## 5. Config reference

`robot_driver/config/robot_driver.yaml` → `underbrush_swing` (**uncommitted local edits**):

| param | current | committed | note |
|---|---|---|---|
| `tau_contact_start` | **1000.0** | 2.0 | retract can NEVER fire; measured torques peak ~2.4 Nm |
| `tau_contact_end` | **2.0** | 5.0 | |
| `hip_retract_sign` | **1.0** | -1.0 | Go2 hip axis; untested at runtime |
| `retract_vel` | 20.0 | | rad/s |
| `tau_push` | 2.0 | | Nm |
| `min_switch` / `t_down` / `t_up` | 0.1 / 0.135 / 0.04 | | s |

`quad_utils/config/go2.yaml`: `stance_kp [60,60,60]`, `stance_kd [4,4,4]`, `swing_kp [10,10,10]`,
`swing_kd [1,1,1]`, `swing_kp_cart [0,0,0]`, `swing_kd_cart [0,0,0]`, gait `period: 0.54`, `duty_cycles 0.5`
→ swing ≈ 0.27 s. Torque limits: 33.5 Nm (motors 0,1), 50.0 Nm (motor 2).

`local_planner/config/local_planner.yaml`: `update_rate 333.0`, `timestep 0.03`, `horizon_length 26`
→ 0.78 s horizon over a 0.54 s gait.

---

## 6. CRITICAL CAVEAT

**At `tau_contact_start: 1000`, nothing exercises the underbrush swing law at all.** Fixes 2+3 mean
non-retracting legs fall through to plain ID, and retract can never trigger. The controller is currently
**provably ID with extra steps** — the correct bisection baseline, but *not* Underbrush.

`[7:switch]` and `[9:cmd-retract]` have **never fired in any run to date**. `hip_retract_sign`, `retract_vel_`
and `tau_push_` remain completely unexercised at runtime. Restoring `tau_contact_start: 2.0` /
`tau_contact_end: 5.0` re-enables Underbrush, and those two prints will immediately show whether retract fires
and in which direction the hip sweeps.

Note also: Fixes 2+3 make non-retracting legs ignore the underbrush swing law entirely, which is a **departure
from the published controller**. Correct for the ID bisection; whether it ships that way is the user's call.

---

## 7. Open task — ROS 1 bag analysis

### The question

Fix 1's bug is **not** a ROS 2 migration artifact. It is present verbatim in the original ROS 1 source:

| commit | date | branch | note |
|---|---|---|---|
| `ba12253d` | 2022-12-12 | `devel` | "Adding underbrush controller" — bug present |
| `2bc8cfd4` | 2023-10-13 | `devel` | "Addition of Underbrush Walking (#396)" |
| `cd1f5d72` | 2025-06-11 | `devel_ros2` | ported to ROS 2, bug carried faithfully (`.toSec()` → `.seconds()`) |

ROS 1 original for reference:

```cpp
double t_now = (ros::Time::now() - last_local_plan_msg_->state_timestamp).toSec();  // plan-relative
...
if (t_now < last_local_plan_msg_->states[j].header.stamp.toSec() &&   // absolute -> same bug
    bool(last_local_plan_msg_->states[j].feet.feet.at(i).contact)) {
```

**So why did the IROS work succeed (14/16 lab trials) with this bug present?**

### Working hypothesis (UNPROVEN — this is what the bag is for)

The RETRACT branch sets `kp = 0` on hip and knee and drives them open-loop, so those two joints **do not read
the swing reference at all**. Only abad still position-tracks. A snagged leg is therefore structurally immune
to a corrupted swing target on 2 of its 3 joints.

In the original work the robot walked in real entanglements with `tau_contact_start: 2.0`, so legs entered
retract often — plausibly masking the bug on exactly the joints that dominate foot placement. The current
config is the opposite extreme: retract never fires, on clear ground, so 100% of swings run the corrupted
reference at full position gain with nothing hiding it.

Secondary factor: the bug's blast radius equals `t_now` (plan age = sensing + MPC solve latency), since that
is how far into the past the plan buffer's leading states sit. Observed corruption window here was ~30–100 ms
of a 270 ms swing (11–37%). Anything raising solve latency or shortening swing widens it. **This was inferred,
not measured** — `t_now` is not currently printed.

### What a ROS 1 bag would settle

1. **Did the stale-target bug fire back then?** `RobotPlan` carries `state_timestamp` plus the full `states[]`
   array with per-foot `contact` flags — everything the buggy search consumed. Replay both versions offline
   against the recorded plans (walk `states[j]` from 0 taking first contact = the bug; again with the
   `- t_first_state` filter = the fix) and diff the targets returned.
2. **How wide was the corrupted window?** `now − state_timestamp` per message gives plan age directly,
   converting the inferred 30–100 ms into a measurement.
3. **Was retract masking it?** `force_mode_` was never published — **but it is recoverable.** RETRACT sets
   `kp = 0` on hip and knee while NORMAL sets `swing_kp_`. So in `LegCommandArray`, a swing leg with
   `motor_commands[1].kp == 0 && motor_commands[2].kp == 0` **is** a leg in retract. Reconstruct the mode
   timeline per leg and measure what fraction of swing time the original trials spent retracting. This turns
   the masking hypothesis into a number.

### Topics needed (priority order)

- `local_plan` — **essential**; without it none of the above is possible
- `control/joint_command` (`LegCommandArray`) — for the `force_mode_` reconstruction
- `state/ground_truth` (preferred) or `state/estimate`
- `body_force/joint_torques` (`BodyForceEstimate`) — bonus, torques vs thresholds

Filter before copying if size is a concern:

```bash
rosbag filter in.bag out.bag \
  "topic in ['/robot_1/local_plan','/robot_1/control/joint_command','/robot_1/state/ground_truth']"
```

### Blockers / gotchas

- **The bag is not on this machine yet.** Work happens on `unitree-jetson-payload` (`HOME=/root`); the user's
  Downloads/home is their laptop. Needs `scp <bag> root@<jetson>:/root/`. Verified: no `.bag` anywhere on this
  filesystem, no `Downloads` dir, nothing >5 MB written recently.
- **ROS 2 tooling cannot read ROS 1 `.bag`.** The `rosbags` Python package can (pure Python, ROS 1 reader,
  custom types registerable from the `.msg` files in `quad_msgs/msg/`). **Whether it is installed here was
  never confirmed** — that check got interrupted. Verify early; if missing and the Jetson is offline, that is
  a hard blocker.
- **Different robot.** Original work is Spirit 40; this is Go2. Absolute numbers will not transfer. The
  transferable questions are mechanism-level: did the search return stale targets, how wide was the window,
  how much of the time was retract masking it.
- Per `docs/tutorials/specialized-controllers.md`, ROS 2 Underbrush is documented as **Spirit 40 only**, with
  porting to other platforms listed as open work.

---

## 8. Suggested next steps

1. **Get the ROS 1 bag onto the Jetson** and confirm `rosbags` availability. Run the three analyses in §7.
2. **Optionally add a `t_now` print** to the `[1:td-scan]` block to measure plan age directly on the Go2, so
   the corrupted-window width is observed rather than inferred.
3. **Restore `tau_contact_start: 2.0` / `tau_contact_end: 5.0`** and re-run. `[7:switch]` and
   `[9:cmd-retract]` will fire for the first time; check hip sweep direction against `hip_retract_sign`.
4. **Commit as two separate changes.** The time-base fix (Fix 1) stands alone as a genuine bug fix valid in
   every configuration and is worth upstreaming. Fixes 2+3 are a design change coupled to the ID-equivalence
   spec and the yaml — keep them separate.
5. **Strip or gate the `[ub-diff]` instrumentation** before merging; it is verbose and debug-only.

---

## 9. User context

The user is **D. Ologan** (`ologandavid`), a **co-author of the Underbrush paper** this controller implements.
They are the spec owner. Do not infer design intent from code structure and assert it as fact — ask. (I got
this wrong once, claiming the unconditional swing rewrite was deliberate "dragging" behavior by design; it was
inference presented as intent, and it was not correct.)
