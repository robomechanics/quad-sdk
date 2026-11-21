# Frozen-policy contact-free simulation — 2026-11-17

The frozen 75-input selected actor can execute the full choreography without
measured contact feedback in these MuJoCo trials. Flat-floor forward travel has
rare support/landing misses; this is a promising experiment, not hardware
qualification or evidence of general terrain robustness. No training or hardware
commands were performed. Existing controller defaults and exports are unchanged.

## Controller change

`scripts/cpg_contact_free_mujoco.py` is a separate simulation-only entry point.
It replaces incoming foot forces BEFORE estimation/control with scheduled support
weights (20 N for assumed stance, zero for assumed swing). Those values are
software flags, not measured or estimated physical forces. The existing
support-foot estimator, event filter and support velocity damping therefore use
planned support. Encoders and IMU remain live feedback. STEP and KICK transition
at their reference endpoint; HOLD continues to the next scheduled STEP. Admission
uses assumed support. Startup still checks encoder/IMU stability and physical
age, but assumes feet support. Tilt/encoder/sensor-timing faults remain active.
No measured contact, simulator root state or true foot pose feeds control.
This does not remove contact sensing from the production/hardware runtime.

A 400-frame replay with identical joint/IMU inputs and zero versus random
0–1000 N forces produced exactly identical observations and targets. Meaningful
checks also verified that the physical landing audit rejects missing contacts
and missing swing clearance even when the controller success flag remains set.

## Matched results

Each trial lasts 90 simulated seconds. Flat cases: neutral, +0.10 m/s forward,
+0.05 m/s with +/-0.10 rad/s yaw, each from both starting diagonals. Mild-slope
cases: floor pitch +/-0.01 rad (0.57 degrees), neutral and +0.10 m/s forward,
FL initialization. All 24 trials completed, zero falls in either mode.

| Cases | Measured-contact steps | Contact-free steps | Measured-contact kick chains | Contact-free kick chains |
|---|---:|---:|---:|---:|
| Flat neutral, 2 trials | 200/200 | 200/200 | 66/66 | 66/66 |
| Flat forward, 2 trials | 200/200 | 198/200 | 66/66 | 64/66 |
| Flat combined turns, 4 trials | 400/400 | 400/400 | 132/132 | 132/132 |
| Mild slopes, 4 trials | 395/400 | 397/400 | 129/132 | 131/132 |

Flat contact-free forward misses each include an opposite-support loss in the
post-kick STEP window: one at controller time 16.0 s (FL initialization), one at
77.333 s (FR initialization). Both runs continued. Flat neutral net displacement
was about -0.278/-0.279 m versus baseline -0.294/-0.293 m. Forward FL travel-only
speed was 0.0840 m/s versus baseline 0.0786 m/s at a 0.10 m/s request; this excludes
kick/hold pauses. Whole-trial forward displacement was 5.418 m versus 5.089 m,
a separate measure including pauses. Slope differences are single deterministic
trials per case; do not infer superiority from them.

## Scoring and reproducibility

Independent simulator contacts, foot poses and velocities score the motion.
STEP audit v2 retains +/-60 ms touchdown and checks support through +100 ms.
The existing kick audit's postkick_success flag depends on the controller's
contact assumptions. `scripts/audit_cpg_contact_free.py` replaces that landing
credit with independent physical swing, clearance, touchdown and support checks
for BOTH arms. Original audit criteria and defaults are preserved. On slopes,
sole clearance is measured against the actual plane normal; raw world-Z sole
values are retained as world_z_sole in traces. Endpoint XY and musical tolerances
are unchanged. Controller success counters in contact-free result.json are
synthetic diagnostics and must not be used as qualification scores.

Actor: exports/go2_cpg_hardware_prep_20261102/assets/selection.json;
SHA256 bfe0617876fd09abc972161193004974975f402927119023e707ac0037df3051.
Same 18 Nm sampled command cap and speed envelope in both arms.
MuJoCo 4.0.1 on CPU; current official Unitree go2.xml downloaded into the session,
SHA256 2014a3d76e30f17ab9447d8a67bd015291f74fa4d71ae30d005f1a32bd693d4b.
Robot dynamics are unmodified; visual mesh assets resolve to the local Quad-SDK
asset directory. Flat floor and fixed 2 ms physics timestep match the existing
simulation adapter. This engine/model provenance is separate from historical
qualification; the measured-contact baseline was rerun in the same environment.

Evidence: logs/cpg_contact_free_20261117/summary.json; per-trial traces and audits;
force_invariance.json; physical_audit_checks.json. run.py and
run_tilt_baseline.py record the exact trial matrix; normalize_slope_audits.py
and final_audits.py record postprocessing. The environment used is
/home/rml4/venvs/go2-cpg/bin/python. One setup smoke failed on a missing local
URDF; the bundle's unchanged URDF was copied into the expected ignored .local
path, then smoke2 passed. No experiment remains running.

Reproduce an individual contact-free trial:

```sh
/home/rml4/venvs/go2-cpg/bin/python scripts/cpg_contact_free_mujoco.py \
  --model logs/cpg_contact_free_20261117/model/go2.xml \
  --selection exports/go2_cpg_hardware_prep_20261102/assets/selection.json \
  --output logs/contact_free_new_trial --seconds 90 \
  --input fixed --velocity .1 0 0 --first-front FL
```

Add --baseline for measured-contact control, --floor-tilt .01 for a mild pitched
floor. Run the independent audits before interpreting motion quality.

Next useful validation is disturbances, friction/actuator variation, and Gazebo
transfer under scheduled support. This test does not establish slip detection,
obstacle handling, support after unexpected landing, or physical recovery.
