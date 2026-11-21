# Conga port audit — 2026-11-17

Checkout: `/home/rml4/Documents/nick/quad-sdk`, base `b986dc4b`, including
existing uncommitted Conga integration. No existing port changes were discarded.
No hardware interface was launched and no training occurred.

## Fixes

- Initialized the pinned Unitree SDK2 and RBDL submodules, including RBDL's nested
  URDF parser. The documented native build initially failed because SDK2 was empty.
- Declared dependencies actually required by CMake but missing from package.xml:
  quad_utils: pinocchio, pluginlib, filters; robot_driver: tf2_eigen,
  geometry_msgs, test dependencies ament_cmake_gtest/ament_index_cpp;
  gazebo_scripts: ros_gz_interfaces. Fresh rosdep installs should not depend on
  unrelated packages supplying these transitively.
- Dedicated Conga Dockerfile now checks the nested URDF parser as well as SDK2
  and RBDL, with a specific initialization command if anything is absent.

## Verification

- Native build: all eight selected packages succeeded via scripts/build_conga.sh.
- rosdep check on the image's package subset passed after manifest changes.
- 600 frozen policy frames: exact observation/target parity and force invariance.
- Installed bundle also passed the same replay under isolated Python 3.12.3;
  pip check reported no broken requirements. Native driver shared libraries
  all resolve after sourcing the installed overlay.
- C++ CPG controller/sensor tests: 12 passed, zero failures/errors/skips.
- Docker Compose configuration parses successfully.
- Isolated packaged Gazebo launch, with no conga_root override: READY, neutral,
  forward, left and right combined turns; 50 seconds after READY, simulation
  clock 54.001 s, 2,449 frames, no controller faults. Minimum ready-state body
  height 0.28786 m. This verifies integration, not strict physical choreography.
- ROS_DOMAIN_ID=87, localhost discovery and a separate GZ_PARTITION isolated this
  trial from an existing user Gazebo session. The audit's process group was
  stopped; the pre-existing session was left running.

Evidence: log/conga-port-audit/{launch.log,trace.jsonl,result.json}; C++ results
under build/conga/robot_driver/test_results. Frozen actor/bundle unchanged.

## Docker handoff

First user-run image build reached robot_driver compilation, then failed because
the recursive `**/log` dockerignore rule removed Unitree SDK's
`include/unitree/common/log/log.hpp`. The full host log was available at
`/tmp/conga-docker-build.log`; the pasted tail contained warnings and the summary
but omitted this compiler error. Workspace output exclusions are now scoped to
root build/install/log directories, and the image checks the SDK logging header
before compiling. The corrected image still needs a user-run build.

Direct Docker daemon access is denied for this session user; passwordless sudo
is unavailable. No permissions were changed. Image build and image runtime are
therefore still unverified. Run on the host with your normal Docker permissions:

```bash
cd /home/rml4/Documents/nick/quad-sdk
set -o pipefail
docker build --progress=plain -f .devcontainer/Dockerfile.conga \
  -t quad-sdk:conga . 2>&1 | tee /tmp/conga-docker-build.log
```

The image build itself runs the frozen replay and C++ CPG tests. Send the first
failing build step and its error text (or /tmp/conga-docker-build.log). Once it
builds, the next check is a headless image launch in an isolated ROS domain.
No hardware readiness or ARM execution is claimed.

## Image build confirmed

The user reran the build after the dockerignore fix. The full host log confirms
eight packages built, zero error on the 600-frame frozen replay, and all 12 CPG
C++ tests passed. Image export completed as quad-sdk:conga, manifest list
sha256:b0726ac1d76a48c8ff4f086cc2df2409da9d5539b87e4565f8fc5e85ddcf20cb.

The remaining image runtime check is prepared as:

```bash
bash scripts/verify_conga_container.sh
```

This mounts the verification script into the already-built image; no rebuild is
needed. It launches packaged headless Gazebo, sends simulation READY and neutral,
forward and combined turn commands, checks 50 simulated seconds for controller
faults/body-height loss, and shuts down the test. It uses ROS domain 88 and a
separate Gazebo partition, writes the summary and trace into a new
/tmp/conga-container-check.* directory, and tees output to
/tmp/conga-container-run.log. It does not launch hardware. As with the native
check, this is integration verification, not an independent foot-contact audit.

## Container simulation confirmed

User-run verification passed. Evidence:
`/tmp/conga-container-check.KjqHNk/{result.json,launch.log,trace.jsonl}` and
`/tmp/conga-container-run.log`.

- 50 simulated seconds after READY; final simulation time 54.001 s.
- 2,449 policy frames, including 2,399 ready-state frames.
- Neutral, 0.10 m/s forward, and 0.05 m/s forward with +/-0.10 rad/s yaw exercised.
- Zero latched controller faults; minimum ready-state body height 0.28710 m.
- Worker round-trip median 2.63 ms, p99 3.36 ms, maximum 8.32 ms in this run.
- Test returned passed=true and stopped the simulation. SIGINT/exit-code -2
  messages and Python KeyboardInterrupt traces occur after the deliberate test
  shutdown, not during the motion window.

The standalone x86 CPU container build, bundled-policy tests and headless
simulation integration are now verified. Physical hardware, ARM execution and
strict independent contact/choreography qualification of this Gazebo port remain
outside this check.

## VS Code Dev Containers follow-up

The user's VS Code log selected `.devcontainer/x86/devcontainer.json`, which
builds the older CUDA development image. It failed cloning the inaccessible
`liyufan/Livox-SDK2` fork. Dockerfile.x86 now points to the reachable public
`Livox-SDK/Livox-SDK2` upstream. That entire CUDA image was not rebuilt in this
follow-up.

Added `.devcontainer/conga/devcontainer.json` using the verified `quad-sdk:conga`
image. It mounts the editable checkout at `/workspaces/quad-sdk`, retains the
image's built installation under `/opt/quad-sdk/install/conga`, and sources that
overlay in interactive terminals. No CUDA image build or Livox download is
needed for this configuration. It uses ROS domain 88 and GZ partition
conga_vscode. Editing source still requires rebuilding/sourcing the checkout's
own overlay when testing those edits.

The exact installed Dev Containers CLI (0.475.0 extension) successfully created
and initialized the Conga dev container, including postCreateCommand. Interactive
bash finds robot_driver in the image installation and the source mount is
readable. Log: `/tmp/conga-vscode-up.log`.

Docker access is available through the user's existing
`~/.local/bin/docker-with-group` wrapper, discovered in their VS Code log; no
account or socket permissions were changed.

The new VS Code dev container also passed the simulation smoke test: 12 seconds
after READY, 549 frames, no faults, minimum ready height 0.29127 m. Results are
in `/tmp/conga-vscode-check.log`. The dev container remains running as
`quad-sdk-conga-dev` for the user to attach; the test simulation was stopped.

At the user's request, the verified configuration is also the canonical
`.devcontainer/devcontainer.json`, so standard Reopen in Container discovers it
directly. Dev Containers CLI `up` with no explicit --config successfully chose
this root configuration and completed startup/postCreateCommand. Evidence:
`/tmp/conga-vscode-default-up.log`. Manual attachment is not required.

Final user preference: use the existing **Quad-SDK x86 (Sim + Learned Dev)**
configuration at `.devcontainer/x86/devcontainer.json`. Removed the two added
Conga Dev Containers configurations and their two temporary verification
containers. The standalone Conga image/dependency installer and controller port
are retained; the x86 Dockerfile continues to provision the policy environment.
The corrected public Livox clone URL remains in Dockerfile.x86. The earlier
root-default/attachment instructions above are superseded by this preference.

Subsequent x86 image build succeeded and VS Code Server started, but VS Code's
Wayland link setup failed because XDG_RUNTIME_DIR=/tmp/runtime-root did not
exist. Dockerfile.x86 now creates it with root ownership and mode 0700 before
container startup (postStartCommand is too late for that link setup). The
existing stopped x86 container a62c43156260 was started, repaired in place, and
the exact failed socket-link operation passed, including a Unix-socket check.
The user can retry Reopen in Container without rebuilding; future image rebuilds
include the directory fix.

## Host GUI and build isolation repair

Host launch had a live RViz window but saved geometry X=2560, plus the shared
install/conga tree had been overwritten by a build in the x86 container. Its
CMake caches referenced /root/ros2_ws and its binaries required host-unavailable
Pinocchio 4.1.0 and libonnxruntime.so.1. The driver and visualization helper failed
with loader exit 127.

Moved/resized RViz onto the primary display and changed the saved viewer geometry
to 1280x800 at (80,80). build_conga.sh now defaults to separate conga-host and
conga-container profiles, keeping the standalone /opt/quad-sdk image's conga
profile unchanged. CONGA_BUILD_PROFILE permits a deliberate override.

Fresh host build: all eight packages passed; required libraries resolve to host
Pinocchio 4.0.0 and no libraries are missing. All 12 CPG C++ tests passed. Stopped
the two broken old simulation launches and started one fresh host GUI simulation
in ROS domain 42 / GZ partition conga_host_gui. RViz window, driver, worker and
Gazebo controllers are running; startup log contains no loader errors, dead
processes, Python tracebacks or latched controller faults. Simulation remains
running for the user; READY was not requested by this repair.

Host launch setup is now source install/conga-host/setup.bash. The container
build uses source install/conga-container/setup.bash. See docs/cpg_gazebo.md.
Evidence: /tmp/conga-host-rebuild.log, /tmp/conga-host-ldd.log,
/tmp/conga-host-gui.log; tests in build/conga-host/robot_driver/test_results.
