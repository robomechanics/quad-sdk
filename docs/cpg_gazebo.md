# Contact-free Conga

Conga uses the frozen **75-input** actor from
`unitree_rl_lab/exports/go2_cpg_hardware_prep_20261102/assets/selection.json`
(SHA256 `bfe0617876fd09abc972161193004974975f402927119023e707ac0037df3051`).
It runs through the existing `robot_driver`, control modes, PD command messages,
and Gazebo/Unitree interfaces. The selectable controller is `conga_driver`
(`cpg` is an internal compatibility alias).

## Build

Install [ROS 2 Jazzy on Ubuntu 24.04](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) for a native build. Run the commands below from the Quad-SDK checkout root. Initialize the pinned submodules:

```bash
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
```

For a native environment that still needs the build and Gazebo dependencies,
install them before running the build:

```bash
sudo apt update
sudo apt install build-essential cmake python3-colcon-common-extensions python3-rosdep python3-venv \
  ros-jazzy-grid-map \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-topic-tools ros-jazzy-xacro ros-jazzy-robot-state-publisher
# If rosdep has never been initialized on this system, run sudo rosdep init first.
rosdep update --rosdistro jazzy
rosdep install -y --ignore-src --rosdistro jazzy --from-paths \
  quad_msgs quad_utils robot_driver quad_simulator/gazebo_plugins \
  quad_simulator/gazebo_scripts quad_simulator/go2_description \
  quad_simulator/other/sensor_description quad_simulator/other/objects_description
```

If RBDL with its URDF reader is missing, build and install the pinned submodule
with Ubuntu's system CMake:

```bash
/usr/bin/cmake -S external/rbdl-orb -B build/conga-rbdl \
  -DCMAKE_BUILD_TYPE=Release -DRBDL_BUILD_ADDON_URDFREADER=ON
/usr/bin/cmake --build build/conga-rbdl -j2
sudo /usr/bin/cmake --install build/conga-rbdl
sudo ldconfig
```

```bash
./scripts/build_conga.sh
source /opt/ros/jazzy/setup.bash
source install/conga-host/setup.bash
```

On a memory-constrained laptop, use `CONGA_BUILD_JOBS=2 ./scripts/build_conga.sh`.
The build script uses a clean system environment to avoid Conda Python and library
conflicts. The setup command above is for a native host build. In a Docker
development container, run the same build script and source
`install/conga-container/setup.bash` instead. The standalone image uses
`install/conga` internally. These separate directories keep host and container
artifacts isolated in a shared checkout. The script prints the setup file to
source, including when `CONGA_BUILD_PROFILE` overrides the directory name.
The installed robot_driver setup hook supplies the SDK's shared-library paths.

If CMake reports that the cache was created in a different directory or that the
source does not match the one used to generate the cache, the checkout contains
build artifacts from another path. From the checkout root, move the generated
directories for the affected profile aside and rerun the build. This example
resets the host profile. Use `conga-container` inside a development container
or the custom profile name if you set one.

```bash
profile=conga-host
stamp=$(date +%Y%m%d-%H%M%S)
if [ -d "build/$profile" ]; then mv "build/$profile" "build/$profile-relocated-$stamp"; fi
if [ -d "install/$profile" ]; then mv "install/$profile" "install/$profile-relocated-$stamp"; fi
CONGA_BUILD_PROFILE="$profile" ./scripts/build_conga.sh
```

After the rebuild, open a fresh shell and source ROS Jazzy and the setup file
printed by the build script. Use that same setup in terminals that send ROS
commands.

The policy worker uses a separate CPU Python environment with pinned dependencies.
For a native install (Docker provisions this automatically):

```bash
bash .devcontainer/conga/install.sh "$HOME/venvs/go2-cpg"
```

The runtime, actor, metadata, joint-limit URDF, license, checksums, and a small
reference replay are bundled in `robot_driver/conga` and installed into the
robot_driver package share directory. No training checkout, Isaac Sim, hidden
`.local` assets, or external model download is needed. The URDF is parsed for
joint limits only; Gazebo uses Quad-SDK's existing Go2 meshes.

## Fresh clone and Docker

Clone the branch that contains this guide and `robot_driver/conga/assets/policy.onnx`. Initialize recursive submodules before building. If you received a Git bundle, follow the transfer commands supplied with it and then start at the Docker build command.

```bash
git clone --recurse-submodules --branch nick-conga-transfer '<your-quad-sdk-repo-url>'
cd quad-sdk
docker build -f .devcontainer/Dockerfile.conga -t quad-sdk:conga .
docker run --rm --init --name conga-sim --network host --ipc host \
  -e ROS_DOMAIN_ID=42 quad-sdk:conga
```

This dedicated CPU image installs the ROS/Gazebo build dependencies, builds
RBDL and the required Quad-SDK packages, installs the pinned policy environment,
and runs the frozen replay and controller tests during image build. It starts
headless Gazebo by default. No GPU or separate `unitree_rl_lab` mount is needed.
The equivalent Compose service is `docker compose -f .devcontainer/docker-compose.yml up --build conga`.

Send the usual commands from another terminal, for example:

```bash
docker exec conga-sim /conga_entrypoint.sh ros2 topic pub --once \
  /robot_1/control/mode std_msgs/msg/UInt8 '{data: 1}'
docker exec conga-sim /conga_entrypoint.sh ros2 topic pub -r 20 \
  /robot_1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
```

The existing x86 and ARM learned Dockerfiles also provision `/opt/conga-venv`;
the MPC-only image is unchanged. For those development images, retain the
repository's normal ROS/system-dependency setup, then run `scripts/build_conga.sh`.
The dedicated image above builds only the Conga dependencies and needs no HSL.
`CONGA_POLICY_PYTHON` or launch argument `policy_python:=...` overrides the
interpreter. Launch defaults resolve the bundled runtime through the ROS package
index, so host usernames and checkout paths do not matter.

The x86 image was built and exercised headlessly. See the [port audit](conga_port_audit_20261117.md) for the recorded run and `bash scripts/verify_conga_container.sh` to repeat it on the destination laptop.

## Gazebo: launch, stand, send velocity

```bash
ros2 launch quad_utils quad_conga_gazebo.py gui:=false rviz:=false
```

This runs the simulator headless. For visible windows, run the following
from a desktop session that has access to its display:

```bash
ros2 launch quad_utils quad_conga_gazebo.py gui:=true rviz:=true
```

If Qt reports `Authorization required` or `could not connect to display`,
use the headless command or switch to an authorized desktop session. The
legacy launch name `quad_cpg_gazebo.py` runs the same contact-free controller.

Wait for the launch log to report that `joint_controller` is configured and
active. In another terminal with the same setup and `ROS_DOMAIN_ID`, request
standing/READY:

```bash
ros2 topic pub --once /robot_1/control/mode std_msgs/msg/UInt8 '{data: 1}'
```

After standing and startup settling, the frozen dance routine starts. Send the
usual velocity command:

```bash
ros2 topic pub -r 20 /robot_1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

The selected actor's envelope is forward `0–0.10 m/s`, lateral `0`, yaw
`±0.10 rad/s`. Neutral or stale velocity commands remove travel but **keep
dancing**, matching the frozen experiment. Travel pauses during kick/hold phases.

Status: `/robot_1/cpg/status`. Trace: `/tmp/cpg_gazebo_trace.jsonl` (override
with `trace:=...`). To finish the scheduled motion and hold a standing pose:

```bash
ros2 topic pub --once /robot_1/cpg/stand std_msgs/msg/Bool '{data: true}'
```

That stand request is terminal for the current runtime; relaunch to dance again.
The existing safety command is mode `4` (damping). Faults latch and require
relaunch; no automatic rearming occurs. Mode `0` requests the existing sit
transition; leaving READY stops the current dance runtime.

The Conga-only Gazebo actuator profile defaults to `go2hv_mujoco_v1`, matching
the selected deployment's prior transfer setup. Other controllers keep their
existing dynamics. No contact bridge or contact-state publisher is launched
for Conga. Simulator ground truth is used for normal driver bookkeeping and
trace evaluation, never as input to the policy/reference worker.

## Go2 hardware

Follow the [Go2 hardware runbook](cpg_hardware.md) for interface selection, preflight, supported startup, stop, and rollback. The dedicated hardware launch starts in SAFETY after releasing the Unitree motion service.

## Runtime contract and validation

The 50 Hz worker advances the scheduler, builds observations, evaluates ONNX,
and adds `0.12 * tanh(action)` to the CPG reference. The normal faster driver
loop tracks cached targets. Scheduled support flags are substituted **before**
the shared support-foot kinematic calculations and event processing. STEP/KICK
completion uses reference deadlines. This reproduces the report's contact-free
adapter; support flags are assumptions, not estimates of actual ground contact.

The native and x86 container runs each completed 50 simulated seconds after READY with neutral, forward, and combined turning commands. The recorded port audit gives the exact results.

Replay the reference check with:

```bash
"$HOME/venvs/go2-cpg/bin/python" scripts/test_conga_runtime.py
```

The replay uses the bundled first 600 frozen frames and verifies the bundle manifest.
