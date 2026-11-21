#!/usr/bin/env bash
# Build outside Conda so ROS cannot pick up incompatible Python/curl libraries.
set -euo pipefail
conga_repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
conga_sdk=${UNITREE_SDK2_DIR:-"$conga_repo/external/unitree_sdk2"}
if [[ "$conga_repo" == /opt/quad-sdk ]]; then
  conga_default_profile=conga # Standalone image's fixed installation contract.
elif [[ -f /.dockerenv ]]; then
  conga_default_profile=conga-container
else
  conga_default_profile=conga-host
fi
conga_profile=${CONGA_BUILD_PROFILE:-$conga_default_profile}
conga_jobs=${CONGA_BUILD_JOBS:-4}
if [[ ! "$conga_profile" =~ ^[a-zA-Z0-9_-]+$ ]]; then
  echo 'CONGA_BUILD_PROFILE must be a single directory name (letters, digits, underscore, dash).' >&2
  exit 1
fi
if [[ ! "$conga_jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo 'CONGA_BUILD_JOBS must be a positive integer.' >&2
  exit 1
fi
echo "Building $conga_profile; afterward source install/$conga_profile/setup.bash"
if [[ ! -f "$conga_sdk/include/unitree/robot/channel/channel_publisher.hpp" ]]; then
  echo 'Set UNITREE_SDK2_DIR to a complete Unitree SDK2 checkout (or initialize external/unitree_sdk2).' >&2
  exit 1
fi
exec env -i HOME="$HOME" USER="${USER:-}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  MAKEFLAGS="-j$conga_jobs" CMAKE_BUILD_PARALLEL_LEVEL="$conga_jobs" bash --noprofile --norc -c '
    set -eo pipefail
    cd "$1"
    source /opt/ros/jazzy/setup.bash
    colcon --log-base "log/$3" build --executor sequential --build-base "build/$3" --install-base "install/$3" \
      --packages-up-to robot_driver quad_utils gazebo_plugins gazebo_scripts go2_description sensor_description objects_description \
      --cmake-args -DUNITREE_SDK2_DIR="$2" -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
  ' bash "$conga_repo" "$conga_sdk" "$conga_profile"
