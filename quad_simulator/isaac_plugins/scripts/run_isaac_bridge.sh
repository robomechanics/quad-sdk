#!/usr/bin/env bash
#
# Launch isaac_bridge.py with Isaac Sim 5.1's bundled ROS 2 Jazzy.
#
# Must be run from a shell where /opt/ros/jazzy/setup.bash has NOT been
# sourced -- system ROS 2 (Python 3.12) and Isaac's bundle (Python 3.11)
# conflict on LD_LIBRARY_PATH and DDS symbol resolution.
#
# Usage:
#   conda activate isaaclab
#   bash run_isaac_bridge.sh [bridge args...]
#
# Examples:
#   run_isaac_bridge.sh                          # spirit, flat
#   run_isaac_bridge.sh --robot go2              # go2, flat
#   run_isaac_bridge.sh --scene underbrush       # vines
#   run_isaac_bridge.sh --terrain step_20cm      # STL terrain
#   run_isaac_bridge.sh --physx-gpu              # GPU dynamics
#
# All flags after the script name are forwarded to isaac_bridge.py.

set -euo pipefail

readonly WS=/home/rml/ros2_ws
readonly CONDA_ENV=/home/rml/anaconda3/envs/isaaclab
readonly BRIDGE="${WS}/src/quad-sdk/quad_simulator/isaac_plugins/scripts/isaac_bridge.py"
readonly ISAACLAB=/home/rml/IsaacLab/isaaclab.sh
readonly QUAD_MSGS_PY="${WS}/install/quad_msgs/lib/python3.12/site-packages"
readonly QUAD_MSGS_LIB="${WS}/install/quad_msgs/lib"
readonly BUNDLED_ROS2_LIB="${CONDA_ENV}/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib"

require_path() {
  local kind=$1 path=$2
  if [[ "$kind" == dir && ! -d "$path" ]] \
      || [[ "$kind" == exec && ! -x "$path" ]]; then
    echo "run_isaac_bridge: missing $kind at $path" >&2
    exit 1
  fi
}

require_path dir  "$QUAD_MSGS_PY"
require_path dir  "$BUNDLED_ROS2_LIB"
require_path exec "$ISAACLAB"

# Drop /opt/ros/* entries from LD_LIBRARY_PATH; keep everything else.
strip_ros_entries() {
  local IFS=':'
  local out=()
  for entry in $1; do
    [[ "$entry" != *"/opt/ros/"* ]] && out+=("$entry")
  done
  (IFS=':'; echo "${out[*]}")
}

# Isaac's bundled ROS 2 libs must lead LD_LIBRARY_PATH before launch --
# glibc caches it at process start for in-process dlopens.
stripped_ld="$(strip_ros_entries "${LD_LIBRARY_PATH:-}")"
export LD_LIBRARY_PATH="${BUNDLED_ROS2_LIB}:${QUAD_MSGS_LIB}${stripped_ld:+:${stripped_ld}}"
export PYTHONPATH="${QUAD_MSGS_PY}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

unset AMENT_PREFIX_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
unset CMAKE_PREFIX_PATH COLCON_PREFIX_PATH

exec "$ISAACLAB" -p "$BRIDGE" "$@"
