#!/bin/bash
 
set -e

# Ros build
source "/opt/ros/jazzy/setup.bash"

echo "==============ROS2 Docker Env Ready================"

if [ -n "$VSCODE_REMOTE_CONTAINERS" ]; then
  exec "$@"
fi
cd "${WS_DIR:-$HOME/ros2_ws}"
exec "$@"
