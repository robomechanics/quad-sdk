#!/bin/bash
echo "Sourcing ~/ros2_ws/install/setup.bash"
source ~/ros2_ws/install/setup.bash

# Destination for ros2 bag logs (used by logging.py via quad_plan logging:=true)
export QUAD_LOGGER_SRC=/home/rml_admin/ros2_ws/src/quad-sdk/quad_logger
mkdir -p "$QUAD_LOGGER_SRC/bags/archive"

echo "Which robot? (t)heodore / (a)lvin / (s)imon"
read robot
echo "Connected to robot with wifi or ethernet? (w/e)"
read conn

case "${robot,,}" in
    t|theodore) eth_ip=192.168.8.18; wifi_ip=192.168.8.51 ;;
    a|alvin)    eth_ip=192.168.8.19; wifi_ip=192.168.8.50 ;;
    s|simon)    eth_ip=192.168.8.20; wifi_ip=192.168.8.52 ;;
    *) echo "Unknown robot: $robot"; return 1 2>/dev/null || exit 1 ;;
esac

case "${conn,,}" in
    w|wifi)     export REMOTE_ROBOT_IP=$wifi_ip ;;
    e|ethernet) export REMOTE_ROBOT_IP=$eth_ip ;;
    *) echo "Unknown connection: $conn"; return 1 2>/dev/null || exit 1 ;;
esac

echo "Robot peer IP: $REMOTE_ROBOT_IP"
echo "Setting ROS_DOMAIN_ID to 42 and RMW_IMPLEMENTATION to rmw_cyclonedds_cpp"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/rml_admin/ros2_ws/src/quad-sdk/quad_utils/scripts/cyclone_dds_remote.xml
