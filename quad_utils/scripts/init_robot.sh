#!/bin/bash
echo "Sourcing ros2_ws/install/setup.bash"
source /root/ros2_ws/install/setup.bash

# Assign static IP to eth0 for MCU communication (MCU is at 192.168.123.161)
if ! ip addr show eth0 | grep -q "192.168.123.222"; then
    echo "Adding MCU subnet (192.168.123.222/24) to eth0"
    sudo ip addr add 192.168.123.222/24 dev eth0
fi

echo "Setting ROS_DOMAIN_ID to 42 and RMW_IMPLEMENTATION to rmw_cyclonedds_cpp"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///root/ros2_ws/src/quad-sdk/quad_utils/scripts/cyclone_dds_robot.xml