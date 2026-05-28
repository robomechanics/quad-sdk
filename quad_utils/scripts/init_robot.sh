#!/bin/bash
echo "Sourcing ros2_ws/install/setup.bash"
source /root/ros2_ws/install/setup.bash

# Find the robot's own IP on the Unitree MCU subnet (192.168.123.0/24).
# The MCU is at 192.168.123.161; we just need an address on the same subnet.
ROBOT_MCU_IP=$(ip -4 -o addr show | awk '$4 ~ /^192\.168\.123\./ {split($4, a, "/"); print a[1]; exit}')
if [[ -z "$ROBOT_MCU_IP" ]]; then
    echo "WARNING: No interface has an IP on 192.168.123.0/24 (MCU network)."
    echo "         The Unitree MCU will be unreachable. Check that the built-in"
    echo "         ethernet is up and has an address on that subnet."
else
    echo "Robot MCU-side IP detected: $ROBOT_MCU_IP"
fi

# Find the robot's own IP on the ROS2 comms subnet (192.168.8.0/24).
# The remote computer lives at 192.168.8.103.
ROBOT_ROS_IP=$(ip -4 -o addr show | awk '$4 ~ /^192\.168\.8\./ {split($4, a, "/"); print a[1]; exit}')
if [[ -z "$ROBOT_ROS_IP" ]]; then
    echo "WARNING: No interface has an IP on 192.168.8.0/24 (ROS2 comms network)."
    echo "         The remote computer at 192.168.8.103 will be unreachable."
    echo "         Check that the USB-Ethernet dongle is plugged in."
else
    echo "Robot ROS2-side IP detected: $ROBOT_ROS_IP"
fi

export ROBOT_MCU_IP
export ROBOT_ROS_IP

echo "Setting ROS_DOMAIN_ID to 42 and RMW_IMPLEMENTATION to rmw_cyclonedds_cpp"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///root/ros2_ws/src/quad-sdk/quad_utils/scripts/cyclone_dds_robot.xml