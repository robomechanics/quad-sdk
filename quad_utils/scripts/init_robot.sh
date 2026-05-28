#!/bin/bash
echo "Sourcing ros2_ws/install/setup.bash"
source /root/ros2_ws/install/setup.bash

DDS_BUF=134217728  # 128 MiB, matches <SocketReceiveBufferSize min="128MB"/>
if [[ $(cat /proc/sys/net/core/rmem_max 2>/dev/null) -lt $DDS_BUF ]]; then
    echo "Raising net.core.rmem_max / wmem_max to 128MB for CycloneDDS"
    sysctl -w net.core.rmem_max=$DDS_BUF >/dev/null \
        || echo "WARNING: failed to set net.core.rmem_max (need root?)"
    sysctl -w net.core.wmem_max=$DDS_BUF >/dev/null \
        || echo "WARNING: failed to set net.core.wmem_max (need root?)"
fi

# Find the robot's own IP and interface name on the Unitree MCU subnet
# (192.168.123.0/24). The MCU is at 192.168.123.161; we just need an address on
# the same subnet. ROBOT_MCU_IP feeds the Cyclone XML (which wants an IP),
# ROBOT_MCU_IFACE feeds the Unitree SDK ChannelFactory (which wants a name).
read -r ROBOT_MCU_IFACE ROBOT_MCU_IP < <(ip -4 -o addr show \
    | awk '$4 ~ /^192\.168\.123\./ {split($4, a, "/"); print $2, a[1]; exit}')
if [[ -z "$ROBOT_MCU_IP" ]]; then
    echo "WARNING: No interface has an IP on 192.168.123.0/24 (MCU network)."
    echo "         The Unitree MCU will be unreachable. Check that the built-in"
    echo "         ethernet is up and has an address on that subnet."
else
    echo "Robot MCU-side: $ROBOT_MCU_IFACE @ $ROBOT_MCU_IP"
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
export ROBOT_MCU_IFACE
export ROBOT_ROS_IP

echo "Setting ROS_DOMAIN_ID to 42 and RMW_IMPLEMENTATION to rmw_cyclonedds_cpp"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///root/ros2_ws/src/quad-sdk/quad_utils/scripts/cyclone_dds_robot.xml
