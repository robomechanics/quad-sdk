#!/bin/bash

echo "Syncing clock with remote computer"
sudo iw dev wlan0 set power_save off

# Start chronyd if not already running (systemctl not available in container)
if ! chronyc tracking &>/dev/null; then
    echo "Starting chronyd..."
    sudo chronyd
    sleep 2
fi
sudo chronyc -a makestep

echo "Checking chrony status"
chronyc tracking || true
chronyc sources || true


echo "Sourcing quad_utils/scripts/init_robot.sh to source env and setup IPs"
source ~/ros2_ws/src/quad-sdk/quad_utils/scripts/init_robot.sh

echo "Launch robot_driver.py with inverse dynamics controller? (y/n)"
read input
if [[ $input == "Y" || $input == "y" ]]; then
	ros2 launch quad_utils robot_driver.py mocap:=true is_hardware:=true
fi