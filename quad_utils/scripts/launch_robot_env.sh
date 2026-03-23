#!/bin/bash

echo "Syncing clock with remote computer"
sudo iw dev wlan0 set power_save off
sudo chronyc -a makestep

echo "Syncing clock with remote computer"
sudo systemctl restart chrony
sleep 2
sudo chronyc -a makestep

echo "Checking chrony status"
chronyc tracking || true
chronyc sources || true


echo "Sourcing quad_utils/scripts/init_robot.sh to source env and setup IPs"
source ~/ros2_ws/src/quad-sdk/quad_utils/scripts/init_robot.sh

echo "Launch robot_driver.launch with inverse dynamics controller? (y/n)"
read input
if [[ $input == "Y" || $input == "y" ]]; then
	ros2 launch quad_utils robot_driver.launch mocap:=true hardware:=true
fi
