#!/bin/bash

echo "Syncing clock with remote computer"
sudo iw dev wlan0 set power_save off
sudo chronyc -a makestep

echo "Restarting ghost service to use correct IP"
sudo service ghost restart

echo "Sourcing quad_utils/scripts/init_robot.sh to source env and setup IPs"
source ~/catkin_ws/src/quad-sdk/quad_utils/scripts/init_robot.sh

echo "Launch robot_driver.launch with inverse dynamics controller? (y/n)"
read input
if [[ $input == "Y" || $input == "y" ]]; then
	roslaunch quad_utils robot_driver.launch mocap:=true
fi
