#!/bin/bash
echo "Sourcing ~/catkin_ws/devel/setup.bash"
source ~/catkin_ws/devel/setup.bash

echo "Connected to robot with wifi or ethernet? (w/e)"
read input
if [[ $input == "E" || $input == "e" ]]; then
    echo "Setting ROS_MASTER_URI to 192.168.8.101 and ROS_IP to 192.168.8.102"
	export ROS_MASTER_URI=http://192.168.8.101:11311
    export ROS_IP=192.168.8.102
else
    echo "Setting ROS_MASTER_URI to 192.168.8.101 and ROS_IP to 192.168.8.103"
	export ROS_MASTER_URI=http://192.168.8.101:11311
    export ROS_IP=192.168.8.103
fi

