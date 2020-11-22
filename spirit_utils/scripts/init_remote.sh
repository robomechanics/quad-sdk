#!/bin/bash
echo "Sourcing ~/catkin_ws/devel/setup.bash"
source ~/catkin_ws/devel/setup.bash
echo "Setting ROS_MASTER_URI to 192.168.168.105 and ROS_IP to 192.168.168.5"
export ROS_MASTER_URI=http://192.168.168.105:11311
export ROS_IP=192.168.168.5
