#!/bin/bash
echo "Sourcing catkin_ws/devel/setup.bash"
source /home/ghost/catkin_ws/devel/setup.bash
echo "Setting ROS_MASTER_URI and ROS_IP to 192.168.168.105"
export ROS_MASTER_URI=http://192.168.168.105:11311
export ROS_IP=192.168.168.105
# MOCAP="false"
# PROXY="false"
# while getopts "mp" flag
# do
#   case $flag in
#     m) MOCAP="true"; echo "using mocap" ;;
#     p) PROXY="true"; echo "using proxy robot" ;;
#   esac
# done
# echo "Launching robot_driver"
# roslaunch spirit_utils robot_driver.launch mocap:=$MOCAP proxy:=$PROXY