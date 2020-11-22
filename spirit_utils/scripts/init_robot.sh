#!/bin/bash
echo "Sourcing catkin_ws/devel/setup.bash"
source /home/ghost/catkin_ws/devel/setup.bash
echo "Setting ROS_MASTER_URI and ROS_IP to 192.168.168.105"
export ROS_MASTER_URI=http://192.168.168.105:11311
export ROS_IP=192.168.168.105
WITH_MOCAP="false"
WITH_PROXY="false"
while getopts "mi" flag
do
  case $flag in
    m) WITH_MOCAP="true"; echo "using mocap" ;;
    i) WITH_PROXY="true"; echo "using proxy robot" ;;
  esac
done
echo "Launching robot_driver"
roslaunch spirit_utils robot_driver.launch with_mocap:=$WITH_MOCAP with_proxy:=$WITH_PROXY