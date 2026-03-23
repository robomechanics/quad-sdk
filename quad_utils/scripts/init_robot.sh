#!/bin/bash
echo "Sourcing ros2_ws/install/setup.bash"
source /home/unitree/ros2_ws/install/setup.bash
echo "Setting ROS_DOMAIN_ID to 0 and RMW_IMPLEMENTATION to rmw_cyclonedds_cpp"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/patrick/cyclonedds.xml