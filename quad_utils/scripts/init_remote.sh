#!/bin/bash
echo "Sourcing ~/ros2_ws/install/setup.bash"
source ~/ros2_ws/install/setup.bash

echo "Connected to robot with wifi or ethernet? (w/e)"
read input
if [[ $input == "E" || $input == "e" ]]; then
    echo "Setting ROS_DOMAIN_ID to 42 and RMW_IMPLEMENTATION to rmw_cyclonedds_cpp"
	export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file:///home/rml_admin/ros2_ws/src/quad-sdk/quad_utils/scripts/cyclone_dds_remote.xml
else
    echo "Setting ROS_DOMAIN_ID to 42 and RMW_IMPLEMENTATION to rmw_cyclonedds_cpp"
	export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file:///home/rml_admin/ros2_ws/src/quad-sdk/quad_utils/scripts/cyclone_dds_remote.xml
fi