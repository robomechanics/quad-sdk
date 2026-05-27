#!/bin/bash

echo "Make sure you sourced this file (not ./)"
echo "Sourcing environment and setting IPs"
source ~/ros2_ws/src/quad-sdk/quad_utils/scripts/init_remote.sh
   
echo "Launch remote driver? [y/n]"
read input

if [[ $input == "Y" || $input == "y" ]]; then
	ros2 launch quad_utils remote_driver.py
fi