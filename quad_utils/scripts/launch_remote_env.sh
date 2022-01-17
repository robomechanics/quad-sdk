#!/bin/bash

echo "Make sure you sourced this file (not ./)"
echo "Sourcing environment and setting IPs"
source ~/catkin_ws/src/quad-software/quad_utils/scripts/init_remote.sh
   
echo "Launch remote driver? [y/n]"
read input

if [[ $input == "Y" || $input == "y" ]]; then
	roslaunch quad_utils remote_driver.launch
fi