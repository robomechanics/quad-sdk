#!/bin/bash

echo "Make sure you sourced this file (not ./)"
echo "Sourcing environment and setting IPs"
<<<<<<< HEAD
source ~/catkin_ws/src/quad-sdk/quad_utils/scripts/init_remote.sh
=======
<<<<<<< HEAD
source ~/catkin_ws/src/quad-sdk/quad_utils/scripts/init_remote.sh
=======
source ~/catkin_ws/src/quad-software/quad_utils/scripts/init_remote.sh
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
   
echo "Launch remote driver? [y/n]"
read input

if [[ $input == "Y" || $input == "y" ]]; then
	roslaunch quad_utils remote_driver.launch
fi