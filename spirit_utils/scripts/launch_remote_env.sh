#!/bin/bash

echo "Navigaing to spirit-software"
cd ~/catkin_ws/src/spirit-software/

echo "Pull and build most recent code (requires internet connection)? [y/n]"
read input

if [[ $input == "Y" || $input == "y" ]]; then
	echo "Pulling most recent code from main"
	git checkout main
	git pull

	echo "Running setup script to make sure package deps are up to date"
	./setup.sh

	echo "Compiling"
	cd ~/catkin_ws/
	catkin_make
	cd ~/catkin_ws/src/spirit-software/
fi

echo "Make sure you sourced this file (not ./)"
echo "Sourcing environment and setting IPs"
source ~/catkin_ws/src/spirit-software/spirit_utils/scripts/init_remote.sh
   
echo "Launch remote driver? [y/n]"
read input

if [[ $input == "Y" || $input == "y" ]]; then
	roslaunch spirit_utils remote_driver.launch
fi