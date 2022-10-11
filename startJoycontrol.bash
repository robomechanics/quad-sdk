#!/bin/bash         


echo "set up Sims"

cd ~/catkin_ws2
source devel/setup.bash

echo "done!"

roslaunch teleop_twist_joy teleop.launch
