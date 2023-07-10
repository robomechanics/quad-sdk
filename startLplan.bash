#!/bin/bash         


echo "set up Sims"

cd ~/catkin_ws
source devel/setup.bash

echo "done!"

rostopic pub /robot_1/control/mode std_msgs/UInt8 "data: 1"
roslaunch quad_utils quad_plan.launch reference:=twist twist_input:=keyboard