#!/bin/bash         


echo "set up Sims"

cd ~/catkin_ws_opt
source devel/setup.bash

echo "done!"

rostopic pub /robot_1/control/mode std_msgs/UInt8 "data: 1"
roslaunch quad_utils planning.launch global_planner:=twist logging:=false

