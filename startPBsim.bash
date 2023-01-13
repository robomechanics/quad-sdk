#!/bin/bash         


echo "set up Sims"

cd ~/catkin_ws_opt
source devel/setup.bash

echo "done!"

roslaunch quad_utils quad_pybullet.launch robot_type:=spirit gui:=true rviz_gui:=false

