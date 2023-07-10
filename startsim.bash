#!/bin/bash         


echo "set up Sims"

cd ~/catkin_ws
source devel/setup.bash

echo "done!"

# roslaunch quad_utils quad_gazebo.launch gui:=true rviz_gui:=true
roslaunch quad_utils quad_gazebo.launch gui:=true rviz_gui:=true robot_type:=test_cg
