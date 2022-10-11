#!/bin/bash         


echo "set up Sims"

cd ~/catkin_ws2
source devel/setup.bash

echo "done!"

roslaunch quad_utils quad_gazebo.launch gui:=true rviz_gui:=false robot_type:=test rviz_onoff:=false

