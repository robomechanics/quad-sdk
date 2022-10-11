#!/bin/bash         


echo "set up Sims"

cd ~/catkin_ws2
source devel/setup.bash

echo "done!"

rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot_1/cmd_vel

