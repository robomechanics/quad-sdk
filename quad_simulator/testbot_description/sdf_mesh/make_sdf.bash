#!/bin/bash         


echo "set up catkin"


source ~/catkin_ws/devel/setup.bash
echo "Making urdf and sdf"
rosrun xacro xacro spine_robot.urdf.xacro >test.urdf
gz sdf -p test.urdf >test.sdf

echo "Done"

