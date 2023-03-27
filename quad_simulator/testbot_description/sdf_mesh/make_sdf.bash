#!/bin/bash         


echo "set up catkin"


source /home/haoluo/catkin_ws_opt/devel/setup.bash
echo "Making urdf and sdf"
rosrun xacro xacro spine_robot.urdf.xacro >test.urdf
gz sdf -p test.urdf >test.sdf

echo "Done"

