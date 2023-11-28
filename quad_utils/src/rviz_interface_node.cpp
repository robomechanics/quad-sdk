#include <ros/ros.h>
<<<<<<< HEAD:quad_utils/src/rviz_interface_node.cpp

=======
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*:spirit_utils/src/rviz_interface_node.cpp
#include "quad_utils/rviz_interface.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rviz_interface_node");
  ros::NodeHandle nh;

  RVizInterface rviz_interface(nh);
  rviz_interface.spin();

  return 0;
}
