#include <ros/ros.h>
#include <iostream>

#include "nmpc_controller/nmpc_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle nh;

  NMPCController nmpc_controller(nh);
  nmpc_controller.spin();

  return 0;
}
