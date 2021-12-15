#include <ros/ros.h>
#include <iostream>

#include "leg_controller/leg_controller_interface.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "leg_controller_node");
  ros::NodeHandle nh;

  LegControllerInterface leg_controller_interface(nh, argc, argv);
  leg_controller_interface.spin();

  return 0;
}
