#include <ros/ros.h>
#include <iostream>

#include "leg_controller/leg_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "leg_controller_node");
  ros::NodeHandle nh;

  LegController leg_controller(nh);
  leg_controller.spin();

  return 0;
}
