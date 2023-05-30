#include <ros/ros.h>

#include <iostream>

#include "spine_controller/spine_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "spine_controller_node");
  ros::NodeHandle nh;

  SpineController spine_controller(nh);
  spine_controller.spin();

  return 0;
}
