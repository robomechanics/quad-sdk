#include <ros/ros.h>

#include <iostream>

#include "tail_controller/tail_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "tail_controller_node");
  ros::NodeHandle nh;

  TailController tail_controller(nh);
  tail_controller.spin();

  return 0;
}
