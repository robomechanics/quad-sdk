#include <ros/ros.h>
#include <iostream>

#include "mblink_converter/mblink_converter.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mblink_converter_node");
  ros::NodeHandle nh;

  MBLinkConverter mblink_converter(nh, argc, argv);
  mblink_converter.spin();

  return 0;
}
