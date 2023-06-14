#include <ros/ros.h>

#include <iostream>

#include "robot_driver/robot_driver.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_driver_node");
  ros::NodeHandle nh;
  RobotDriver robot_driver(nh, argc, argv);
  //rosinfo
  robot_driver.spin();
  return 0;
}
