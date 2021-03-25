#include <ros/ros.h>
#include <iostream>

#include "local_planner/local_planner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_planner_node");
  ros::NodeHandle nh;

  LocalPlanner local_planner(nh);
  local_planner.spin();

  return 0;
}
