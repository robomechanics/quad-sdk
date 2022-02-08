#include "global_body_planner/global_body_planner.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_body_planner");
  ros::NodeHandle nh;

  GlobalBodyPlanner global_body_planner(nh);
  global_body_planner.spin();
  return 0;
}