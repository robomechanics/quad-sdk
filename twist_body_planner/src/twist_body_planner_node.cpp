#include <ros/ros.h>

#include "twist_body_planner/twist_body_planner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "twist_body_planner");
  ros::NodeHandle nh;

  TwistBodyPlanner twist_body_planner(nh);
  twist_body_planner.spin();
  return 0;
}
