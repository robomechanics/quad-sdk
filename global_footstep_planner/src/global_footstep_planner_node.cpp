#include <ros/ros.h>

#include "global_footstep_planner/global_footstep_planner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_footstep_planner_node");
  ros::NodeHandle nh;

  GlobalFootstepPlanner global_footstep_planner(nh);
  global_footstep_planner.spin();
  return 0;
}
