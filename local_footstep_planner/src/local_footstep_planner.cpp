#include "local_footstep_planner.h"

LocalFootstepPlanner::LocalFootstepPlanner(ros::NodeHandle nh) {
  _nh = nh;
}

void LocalFootstepPlanner::loop() {
while (ros::ok())
{
  ros::spinOnce();
}

}