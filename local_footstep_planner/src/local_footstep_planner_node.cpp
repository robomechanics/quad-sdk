#include <ros/ros.h>
#include "local_footstep_planner/local_footstep_planner.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "local_footstep_planner_node");
	ros::NodeHandle nh;

	LocalFootstepPlanner local_footstep_planner(nh);
	local_footstep_planner.spin();
	return 0;
}