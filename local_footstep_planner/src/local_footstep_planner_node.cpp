#include <ros/ros.h>
#include "local_footstep_planner.h"

#include <yaml-cpp/yaml.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "local_footstep_planner_node");
	ros::NodeHandle nh;
	LocalFootstepPlanner local_footstep_planner(nh);
	local_footstep_planner.loop();
	return 0;
}