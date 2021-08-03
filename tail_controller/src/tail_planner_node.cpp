#include <ros/ros.h>
#include <iostream>

#include "tail_controller/tail_planner.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "tail_planner_node");
	ros::NodeHandle nh;

	TailPlanner tail_planner(nh);
	tail_planner.spin();

	return 0;
}
