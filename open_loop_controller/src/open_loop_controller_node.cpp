#include <ros/ros.h>
#include <iostream>

#include "open_loop_controller/open_loop_controller.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "open_loop_controller_node");
	ros::NodeHandle nh;

	OpenLoopController open_loop_controller(nh);
	open_loop_controller.spin();

	return 0;
}
