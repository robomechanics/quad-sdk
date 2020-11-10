#include <ros/ros.h>
#include <iostream>

#include "clark_trot_controller/clark_trot_controller.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "controller_node");
	ros::NodeHandle nh;

	ClarkTrotController clark_trot_controller(nh);
	clark_trot_controller.spin();

	return 0;
}
