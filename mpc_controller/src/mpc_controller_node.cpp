#include <ros/ros.h>
#include <iostream>

#include "mpc_controller/mpc_controller.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "controller_node");
	ros::NodeHandle nh;

	MPCController mpc_controller(nh);
	mpc_controller.spin();

	return 0;
}
