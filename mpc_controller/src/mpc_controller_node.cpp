#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

#include "mpc_controller/mpc_controller.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "controller_node");
	ros::NodeHandle nh;

	// Pull needed parameters off rosparam server
	double update_rate;
	nh.param<double>("mpc_controller/update_rate", update_rate, 100); // Default to 100 hz

	// Primary logic
	MPCController mpc_controller(nh);

	// Control loop frequency
	ros::Rate r(update_rate);
	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
