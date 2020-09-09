#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

#include "controller/controller.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "controller_node");
	ros::NodeHandle nh;

	// Pull needed parameters off rosparam server
	double update_rate;
	nh.param<double>("controller/update_rate", update_rate, 100); // Default to 100 hz

	// Primary logic
	Controller controller;

	// Control loop frequency
	ros::Rate r(update_rate);
	while (ros::ok()) {
		controller.update();
		r.sleep();
	}
	return 0;
}
