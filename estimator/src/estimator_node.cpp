#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

#include "estimator/estimator.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "estimator_node");
	ros::NodeHandle nh;

	// Pull needed parameters off rosparam server
	double update_rate;
	nh.param<double>("estimator/update_rate", update_rate, 200); // Default to 200

	// Primary logic
	Estimator estimator;

	// ROS Publishers and Subscribers here

	// Control loop frequency
	ros::Rate r(update_rate);
	while (ros::ok()) {
		estimator.update();
		r.sleep();
	}
	return 0;
}
