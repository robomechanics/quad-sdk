#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

#include "ekf_estimator/ekf_estimator.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "ekf_estimator_node");
	ros::NodeHandle nh;

	// Pull needed parameters off rosparam server
	double update_rate;
	nh.param<double>("ekf_estimator/update_rate", update_rate, 200); // Default to 200

	// Primary logic
	EKFEstimator ekf_estimator(nh);

	// Control loop frequency
	ros::Rate r(update_rate);
	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
