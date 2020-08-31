#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "spirit_estimator.h"
#include "spirit_controller.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "spirit_estimator_controller_node");
	ros::NodeHandle nh;
	SpiritEstimator spirit_estimator(nh);
	SpiritController spirit_controller(nh);

	while (true) {
		// These will be run in parallel threads at different frequencies
		spirit_estimator.update();
		spirit_controller.update();
	}
	return 0;
}
