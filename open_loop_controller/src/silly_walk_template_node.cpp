#include <ros/ros.h>
#include <iostream>

#include "open_loop_controller/silly_walk_template.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "silly_walk_template_node");
	ros::NodeHandle nh;

	SillyWalkController silly_walk_controller(nh);
	silly_walk_controller.spin();

	return 0;
}
