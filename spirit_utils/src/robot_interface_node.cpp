#include <ros/ros.h>
#include "spirit_utils/robot_interface.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_interface_node");
	ros::NodeHandle nh;

	RobotInterface robot_interface(nh);
	robot_interface.spin();
	
	return 0;
}
