#include <ros/ros.h>
#include "spirit_utils/rviz_interface.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "rviz_interface_node");
	ros::NodeHandle nh;

	RVizInterface rviz_interface(nh);
	rviz_interface.spin();
	
	return 0;
}
