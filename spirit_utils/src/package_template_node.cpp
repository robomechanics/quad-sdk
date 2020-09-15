#include <ros/ros.h>
#include "spirit_utils/package_template.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "package_template_node");
	ros::NodeHandle nh;

	PackageTemplate package_template(nh);

	return 0;
}
