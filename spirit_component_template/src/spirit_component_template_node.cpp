#include <ros/ros.h>
#include "spirit_component_template.h"

#include <yaml-cpp/yaml.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "spirit_component_template_node");
	ros::NodeHandle nh;
	SpiritComponentTemplate spirit_component(nh);
	spirit_component.loop();
	return 0;
}
