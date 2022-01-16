#include <ros/ros.h>
#include "robot_driver/leg_overrider.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "leg_overrider_node");
  ros::NodeHandle nh;

  LegOverrider leg_overrider(nh);
  leg_overrider.spin();

  return 0;
}
