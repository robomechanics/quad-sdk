#include <ros/ros.h>
#include "spirit_utils/robot_proxy.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_proxy_node");
  ros::NodeHandle nh;

  RobotProxy robot_proxy(nh);
  robot_proxy.spin();
  
  return 0;
}
