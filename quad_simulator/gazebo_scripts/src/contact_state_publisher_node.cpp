#include <ros/ros.h>

#include <iostream>

#include "contact_state_publisher.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "contact_state_publisher_node");
  ros::NodeHandle nh;

  ContactStatePublisher contact_state_publisher(nh);
  contact_state_publisher.spin();

  return 0;
}
