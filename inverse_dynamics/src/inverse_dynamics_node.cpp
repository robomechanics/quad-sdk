#include <ros/ros.h>
#include <iostream>

#include "inverse_dynamics/inverse_dynamics.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle nh;

  inverseDynamics inverse_dynamics(nh);
  inverse_dynamics.spin();

  return 0;
}
