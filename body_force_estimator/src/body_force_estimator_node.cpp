#include <ros/ros.h>

#include "body_force_estimator/body_force_estimator.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "body_force_estimator_node");
  ros::NodeHandle nh;

  BodyForceEstimator body_force_estimator(nh);
  body_force_estimator.spin();

  return 0;
}
