#include <ros/ros.h>
#include <iostream>

#include "ekf_estimator/ekf_estimator.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ekf_estimator_node");
  ros::NodeHandle nh;

  EKFEstimator ekf_estimator(nh);
  ekf_estimator.spin();

  return 0;
}
