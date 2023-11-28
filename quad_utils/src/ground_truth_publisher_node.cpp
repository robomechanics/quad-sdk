#include <ros/ros.h>
#include <iostream>

#include "quad_utils/ground_truth_publisher.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_truth_publisher_node");
  ros::NodeHandle nh;

  GroundTruthPublisher ground_truth_publisher(nh);
  ground_truth_publisher.spin();

  return 0;
}
