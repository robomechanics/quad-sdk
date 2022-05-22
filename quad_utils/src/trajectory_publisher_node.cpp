#include <ros/ros.h>

#include "quad_utils/trajectory_publisher.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_publisher_node");
  ros::NodeHandle nh;

  TrajectoryPublisher trajectory_publisher(nh);
  trajectory_publisher.spin();

  return 0;
}
