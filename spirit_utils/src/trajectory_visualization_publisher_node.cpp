#include <ros/ros.h>
#include <iostream>

#include "spirit_utils/trajectory_visualization_publisher.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_visualization_publisher_node");
  ros::NodeHandle nh;

  TrajectoryVisualizationPublisher trajectory_visualization_publisher(nh);
  trajectory_visualization_publisher.spin();

  return 0;
}
