#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "quad_utils/trajectory_publisher.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("trajectory_publisher_node");
  TrajectoryPublisher trajectory_publisher(node);
  trajectory_publisher.spin();
  rclcpp::shutdown();
  return 0;
}
