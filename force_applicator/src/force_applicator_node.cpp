#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "force_applicator/force_applicator.hpp"

/// ROS 2 entry point for the force applicator node.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("force_applicator_node");
  ForceApplicator force_applicator(node);
  force_applicator.spin();
  rclcpp::shutdown();
  return 0;
}
