#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "local_planner/local_planner.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("local_planner_node");
  LocalPlanner local_planner(node);
  local_planner.spin();
  rclcpp::shutdown();
  return 0;
}
