#include <rclcpp/rclcpp.hpp>

#include "global_body_planner/global_body_planner.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("global_body_planner");
  GlobalBodyPlanner global_body_planner(node);
  global_body_planner.spin();
  rclcpp::shutdown();
  return 0;
}
