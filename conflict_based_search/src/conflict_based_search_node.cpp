#include <rclcpp/rclcpp.hpp>

#include "conflict_based_search/conflict_based_search.hpp"

/// ROS 2 entry point for the conflict-based search coordinator.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("conflict_based_search");
  conflict_based_search::ConflictBasedSearch cbs(node);

  // run() publishes the selected plans; keep spinning for late subscribers.
  cbs.run();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
