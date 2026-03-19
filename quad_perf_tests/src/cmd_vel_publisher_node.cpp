#include <rclcpp/rclcpp.hpp>
#include "quad_perf_tests/cmd_vel_publisher.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cmd_vel_publisher_node");
  CmdVelPublisher cmd_vel_publisher(node);
  cmd_vel_publisher.spin();
  rclcpp::shutdown();
  return 0;
}
