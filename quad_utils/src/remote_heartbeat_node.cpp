#include <rclcpp/rclcpp.hpp>

#include "quad_utils/remote_heartbeat.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("remote_heartbeat_node");
  auto remote_heartbeat = std::make_shared<RemoteHeartbeat>(node);

  remote_heartbeat->spin();
  rclcpp::shutdown();
  return 0;
}
