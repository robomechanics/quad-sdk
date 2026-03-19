#include <rclcpp/rclcpp.hpp>

#include "quad_utils/rviz_interface.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("rviz_interface_node");
  auto rviz_interface = std::make_shared<RVizInterface>(node);

  rviz_interface->spin();
  rclcpp::shutdown();
  return 0;
}
