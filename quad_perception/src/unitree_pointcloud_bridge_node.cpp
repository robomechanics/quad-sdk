#include <rclcpp/rclcpp.hpp>

#include "quad_perception/unitree_pointcloud_bridge.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quad_perception::UnitreePointCloudBridge>());
  rclcpp::shutdown();
  return 0;
}
