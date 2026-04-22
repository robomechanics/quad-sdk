#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <iostream>

#include "robot_driver/robot_driver.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robot_driver_node");
  RobotDriver robot_driver(node, argc, argv);
  robot_driver.spin();
  rclcpp::shutdown();

  // Bypass libstdc++/onnxruntime global static teardown which crashes with
  // "free(): invalid next size" on this image. All ROS2 work has already
  // completed cleanly above; _Exit skips destructors and exits immediately.
  std::_Exit(0);
}
