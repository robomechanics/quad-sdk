#include <rclcpp/rclcpp.hpp>

#include "body_force_estimator/body_force_estimator.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("body_force_estimator");
  BodyForceEstimator body_force_estimator(node);
  body_force_estimator.spin();
  rclcpp::shutdown();
  return 0;
}
