#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "quad_utils/rviz_interface.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

TEST(RVizInterface, testTrue) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  // Load the Params Needed to Successfully Launch the Node
  std::string pkg_share =
      ament_index_cpp::get_package_share_directory("quad_utils");
  std::string params_file = pkg_share + "/config/rviz_visualization.yaml";

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params_file});
  options.append_parameter_override("tf_prefix", "robot_1");
  options.append_parameter_override("use_sim_time", false);

  auto node = std::make_shared<rclcpp::Node>("rviz_interface", options);
  RVizInterface rviz_interface(node);
  EXPECT_EQ(1 + 1, 2);
}
