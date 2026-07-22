#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#define private public
#include "quad_utils/remote_heartbeat.hpp"
#undef private

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace {

rclcpp::NodeOptions heartbeatOptions() {
  const std::string pkg_share =
      ament_index_cpp::get_package_share_directory("quad_utils");
  const std::string params_file = pkg_share + "/config/remote_heartbeat.yaml";

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params_file});
  options.append_parameter_override("use_sim_time", false);
  return options;
}

}  // namespace

TEST(RemoteHeartbeatTest, LoadsConfiguredTopicsAndThresholds) {
  auto node =
      std::make_shared<rclcpp::Node>("remote_heartbeat", heartbeatOptions());
  RemoteHeartbeat heartbeat(node);

  EXPECT_GT(heartbeat.update_rate_, 0.0);
  EXPECT_GT(heartbeat.robot_latency_threshold_warn_, 0.0);
  EXPECT_GT(heartbeat.robot_latency_threshold_error_, 0.0);
  EXPECT_NE(heartbeat.remote_heartbeat_pub_, nullptr);
  EXPECT_NE(heartbeat.robot_heartbeat_sub_, nullptr);
}

TEST(RemoteHeartbeatTest, AcceptsFreshAndStaleRobotHeartbeats) {
  auto node =
      std::make_shared<rclcpp::Node>("remote_heartbeat", heartbeatOptions());
  RemoteHeartbeat heartbeat(node);

  auto fresh = std::make_shared<std_msgs::msg::Header>();
  fresh->stamp = node->now();
  EXPECT_NO_THROW(heartbeat.robotHeartbeatCallback(fresh));

  auto stale = std::make_shared<std_msgs::msg::Header>();
  stale->stamp = node->now() - rclcpp::Duration::from_seconds(1.0);
  EXPECT_NO_THROW(heartbeat.robotHeartbeatCallback(stale));
}
