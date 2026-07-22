#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "quad_perf_tests/cmd_vel_publisher.hpp"

namespace {

constexpr double kTol = 1e-9;

rclcpp::NodeOptions publisherNodeOptions(
    const std::vector<rclcpp::Parameter>& extra = {}) {
  const char* params = std::getenv("QUAD_PERF_TEST_PARAMS");
  const char* topic_params = std::getenv("QUAD_PERF_TEST_TOPIC_PARAMS");
  if (params == nullptr || topic_params == nullptr) {
    throw std::runtime_error("Missing quad_perf_tests parameter file env");
  }

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params, "--params-file",
                     topic_params});
  options.parameter_overrides(extra);
  return options;
}

std::shared_ptr<rclcpp::Node> makeNode(
    const std::string& name,
    const std::vector<rclcpp::Parameter>& extra = {}) {
  (void)name;
  return std::make_shared<rclcpp::Node>("cmd_vel_publisher",
                                        publisherNodeOptions(extra));
}

template <typename MsgT>
bool spinUntilMessage(const std::shared_ptr<rclcpp::Node>& node,
                      std::shared_ptr<MsgT>& msg,
                      std::chrono::milliseconds timeout =
                          std::chrono::milliseconds(1000)) {
  const auto start = std::chrono::steady_clock::now();
  while (!msg && std::chrono::steady_clock::now() - start < timeout) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return static_cast<bool>(msg);
}

void expectZeroTwist(const geometry_msgs::msg::Twist& msg) {
  EXPECT_NEAR(msg.linear.x, 0.0, kTol);
  EXPECT_NEAR(msg.linear.y, 0.0, kTol);
  EXPECT_NEAR(msg.linear.z, 0.0, kTol);
  EXPECT_NEAR(msg.angular.x, 0.0, kTol);
  EXPECT_NEAR(msg.angular.y, 0.0, kTol);
  EXPECT_NEAR(msg.angular.z, 0.0, kTol);
}

bool sameCommand(const geometry_msgs::msg::Twist& a,
                 const geometry_msgs::msg::Twist& b) {
  return a.linear.x == b.linear.x && a.linear.y == b.linear.y &&
         a.linear.z == b.linear.z && a.angular.x == b.angular.x &&
         a.angular.y == b.angular.y && a.angular.z == b.angular.z;
}

}  // namespace

TEST(CmdVelPublisherTest, ConstructorLoadsYamlConfiguration) {
  auto node = makeNode("cmd_vel_publisher_constructor_test");
  CmdVelPublisher publisher(node);

  EXPECT_EQ(publisher.mode_, "single");
  EXPECT_DOUBLE_EQ(publisher.update_rate_, 500.0);
  EXPECT_DOUBLE_EQ(publisher.resample_sec_, 3.0);
  EXPECT_DOUBLE_EQ(publisher.x_min_, -1.0);
  EXPECT_DOUBLE_EQ(publisher.x_max_, 1.0);
  EXPECT_DOUBLE_EQ(publisher.y_min_, -0.4);
  EXPECT_DOUBLE_EQ(publisher.y_max_, 0.4);
  EXPECT_DOUBLE_EQ(publisher.yaw_min_, -1.0);
  EXPECT_DOUBLE_EQ(publisher.yaw_max_, 1.0);
  EXPECT_EQ(publisher.seed_, 1);
  EXPECT_DOUBLE_EQ(publisher.test_duration_, 10.0);
  EXPECT_FALSE(publisher.has_sample_);
  expectZeroTwist(publisher.last_cmd_vel_msg_);
}

TEST(CmdVelPublisherTest, OffModePublishesZeroTwist) {
  auto node = makeNode("cmd_vel_publisher_off_test",
                       {rclcpp::Parameter("cmd_vel_publisher.mode", "off")});
  CmdVelPublisher publisher(node);
  std::shared_ptr<geometry_msgs::msg::Twist> received;
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [&](geometry_msgs::msg::Twist::SharedPtr msg) { received = msg; });

  publisher.publishCmdVel();

  ASSERT_TRUE(spinUntilMessage(node, received));
  expectZeroTwist(*received);
}

TEST(CmdVelPublisherTest, SingleModeSamplesOnceThenZerosAfterDuration) {
  auto node = makeNode("cmd_vel_publisher_single_test",
                       {rclcpp::Parameter("cmd_vel_publisher.mode", "single"),
                        rclcpp::Parameter("cmd_vel_publisher.test_duration",
                                          10.0)});
  CmdVelPublisher publisher(node);

  publisher.publishCmdVel();
  ASSERT_TRUE(publisher.has_sample_);
  const auto first = publisher.last_cmd_vel_msg_;

  publisher.publishCmdVel();
  EXPECT_TRUE(sameCommand(first, publisher.last_cmd_vel_msg_));

  publisher.last_cmd_vel_msg_time_ =
      node->now() - rclcpp::Duration::from_seconds(11.0);
  publisher.publishCmdVel();

  expectZeroTwist(publisher.last_cmd_vel_msg_);
}

TEST(CmdVelPublisherTest, TimerModeResamplesAfterThreshold) {
  auto node = makeNode("cmd_vel_publisher_timer_test",
                       {rclcpp::Parameter("cmd_vel_publisher.mode", "timer"),
                        rclcpp::Parameter("cmd_vel_publisher.resample_sec",
                                          0.5)});
  CmdVelPublisher publisher(node);

  publisher.publishCmdVel();
  const auto first = publisher.last_cmd_vel_msg_;

  publisher.publishCmdVel();
  EXPECT_TRUE(sameCommand(first, publisher.last_cmd_vel_msg_));

  publisher.last_cmd_vel_msg_time_ =
      node->now() - rclcpp::Duration::from_seconds(1.0);
  publisher.publishCmdVel();

  EXPECT_FALSE(sameCommand(first, publisher.last_cmd_vel_msg_));
}

TEST(CmdVelPublisherTest, SeededSamplingIsDeterministic) {
  auto node_a = makeNode("cmd_vel_publisher_seed_test_a",
                         {rclcpp::Parameter("cmd_vel_publisher.seed", 7)});
  auto node_b = makeNode("cmd_vel_publisher_seed_test_b",
                         {rclcpp::Parameter("cmd_vel_publisher.seed", 7)});
  CmdVelPublisher publisher_a(node_a);
  CmdVelPublisher publisher_b(node_b);

  publisher_a.sampleNewCmd();
  publisher_b.sampleNewCmd();

  EXPECT_TRUE(
      sameCommand(publisher_a.last_cmd_vel_msg_, publisher_b.last_cmd_vel_msg_));
  EXPECT_NEAR(publisher_a.last_cmd_vel_msg_.linear.z, 0.0, kTol);
  EXPECT_NEAR(publisher_a.last_cmd_vel_msg_.angular.x, 0.0, kTol);
  EXPECT_NEAR(publisher_a.last_cmd_vel_msg_.angular.y, 0.0, kTol);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
