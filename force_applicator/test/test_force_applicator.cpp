#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "force_applicator/force_applicator.hpp"

namespace {

constexpr double kTol = 1e-6;

rclcpp::NodeOptions forceNodeOptions(
    const std::string& mode = "single", const std::string& force_mode = "yaml",
    const std::string& link = "robot_1::body",
    const std::vector<rclcpp::Parameter>& extra = {}) {
  const char* params = std::getenv("FORCE_APPLICATOR_TEST_PARAMS");
  const char* topic_params = std::getenv("FORCE_APPLICATOR_TOPIC_PARAMS");
  if (params == nullptr || topic_params == nullptr) {
    throw std::runtime_error(
        "Missing force_applicator test parameter file environment");
  }

  std::vector<rclcpp::Parameter> overrides = {
      rclcpp::Parameter("world", "flat.sdf"),
      rclcpp::Parameter("robot_type", "go2"),
      rclcpp::Parameter("robot_ns", "robot_1"),
      rclcpp::Parameter("mode", mode),
      rclcpp::Parameter("force_mode", force_mode),
      rclcpp::Parameter("link", link),
  };
  overrides.insert(overrides.end(), extra.begin(), extra.end());

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params, "--params-file",
                     topic_params});
  options.parameter_overrides(overrides);
  return options;
}

std::shared_ptr<rclcpp::Node> makeNode(
    const std::string& name,
    const rclcpp::NodeOptions& options = forceNodeOptions()) {
  return std::make_shared<rclcpp::Node>(name, options);
}

quad_msgs::msg::RobotState makeState(double x, double y, double z) {
  quad_msgs::msg::RobotState state;
  state.body.pose.position.x = x;
  state.body.pose.position.y = y;
  state.body.pose.position.z = z;
  return state;
}

template <typename Predicate>
bool spinUntil(const std::shared_ptr<rclcpp::Node>& node,
               const std::shared_ptr<rclcpp::Node>& listener,
               Predicate predicate,
               std::chrono::milliseconds timeout = std::chrono::seconds(1)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}

}  // namespace

TEST(ForceApplicatorTest, ConstructorLoadsParametersAndTopics) {
  auto node = makeNode("force_applicator");
  ForceApplicator app(node);

  EXPECT_EQ(app.world_name_, "flat");
  EXPECT_EQ(app.robot_type_, "go2");
  EXPECT_EQ(app.robot_ns_, "robot_1");
  EXPECT_EQ(app.mode_, "single");
  EXPECT_EQ(app.force_mode_, "yaml");
  EXPECT_EQ(app.link_, "robot_1::body");
  EXPECT_EQ(app.wrench_topic_persist_, "/world/flat/wrench/persistent");
  EXPECT_EQ(app.wrench_topic_clear_, "/world/flat/wrench/clear");
  EXPECT_NE(app.robot_state_sub_, nullptr);
  EXPECT_NE(app.force_marker_pub_, nullptr);
  EXPECT_NE(app.wrench_persist_pub_, nullptr);
  EXPECT_NE(app.wrench_clear_pub_, nullptr);
}

TEST(ForceApplicatorTest, SingleModeTriggersOnce) {
  auto node = makeNode("force_applicator");
  ForceApplicator app(node);

  EXPECT_TRUE(app.shouldTrigger());
  EXPECT_FALSE(app.shouldTrigger());
}

TEST(ForceApplicatorTest, PeriodicModeWaitsForInitialTimerThenFires) {
  auto node = makeNode(
      "force_applicator",
      forceNodeOptions("periodic", "yaml", "robot_1::body",
                       {rclcpp::Parameter("force_applicator.periodic.period",
                                          0.0)}));
  ForceApplicator app(node);

  EXPECT_FALSE(app.shouldTrigger());
  EXPECT_TRUE(app.shouldTrigger());
}

TEST(ForceApplicatorTest, DistanceModeUsesRobotStateCallback) {
  auto node = makeNode("force_applicator",
                       forceNodeOptions("distance", "yaml"));
  ForceApplicator app(node);

  app.last_robot_pose_ = Eigen::Vector3d::Zero();
  EXPECT_FALSE(app.shouldTrigger());

  auto near_state = std::make_shared<quad_msgs::msg::RobotState>(
      makeState(0.5, 0.0, 0.0));
  app.robotStateCallback(near_state);
  EXPECT_FALSE(app.shouldTrigger());

  auto far_state = std::make_shared<quad_msgs::msg::RobotState>(
      makeState(1.5, 0.0, 0.0));
  app.robotStateCallback(far_state);
  EXPECT_TRUE(app.shouldTrigger());
  EXPECT_NEAR(app.last_robot_pose_.x(), 1.5, kTol);
}

TEST(ForceApplicatorTest, YamlForcePublishesWrenchMarkerAndClear) {
  auto node = makeNode("force_applicator");
  auto listener = std::make_shared<rclcpp::Node>("force_listener_yaml");
  ForceApplicator app(node);

  std::optional<ros_gz_interfaces::msg::EntityWrench> wrench;
  std::optional<ros_gz_interfaces::msg::Entity> clear;
  std::optional<visualization_msgs::msg::Marker> marker;

  auto wrench_sub =
      listener->create_subscription<ros_gz_interfaces::msg::EntityWrench>(
          "wrench/persistent", 10,
          [&](const ros_gz_interfaces::msg::EntityWrench::SharedPtr msg) {
            wrench = *msg;
          });
  auto clear_sub =
      listener->create_subscription<ros_gz_interfaces::msg::Entity>(
          "wrench/clear", 10,
          [&](const ros_gz_interfaces::msg::Entity::SharedPtr msg) {
            clear = *msg;
          });
  auto marker_sub =
      listener->create_subscription<visualization_msgs::msg::Marker>(
          "visualization/force_torque_markers", 10,
          [&](const visualization_msgs::msg::Marker::SharedPtr msg) {
            marker = *msg;
          });

  app.applyForce();
  app.updateMarker();

  ASSERT_TRUE(spinUntil(node, listener,
                        [&]() { return wrench && marker; }));
  EXPECT_EQ(wrench->entity.name, "robot_1::body");
  EXPECT_EQ(wrench->entity.type, ros_gz_interfaces::msg::Entity::LINK);
  EXPECT_DOUBLE_EQ(wrench->wrench.force.x, app.force_x_);
  EXPECT_DOUBLE_EQ(wrench->wrench.force.y, app.force_y_);
  EXPECT_DOUBLE_EQ(wrench->wrench.force.z, app.force_z_);
  EXPECT_DOUBLE_EQ(wrench->wrench.torque.x, app.torque_x_);
  EXPECT_DOUBLE_EQ(wrench->wrench.torque.y, app.torque_y_);
  EXPECT_DOUBLE_EQ(wrench->wrench.torque.z, app.torque_z_);

  EXPECT_EQ(marker->type, visualization_msgs::msg::Marker::ARROW);
  EXPECT_EQ(marker->ns, "apply_force");
  EXPECT_NEAR(tf2::getYaw(marker->pose.orientation), 0.0, kTol);

  ASSERT_TRUE(spinUntil(node, listener, [&]() { return clear.has_value(); },
                        std::chrono::seconds(2)));
  EXPECT_EQ(clear->name, "robot_1::body");
  EXPECT_EQ(clear->type, ros_gz_interfaces::msg::Entity::LINK);
}

TEST(ForceApplicatorTest, MarkerYawFollowsHorizontalForceDirection) {
  auto node = makeNode(
      "force_applicator",
      forceNodeOptions("single", "yaml", "robot_1::body",
                       {rclcpp::Parameter(
                            "force_applicator.fixed.force.x", 0.0),
                        rclcpp::Parameter(
                            "force_applicator.fixed.force.y", 10.0)}));
  auto listener = std::make_shared<rclcpp::Node>("force_listener_marker");
  ForceApplicator app(node);

  std::optional<visualization_msgs::msg::Marker> marker;
  auto marker_sub =
      listener->create_subscription<visualization_msgs::msg::Marker>(
          "visualization/force_torque_markers", 10,
          [&](const visualization_msgs::msg::Marker::SharedPtr msg) {
            marker = *msg;
          });

  app.applyForce();
  app.updateMarker();

  ASSERT_TRUE(spinUntil(node, listener, [&]() { return marker.has_value(); }));
  EXPECT_NEAR(tf2::getYaw(marker->pose.orientation), M_PI / 2.0, kTol);
}

TEST(ForceApplicatorTest, RandomForcePublishesBoundedDownwardWrench) {
  auto node = makeNode("force_applicator",
                       forceNodeOptions("single", "random"));
  auto listener = std::make_shared<rclcpp::Node>("force_listener_random");
  ForceApplicator app(node);

  std::optional<ros_gz_interfaces::msg::EntityWrench> wrench;
  auto wrench_sub =
      listener->create_subscription<ros_gz_interfaces::msg::EntityWrench>(
          "wrench/persistent", 10,
          [&](const ros_gz_interfaces::msg::EntityWrench::SharedPtr msg) {
            wrench = *msg;
          });

  app.applyForce();

  ASSERT_TRUE(spinUntil(node, listener, [&]() { return wrench.has_value(); }));
  const Eigen::Vector3d f(wrench->wrench.force.x, wrench->wrench.force.y,
                          wrench->wrench.force.z);
  EXPECT_GE(f.norm(), app.force_mag_min_);
  EXPECT_LE(f.norm(), app.force_mag_max_);
  EXPECT_LE(wrench->wrench.force.z, 0.0);
  EXPECT_GE(wrench->wrench.torque.x, app.torque_mag_min_);
  EXPECT_LE(wrench->wrench.torque.x, app.torque_mag_max_);
  EXPECT_GE(wrench->wrench.torque.y, app.torque_mag_min_);
  EXPECT_LE(wrench->wrench.torque.y, app.torque_mag_max_);
  EXPECT_GE(wrench->wrench.torque.z, app.torque_mag_min_);
  EXPECT_LE(wrench->wrench.torque.z, app.torque_mag_max_);
}

TEST(ForceApplicatorTest, UnknownForceModePublishesZeroWrench) {
  auto node = makeNode("force_applicator",
                       forceNodeOptions("single", "invalid"));
  auto listener = std::make_shared<rclcpp::Node>("force_listener_invalid");
  ForceApplicator app(node);

  std::optional<ros_gz_interfaces::msg::EntityWrench> wrench;
  auto wrench_sub =
      listener->create_subscription<ros_gz_interfaces::msg::EntityWrench>(
          "wrench/persistent", 10,
          [&](const ros_gz_interfaces::msg::EntityWrench::SharedPtr msg) {
            wrench = *msg;
          });

  app.applyForce();

  ASSERT_TRUE(spinUntil(node, listener, [&]() { return wrench.has_value(); }));
  EXPECT_DOUBLE_EQ(wrench->wrench.force.x, 0.0);
  EXPECT_DOUBLE_EQ(wrench->wrench.force.y, 0.0);
  EXPECT_DOUBLE_EQ(wrench->wrench.force.z, 0.0);
  EXPECT_DOUBLE_EQ(wrench->wrench.torque.x, 0.0);
  EXPECT_DOUBLE_EQ(wrench->wrench.torque.y, 0.0);
  EXPECT_DOUBLE_EQ(wrench->wrench.torque.z, 0.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
