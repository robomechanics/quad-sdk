#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "local_planner/local_planner.hpp"

namespace {

constexpr double kTol = 1e-6;

std::string runXacro(const std::string& xacro_path) {
  std::array<char, 4096> buffer{};
  std::string result;
  const std::string cmd = "xacro " + xacro_path;
  FILE* pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    throw std::runtime_error("Failed to run xacro");
  }
  while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
    result += buffer.data();
  }
  const int rc = pclose(pipe);
  if (rc != 0 || result.empty()) {
    throw std::runtime_error("xacro returned no robot description");
  }
  return result;
}

std::string go2RobotDescription() {
  static const std::string urdf = []() {
    const char* source_dir = std::getenv("LOCAL_PLANNER_SOURCE_DIR");
    if (source_dir == nullptr) {
      throw std::runtime_error("Missing local_planner source env");
    }
    return runXacro(std::string(source_dir) +
                    "/quad_simulator/go2_description/models/go2/urdf/"
                    "go2.urdf.xacro");
  }();
  return urdf;
}

rclcpp::NodeOptions plannerNodeOptions(
    const std::vector<rclcpp::Parameter>& extra = {}) {
  const char* params = std::getenv("LOCAL_PLANNER_TEST_PARAMS");
  const char* topic_params = std::getenv("LOCAL_PLANNER_TOPIC_PARAMS");
  const char* robot_params = std::getenv("LOCAL_PLANNER_ROBOT_PARAMS");
  const char* nmpc_params = std::getenv("LOCAL_PLANNER_NMPC_PARAMS");
  if (params == nullptr || topic_params == nullptr || robot_params == nullptr ||
      nmpc_params == nullptr) {
    throw std::runtime_error("Missing local_planner test parameter file env");
  }

  std::vector<rclcpp::Parameter> overrides = {
      rclcpp::Parameter("namespace", "robot_1"),
      rclcpp::Parameter("robot_type", "go2"),
      rclcpp::Parameter("robot_description", go2RobotDescription()),
  };
  overrides.insert(overrides.end(), extra.begin(), extra.end());

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params, "--params-file",
                     topic_params, "--params-file", robot_params, "--params-file",
                     nmpc_params});
  options.parameter_overrides(overrides);
  return options;
}

std::shared_ptr<rclcpp::Node> makeNode(
    const std::vector<rclcpp::Parameter>& extra = {}) {
  return std::make_shared<rclcpp::Node>("local_planner",
                                        plannerNodeOptions(extra));
}

quad_msgs::msg::RobotState makePopulatedState() {
  quad_msgs::msg::RobotState state;
  state.header.stamp.sec = 1;
  state.body.pose.position.z = 0.27;
  state.body.pose.orientation.w = 1.0;
  state.joints.position.assign(12, 0.0);
  state.joints.velocity.assign(12, 0.0);
  state.feet.feet.resize(4);
  for (int i = 0; i < 4; ++i) {
    state.feet.feet[i].position.x = (i < 2) ? 0.2 : -0.2;
    state.feet.feet[i].position.y = (i % 2 == 0) ? 0.1 : -0.1;
    state.feet.feet[i].position.z = 0.02;
  }
  return state;
}

}  // namespace

TEST(LocalPlannerTest, ConstructorLoadsYamlConfigurationAndInterfaces) {
  auto node = makeNode();
  LocalPlanner planner(node);

  EXPECT_EQ(planner.robot_name_, "go2");
  EXPECT_EQ(planner.robot_ns_, "robot_1");
  EXPECT_EQ(planner.map_frame_, "map");
  EXPECT_DOUBLE_EQ(planner.update_rate_, 333.0);
  EXPECT_DOUBLE_EQ(planner.dt_, 0.03);
  EXPECT_EQ(planner.N_, 26);
  EXPECT_NE(planner.terrain_map_sub_, nullptr);
  EXPECT_NE(planner.body_plan_sub_, nullptr);
  EXPECT_NE(planner.robot_state_sub_, nullptr);
  EXPECT_NE(planner.cmd_vel_sub_, nullptr);
  EXPECT_NE(planner.local_plan_pub_, nullptr);
  EXPECT_NE(planner.foot_plan_discrete_pub_, nullptr);
  EXPECT_NE(planner.foot_plan_continuous_pub_, nullptr);
  EXPECT_NE(planner.local_body_planner_nonlinear_, nullptr);
  EXPECT_NE(planner.local_footstep_planner_, nullptr);
}

TEST(LocalPlannerTest, RobotStateCallbackIgnoresEmptyAndAcceptsPopulatedState) {
  auto node = makeNode();
  LocalPlanner planner(node);

  auto empty_state = std::make_shared<quad_msgs::msg::RobotState>();
  planner.robotStateCallback(empty_state);
  EXPECT_EQ(planner.robot_state_msg_, nullptr);

  auto valid_state =
      std::make_shared<quad_msgs::msg::RobotState>(makePopulatedState());
  planner.robotStateCallback(valid_state);
  ASSERT_NE(planner.robot_state_msg_, nullptr);
  EXPECT_EQ(planner.robot_state_msg_->joints.position.size(), 12u);
  EXPECT_EQ(planner.robot_state_msg_->feet.feet.size(), 4u);
}

TEST(LocalPlannerTest, RobotPlanCallbackStoresLatestPlan) {
  auto node = makeNode();
  LocalPlanner planner(node);
  auto plan = std::make_shared<quad_msgs::msg::RobotPlan>();
  plan->plan_indices.push_back(42);

  planner.robotPlanCallback(plan);

  ASSERT_NE(planner.body_plan_msg_, nullptr);
  EXPECT_EQ(planner.body_plan_msg_->plan_indices.front(), 42);
}

TEST(LocalPlannerTest, CmdVelCallbackFiltersPlanarTwistOnly) {
  auto node = makeNode();
  LocalPlanner planner(node);
  planner.cmd_vel_.setZero();
  planner.cmd_vel_filter_const_ = 0.5;
  planner.cmd_vel_scale_ = 2.0;

  auto twist = std::make_shared<geometry_msgs::msg::Twist>();
  twist->linear.x = 1.0;
  twist->linear.y = -0.5;
  twist->linear.z = 10.0;
  twist->angular.x = 3.0;
  twist->angular.y = 4.0;
  twist->angular.z = 0.25;

  planner.cmdVelCallback(twist);

  EXPECT_NEAR(planner.cmd_vel_[0], 1.0, kTol);
  EXPECT_NEAR(planner.cmd_vel_[1], -0.5, kTol);
  EXPECT_DOUBLE_EQ(planner.cmd_vel_[2], 0.0);
  EXPECT_DOUBLE_EQ(planner.cmd_vel_[3], 0.0);
  EXPECT_DOUBLE_EQ(planner.cmd_vel_[4], 0.0);
  EXPECT_NEAR(planner.cmd_vel_[5], 0.25, kTol);
}

TEST(LocalPlannerTest, ComputeLocalPlanRejectsMissingInputs) {
  auto node = makeNode();
  LocalPlanner planner(node);

  EXPECT_FALSE(planner.computeLocalPlan());
}

TEST(LocalPlannerTest, UnwrapYawReferenceRemovesPiDiscontinuity) {
  auto node = makeNode();
  LocalPlanner planner(node);
  planner.current_state_ = Eigen::VectorXd::Zero(12);
  planner.current_state_[5] = 3.10;
  planner.ref_body_plan_ = Eigen::MatrixXd::Zero(3, 12);
  planner.ref_body_plan_(0, 5) = -3.10;
  planner.ref_body_plan_(1, 5) = -3.12;
  planner.ref_body_plan_(2, 5) = 3.13;

  planner.unwrapYawReference();

  EXPECT_NEAR(planner.ref_body_plan_(0, 5), 3.183185307, 1e-5);
  EXPECT_LT(std::abs(planner.ref_body_plan_(1, 5) -
                     planner.ref_body_plan_(0, 5)),
            M_PI);
  EXPECT_LT(std::abs(planner.ref_body_plan_(2, 5) -
                     planner.ref_body_plan_(1, 5)),
            M_PI);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
