#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "conflict_based_search/conflict_based_search.hpp"

namespace {

constexpr double kTol = 1e-6;

rclcpp::NodeOptions cbsNodeOptions() {
  const char* params = std::getenv("CONFLICT_BASED_SEARCH_TEST_PARAMS");
  if (params == nullptr) {
    throw std::runtime_error(
        "Missing conflict_based_search test parameter file environment");
  }

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params});
  return options;
}

builtin_interfaces::msg::Time stamp(double seconds) {
  builtin_interfaces::msg::Time msg;
  const int64_t nanoseconds = static_cast<int64_t>(seconds * 1e9);
  msg.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  msg.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return msg;
}

quad_msgs::msg::RobotState makeState(double t, double x, double y, double z,
                                     double yaw) {
  quad_msgs::msg::RobotState state;
  state.header.stamp = stamp(t);
  state.body.header.stamp = state.header.stamp;
  state.body.pose.position.x = x;
  state.body.pose.position.y = y;
  state.body.pose.position.z = z;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  state.body.pose.orientation.x = q.x();
  state.body.pose.orientation.y = q.y();
  state.body.pose.orientation.z = q.z();
  state.body.pose.orientation.w = q.w();
  return state;
}

quad_msgs::msg::RobotPlan makePlan(
    const std::vector<quad_msgs::msg::RobotState>& states) {
  quad_msgs::msg::RobotPlan plan;
  plan.states = states;
  if (!states.empty()) {
    plan.header.stamp = states.front().header.stamp;
    plan.global_plan_timestamp = states.front().header.stamp;
  }
  return plan;
}

std::shared_ptr<conflict_based_search::CBSNode> makeSearchNode(
    const std::vector<std::string>& robots) {
  auto node = std::make_shared<conflict_based_search::CBSNode>();
  node->robot_names = robots;
  for (const auto& robot : robots) {
    node->constraints[robot] = quad_msgs::msg::RobotPlanConstraints();
  }
  return node;
}

std::shared_ptr<conflict_based_search::ConflictBasedSearch> makeCbs() {
  auto node = std::make_shared<rclcpp::Node>(
      "conflict_based_search", cbsNodeOptions());
  return std::make_shared<conflict_based_search::ConflictBasedSearch>(node);
}

}  // namespace

TEST(ConflictBasedSearchConflictTest, FindFirstConflictReturnsFalseWhenClear) {
  const auto cbs = makeCbs();
  auto node = makeSearchNode({"robot_1", "robot_2"});
  node->robot_plan_map["robot_1"] = makePlan({
      makeState(0.0, 0.0, 0.0, 0.3, 0.0),
      makeState(1.0, 1.0, 0.0, 0.3, 0.0),
  });
  node->robot_plan_map["robot_2"] = makePlan({
      makeState(0.0, 0.0, 5.0, 0.3, 0.0),
      makeState(1.0, 1.0, 5.0, 0.3, 0.0),
  });

  conflict_based_search::Conflict conflict;

  EXPECT_FALSE(cbs->findFirstConflict(*node, conflict));
}

TEST(ConflictBasedSearchConflictTest, FindFirstConflictDetectsOverlapWindow) {
  const auto cbs = makeCbs();
  auto node = makeSearchNode({"robot_1", "robot_2"});
  node->robot_plan_map["robot_1"] = makePlan({
      makeState(0.0, -2.0, 0.0, 0.3, 0.0),
      makeState(1.0, 0.0, 0.0, 0.3, 0.0),
      makeState(2.0, 0.1, 0.0, 0.3, 0.0),
      makeState(3.0, 2.0, 0.0, 0.3, 0.0),
  });
  node->robot_plan_map["robot_2"] = makePlan({
      makeState(0.0, 5.0, 0.0, 0.3, 0.0),
      makeState(1.0, 0.0, 0.0, 0.3, 0.0),
      makeState(2.0, 0.1, 0.0, 0.3, 0.0),
      makeState(3.0, 5.0, 0.0, 0.3, 0.0),
  });

  conflict_based_search::Conflict conflict;

  ASSERT_TRUE(cbs->findFirstConflict(*node, conflict));
  EXPECT_EQ(conflict.robot_a, "robot_1");
  EXPECT_EQ(conflict.robot_b, "robot_2");
  EXPECT_EQ(conflict.t_start_idx, 1);
  EXPECT_EQ(conflict.t_end_idx, 2);
}

TEST(ConflictBasedSearchConflictTest, FindFirstConflictUsesMidSegmentCheck) {
  const auto cbs = makeCbs();
  auto node = makeSearchNode({"robot_1", "robot_2"});
  node->robot_plan_map["robot_1"] = makePlan({
      makeState(0.0, -1.0, 0.0, 0.3, 0.0),
      makeState(1.0, 1.0, 0.0, 0.3, 0.0),
  });
  node->robot_plan_map["robot_2"] = makePlan({
      makeState(0.0, 0.0, 0.0, 0.3, 0.0),
      makeState(1.0, 0.0, 0.0, 0.3, 0.0),
  });

  conflict_based_search::Conflict conflict;

  ASSERT_TRUE(cbs->findFirstConflict(*node, conflict));
  EXPECT_EQ(conflict.t_start_idx, 0);
}

TEST(ConflictBasedSearchConflictTest, FindFirstConflictChoosesEarliestOfThree) {
  const auto cbs = makeCbs();
  auto node = makeSearchNode({"robot_1", "robot_2", "robot_3"});
  node->robot_plan_map["robot_1"] = makePlan({
      makeState(0.0, 0.0, 5.0, 0.3, 0.0),
      makeState(1.0, 0.0, 5.0, 0.3, 0.0),
      makeState(2.0, 0.0, 0.0, 0.3, 0.0),
  });
  node->robot_plan_map["robot_2"] = makePlan({
      makeState(0.0, 0.0, -5.0, 0.3, 0.0),
      makeState(1.0, 0.0, -5.0, 0.3, 0.0),
      makeState(2.0, 0.0, 0.0, 0.3, 0.0),
  });
  node->robot_plan_map["robot_3"] = makePlan({
      makeState(0.0, 0.0, 5.0, 0.3, 0.0),
      makeState(1.0, 0.0, 5.0, 0.3, 0.0),
      makeState(2.0, 5.0, 5.0, 0.3, 0.0),
  });

  conflict_based_search::Conflict conflict;

  ASSERT_TRUE(cbs->findFirstConflict(*node, conflict));
  EXPECT_EQ(conflict.robot_a, "robot_1");
  EXPECT_EQ(conflict.robot_b, "robot_3");
  EXPECT_EQ(conflict.t_start_idx, 0);
}

TEST(ConflictBasedSearchConflictTest, TrailingConflictUsesCollisionStartTime) {
  const auto cbs = makeCbs();
  auto node = makeSearchNode({"robot_1", "robot_2", "robot_3"});
  node->robot_plan_map["robot_1"] = makePlan({
      makeState(0.0, 0.0, 0.0, 0.3, 0.0),
      makeState(1.0, 10.0, 0.0, 0.3, 0.0),
      makeState(2.0, 10.0, 0.0, 0.3, 0.0),
  });
  node->robot_plan_map["robot_2"] = makePlan({
      makeState(0.0, 5.0, 5.0, 0.3, 0.0),
      makeState(1.0, 10.0, 0.0, 0.3, 0.0),
      makeState(2.0, 10.0, 0.0, 0.3, 0.0),
  });
  node->robot_plan_map["robot_3"] = makePlan({
      makeState(0.0, 0.0, 5.0, 0.3, 0.0),
      makeState(1.0, 0.0, 5.0, 0.3, 0.0),
      makeState(2.0, 0.0, 5.0, 0.3, 0.0),
  });

  conflict_based_search::Conflict conflict;

  ASSERT_TRUE(cbs->findFirstConflict(*node, conflict));
  EXPECT_EQ(conflict.robot_a, "robot_1");
  EXPECT_EQ(conflict.robot_b, "robot_2");
  EXPECT_EQ(conflict.t_start_idx, 1);
  EXPECT_EQ(conflict.t_end_idx, 2);
}

TEST(ConflictBasedSearchConflictTest,
     BuildConstraintFromConflictAccumulatesInheritedConstraints) {
  const auto cbs = makeCbs();
  auto node = makeSearchNode({"robot_1", "robot_2"});
  node->robot_plan_map["robot_1"] = makePlan({
      makeState(0.0, -1.0, 0.0, 0.3, 0.0),
      makeState(1.0, 0.0, 0.0, 0.3, 0.0),
      makeState(2.0, 1.0, 0.0, 0.3, 0.0),
  });
  node->robot_plan_map["robot_2"] = makePlan({
      makeState(0.0, 5.0, 0.0, 0.3, 0.0),
      makeState(2.0, 7.0, 2.0, 0.5, 0.0),
  });
  node->constraints["robot_1"].length = 0.7;
  node->constraints["robot_1"].width = 0.35;
  node->constraints["robot_1"].height = 0.35;
  node->constraints["robot_1"].pos_x.push_back(-10.0);
  node->constraints["robot_1"].pos_y.push_back(-10.0);
  node->constraints["robot_1"].pos_z.push_back(0.0);
  node->constraints["robot_1"].yaw.push_back(0.0);
  node->constraints["robot_1"].t_start.push_back(0.0);
  node->constraints["robot_1"].t_end.push_back(0.0);

  conflict_based_search::Conflict conflict;
  conflict.robot_a = "robot_1";
  conflict.robot_b = "robot_2";
  conflict.t_start_idx = 1;
  conflict.t_end_idx = 2;

  const auto constraints =
      cbs->buildConstraintFromConflict(*node, conflict, "robot_1");

  ASSERT_EQ(constraints.pos_x.size(), 3u);
  EXPECT_DOUBLE_EQ(constraints.pos_x[0], -10.0);
  EXPECT_NEAR(constraints.pos_x[1], 6.0, kTol);
  EXPECT_NEAR(constraints.pos_y[1], 1.0, kTol);
  EXPECT_NEAR(constraints.pos_z[1], 0.4, kTol);
  EXPECT_NEAR(constraints.t_start[1], 1.0, kTol);
  EXPECT_NEAR(constraints.t_end[2], 2.0, kTol);
  EXPECT_DOUBLE_EQ(constraints.length, 0.7);
  EXPECT_DOUBLE_EQ(constraints.width, 0.35);
  EXPECT_DOUBLE_EQ(constraints.height, 0.35);
}

TEST(ConflictBasedSearchConflictTest,
     BuildConstraintFromConflictCanConstrainRobotBAndSkipBadIndices) {
  const auto cbs = makeCbs();
  auto node = makeSearchNode({"robot_1", "robot_2"});
  node->robot_plan_map["robot_1"] = makePlan({
      makeState(0.0, 1.0, 0.0, 0.3, 0.0),
      makeState(1.0, 2.0, 0.0, 0.3, 0.0),
  });
  node->robot_plan_map["robot_2"] = makePlan({
      makeState(0.0, 5.0, 0.0, 0.3, 0.0),
      makeState(1.0, 6.0, 0.0, 0.3, 0.0),
  });
  node->constraints["robot_2"].length = 0.7;
  node->constraints["robot_2"].pos_x.push_back(-5.0);
  node->constraints["robot_2"].pos_y.push_back(-5.0);
  node->constraints["robot_2"].pos_z.push_back(0.0);
  node->constraints["robot_2"].yaw.push_back(0.0);
  node->constraints["robot_2"].t_start.push_back(0.0);
  node->constraints["robot_2"].t_end.push_back(0.0);

  conflict_based_search::Conflict conflict;
  conflict.robot_a = "robot_1";
  conflict.robot_b = "robot_2";
  conflict.t_start_idx = -1;
  conflict.t_end_idx = 3;

  const auto constraints =
      cbs->buildConstraintFromConflict(*node, conflict, "robot_2");

  ASSERT_EQ(constraints.pos_x.size(), 3u);
  EXPECT_DOUBLE_EQ(constraints.pos_x[0], -5.0);
  EXPECT_NEAR(constraints.pos_x[1], 1.0, kTol);
  EXPECT_NEAR(constraints.pos_x[2], 2.0, kTol);
  EXPECT_NEAR(constraints.t_start[1], 0.0, kTol);
  EXPECT_NEAR(constraints.t_start[2], 1.0, kTol);
  EXPECT_DOUBLE_EQ(constraints.length, 0.7);
}
