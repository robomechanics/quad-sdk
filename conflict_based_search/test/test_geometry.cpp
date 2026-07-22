#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
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

std::shared_ptr<conflict_based_search::ConflictBasedSearch> makeCbs() {
  auto node = std::make_shared<rclcpp::Node>(
      "conflict_based_search", cbsNodeOptions());
  return std::make_shared<conflict_based_search::ConflictBasedSearch>(node);
}

}  // namespace

TEST(ConflictBasedSearchGeometryTest, PoseFromStateExtractsPositionYawAndTime) {
  const auto state = makeState(2.5, 1.0, -2.0, 0.4, M_PI / 2.0);

  const auto pose = conflict_based_search::ConflictBasedSearch::poseFromState(
      state);

  EXPECT_NEAR(pose.pos.x(), 1.0, kTol);
  EXPECT_NEAR(pose.pos.y(), -2.0, kTol);
  EXPECT_NEAR(pose.pos.z(), 0.4, kTol);
  EXPECT_NEAR(pose.yaw, M_PI / 2.0, kTol);
  EXPECT_NEAR(pose.t, 2.5, kTol);
}

TEST(ConflictBasedSearchGeometryTest, SamplePoseAtTimeInterpolatesAndClamps) {
  const auto plan = makePlan({
      makeState(1.0, 0.0, 0.0, 0.3, 0.0),
      makeState(3.0, 2.0, 2.0, 0.5, M_PI / 2.0),
  });

  const auto before =
      conflict_based_search::ConflictBasedSearch::samplePoseAtTime(plan, 0.0);
  const auto middle =
      conflict_based_search::ConflictBasedSearch::samplePoseAtTime(plan, 2.0);
  const auto after =
      conflict_based_search::ConflictBasedSearch::samplePoseAtTime(plan, 4.0);

  EXPECT_NEAR(before.pos.x(), 0.0, kTol);
  EXPECT_NEAR(middle.pos.x(), 1.0, kTol);
  EXPECT_NEAR(middle.pos.y(), 1.0, kTol);
  EXPECT_NEAR(middle.pos.z(), 0.4, kTol);
  EXPECT_NEAR(middle.yaw, M_PI / 4.0, kTol);
  EXPECT_NEAR(after.pos.x(), 2.0, kTol);
}

TEST(ConflictBasedSearchGeometryTest, SamplePoseHandlesEmptyPlanAndYawWrap) {
  const auto empty =
      conflict_based_search::ConflictBasedSearch::samplePoseAtTime(
          quad_msgs::msg::RobotPlan(), 4.5);

  EXPECT_TRUE(empty.pos.isZero(kTol));
  EXPECT_NEAR(empty.yaw, 0.0, kTol);
  EXPECT_NEAR(empty.t, 4.5, kTol);

  const auto wrapped = makePlan({
      makeState(0.0, 0.0, 0.0, 0.3, 3.0 * M_PI / 4.0),
      makeState(2.0, 0.0, 0.0, 0.3, -3.0 * M_PI / 4.0),
  });
  const auto middle =
      conflict_based_search::ConflictBasedSearch::samplePoseAtTime(wrapped,
                                                                   1.0);

  EXPECT_NEAR(std::abs(middle.yaw), M_PI, kTol);
}

TEST(ConflictBasedSearchGeometryTest, ObbsOverlapHandlesPlanarAndHeightGaps) {
  const auto cbs = makeCbs();
  const Eigen::Vector3d he(0.5, 0.25, 0.2);

  conflict_based_search::ConflictBasedSearch::BodyPose a;
  a.pos = Eigen::Vector3d(0.0, 0.0, 0.3);
  a.yaw = 0.0;
  a.t = 0.0;

  auto b = a;
  b.pos = Eigen::Vector3d(0.5, 0.0, 0.3);
  EXPECT_TRUE(cbs->obbsOverlap(a, he, b, he));

  b.pos = Eigen::Vector3d(2.0, 0.0, 0.3);
  EXPECT_FALSE(cbs->obbsOverlap(a, he, b, he));

  b.pos = Eigen::Vector3d(0.0, 0.0, 1.0);
  EXPECT_FALSE(cbs->obbsOverlap(a, he, b, he));
}

TEST(ConflictBasedSearchGeometryTest, ObbsOverlapTreatsTouchingAsOverlap) {
  const auto cbs = makeCbs();
  const Eigen::Vector3d he(0.5, 0.25, 0.2);

  conflict_based_search::ConflictBasedSearch::BodyPose a;
  a.pos = Eigen::Vector3d(0.0, 0.0, 0.3);
  a.yaw = 0.0;
  a.t = 0.0;

  auto b = a;
  b.pos = Eigen::Vector3d(1.0, 0.0, 0.3);

  EXPECT_TRUE(cbs->obbsOverlap(a, he, b, he));
}

TEST(ConflictBasedSearchGeometryTest, ObbsOverlapHandlesRotatedBoxes) {
  const auto cbs = makeCbs();
  const Eigen::Vector3d long_box(1.0, 0.2, 0.2);
  const Eigen::Vector3d short_box(0.35, 0.15, 0.2);

  conflict_based_search::ConflictBasedSearch::BodyPose a;
  a.pos = Eigen::Vector3d(0.0, 0.0, 0.3);
  a.yaw = M_PI / 4.0;
  a.t = 0.0;

  auto b = a;
  b.pos = Eigen::Vector3d(0.5, 0.0, 0.3);
  b.yaw = -M_PI / 4.0;
  EXPECT_TRUE(cbs->obbsOverlap(a, long_box, b, short_box));

  b.pos = Eigen::Vector3d(1.8, 0.0, 0.3);
  EXPECT_FALSE(cbs->obbsOverlap(a, long_box, b, short_box));
}
