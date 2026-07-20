#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "conflict_based_search/conflict_based_search.hpp"

namespace {

using PlanSrv = quad_msgs::srv::PlanWithConstraints;

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

std::shared_ptr<rclcpp::Node> makeNode(const std::string& name) {
  (void)name;
  return std::make_shared<rclcpp::Node>("conflict_based_search",
                                        cbsNodeOptions());
}

quad_msgs::msg::RobotPlan oneStatePlan(const std::string& robot, double t) {
  const double y = (robot == "robot_1") ? 0.0 : 2.0;
  return makePlan({makeState(t, 0.0, y, 0.3, 0.0)});
}

}  // namespace

TEST(ConflictBasedSearchServiceTest, WaitForServicesReportsAvailability) {
  auto node = makeNode("conflict_based_search_wait_test");
  conflict_based_search::ConflictBasedSearch cbs(node);

  auto srv1 = node->create_service<PlanSrv>(
      "/robot_1/plan_with_constraints",
      [](const std::shared_ptr<PlanSrv::Request>,
         std::shared_ptr<PlanSrv::Response> response) {
        response->success = true;
      });
  auto srv2 = node->create_service<PlanSrv>(
      "/robot_2/plan_with_constraints",
      [](const std::shared_ptr<PlanSrv::Request>,
         std::shared_ptr<PlanSrv::Response> response) {
        response->success = true;
      });

  cbs.createServiceClients();

  EXPECT_TRUE(cbs.waitForServices(std::chrono::seconds(0)));
}

TEST(ConflictBasedSearchServiceTest, CallPlanWithConstraintsReturnsPlan) {
  auto node = makeNode("conflict_based_search_call_test");
  conflict_based_search::ConflictBasedSearch cbs(node);

  bool saw_warm_start = false;
  double saw_length = 0.0;
  auto srv = node->create_service<PlanSrv>(
      "/robot_1/plan_with_constraints",
      [&](const std::shared_ptr<PlanSrv::Request> request,
          std::shared_ptr<PlanSrv::Response> response) {
        saw_warm_start = request->warm_start;
        saw_length = request->constraints.length;
        response->plan = oneStatePlan("robot_1", 0.0);
        response->path_length = 4.2;
        response->success = true;
      });

  cbs.createServiceClients();

  quad_msgs::msg::RobotPlanConstraints constraints;
  constraints.length = 0.7;
  quad_msgs::msg::RobotPlan plan;
  double length = 0.0;

  ASSERT_TRUE(cbs.callPlanWithConstraints("robot_1", constraints, true, plan,
                                          length));
  EXPECT_TRUE(saw_warm_start);
  EXPECT_DOUBLE_EQ(saw_length, 0.7);
  EXPECT_DOUBLE_EQ(length, 4.2);
  ASSERT_EQ(plan.states.size(), 1u);
}

TEST(ConflictBasedSearchServiceTest, CallPlanWithConstraintsHandlesFailure) {
  auto node = makeNode("conflict_based_search_failure_test");
  conflict_based_search::ConflictBasedSearch cbs(node);

  auto srv = node->create_service<PlanSrv>(
      "/robot_1/plan_with_constraints",
      [](const std::shared_ptr<PlanSrv::Request>,
         std::shared_ptr<PlanSrv::Response> response) {
        response->path_length = 0.0;
        response->success = false;
      });

  cbs.createServiceClients();

  quad_msgs::msg::RobotPlanConstraints constraints;
  quad_msgs::msg::RobotPlan plan;
  double length = 0.0;

  EXPECT_FALSE(cbs.callPlanWithConstraints("robot_1", constraints, false, plan,
                                           length));
}

TEST(ConflictBasedSearchServiceTest, InitialPlansSeedConstraintsAndCosts) {
  auto node = makeNode("conflict_based_search_initial_test");
  conflict_based_search::ConflictBasedSearch cbs(node);

  auto srv1 = node->create_service<PlanSrv>(
      "/robot_1/plan_with_constraints",
      [](const std::shared_ptr<PlanSrv::Request> request,
         std::shared_ptr<PlanSrv::Response> response) {
        EXPECT_FALSE(request->warm_start);
        EXPECT_DOUBLE_EQ(request->constraints.length, 0.7);
        response->plan = oneStatePlan("robot_1", 0.0);
        response->path_length = 1.0;
        response->success = true;
      });
  auto srv2 = node->create_service<PlanSrv>(
      "/robot_2/plan_with_constraints",
      [](const std::shared_ptr<PlanSrv::Request> request,
         std::shared_ptr<PlanSrv::Response> response) {
        EXPECT_FALSE(request->warm_start);
        EXPECT_DOUBLE_EQ(request->constraints.width, 0.35);
        response->plan = oneStatePlan("robot_2", 0.0);
        response->path_length = 2.0;
        response->success = true;
      });

  cbs.createServiceClients();

  conflict_based_search::CBSNode root;
  root.robot_names = {"robot_1", "robot_2"};

  ASSERT_TRUE(cbs.requestInitialPlans(root));
  EXPECT_DOUBLE_EQ(root.cost, 3.0);
  EXPECT_EQ(root.robot_plan_map.size(), 2u);
  EXPECT_DOUBLE_EQ(root.constraints["robot_1"].height, 0.35);
  EXPECT_DOUBLE_EQ(root.constraints["robot_2"].length, 0.7);
}

TEST(ConflictBasedSearchServiceTest, ReplanUpdatesOnlyRequestedRobot) {
  auto node = makeNode("conflict_based_search_replan_test");
  conflict_based_search::ConflictBasedSearch cbs(node);

  auto srv1 = node->create_service<PlanSrv>(
      "/robot_1/plan_with_constraints",
      [](const std::shared_ptr<PlanSrv::Request> request,
         std::shared_ptr<PlanSrv::Response> response) {
        EXPECT_TRUE(request->warm_start);
        response->plan = oneStatePlan("robot_1", 10.0);
        response->path_length = 5.0;
        response->success = true;
      });

  cbs.createServiceClients();

  conflict_based_search::CBSNode child;
  child.robot_names = {"robot_1", "robot_2"};
  child.robot_plan_map["robot_2"] = oneStatePlan("robot_2", 0.0);
  child.cost_map["robot_2"] = 2.0;
  child.constraints["robot_1"] = quad_msgs::msg::RobotPlanConstraints();

  ASSERT_TRUE(cbs.requestReplan(child, "robot_1"));
  EXPECT_DOUBLE_EQ(child.cost, 7.0);
  EXPECT_EQ(child.robot_plan_map["robot_1"].states.front().header.stamp.sec,
            10);
  EXPECT_EQ(child.robot_plan_map["robot_2"].states.front().header.stamp.sec, 0);
}

TEST(ConflictBasedSearchPublishTest, PublishPlansRebasesTimestamps) {
  auto node = makeNode("conflict_based_search_publish_test");
  conflict_based_search::ConflictBasedSearch cbs(node);
  auto listener = std::make_shared<rclcpp::Node>("cbs_plan_listener");

  std::optional<quad_msgs::msg::RobotPlan> received;
  auto sub = listener->create_subscription<quad_msgs::msg::RobotPlan>(
      "/robot_1/global_plan", 10,
      [&](const quad_msgs::msg::RobotPlan::SharedPtr msg) { received = *msg; });

  conflict_based_search::CBSNode solution;
  solution.robot_names = {"robot_1"};
  solution.robot_plan_map["robot_1"] = makePlan({
      makeState(5.0, 0.0, 0.0, 0.3, 0.0),
      makeState(6.0, 1.0, 0.0, 0.3, 0.0),
  });

  cbs.publishPlans(solution);

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(1);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(listener);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(received.has_value());
  ASSERT_EQ(received->states.size(), 2u);
  const rclcpp::Time first(received->states[0].header.stamp);
  const rclcpp::Time second(received->states[1].header.stamp);
  EXPECT_NEAR((second - first).seconds(), 1.0, 1e-6);
  EXPECT_EQ(received->global_plan_timestamp, received->header.stamp);
}
