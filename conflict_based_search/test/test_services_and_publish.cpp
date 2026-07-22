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

bool spinUntilPlan(const rclcpp::Node::SharedPtr& listener,
                   const rclcpp::Node::SharedPtr& cbs_node,
                   const std::optional<quad_msgs::msg::RobotPlan>& plan,
                   std::chrono::milliseconds timeout =
                       std::chrono::milliseconds(1000)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!plan && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(listener);
    rclcpp::spin_some(cbs_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return plan.has_value();
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

TEST(ConflictBasedSearchServiceTest, WaitForServicesReportsMissingService) {
  auto node = makeNode("conflict_based_search_wait_missing_test");
  conflict_based_search::ConflictBasedSearch cbs(node);

  auto srv1 = node->create_service<PlanSrv>(
      "/robot_1/plan_with_constraints",
      [](const std::shared_ptr<PlanSrv::Request>,
         std::shared_ptr<PlanSrv::Response> response) {
        response->success = true;
      });

  cbs.createServiceClients();

  EXPECT_FALSE(cbs.waitForServices(std::chrono::seconds(0)));
}

TEST(ConflictBasedSearchServiceTest, CustomRobotNamesCreateMatchingClients) {
  rclcpp::NodeOptions options = cbsNodeOptions();
  options.append_parameter_override(
      "robot_names", std::vector<std::string>{"alpha", "bravo", "charlie"});
  auto node = std::make_shared<rclcpp::Node>("conflict_based_search", options);
  conflict_based_search::ConflictBasedSearch cbs(node);

  auto srv_alpha = node->create_service<PlanSrv>(
      "/alpha/plan_with_constraints",
      [](const std::shared_ptr<PlanSrv::Request>,
         std::shared_ptr<PlanSrv::Response> response) {
        response->success = true;
      });
  auto srv_bravo = node->create_service<PlanSrv>(
      "/bravo/plan_with_constraints",
      [](const std::shared_ptr<PlanSrv::Request>,
         std::shared_ptr<PlanSrv::Response> response) {
        response->success = true;
      });
  auto srv_charlie = node->create_service<PlanSrv>(
      "/charlie/plan_with_constraints",
      [](const std::shared_ptr<PlanSrv::Request>,
         std::shared_ptr<PlanSrv::Response> response) {
        response->success = true;
      });

  cbs.createServiceClients();

  EXPECT_EQ(cbs.robot_names_,
            (std::vector<std::string>{"alpha", "bravo", "charlie"}));
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

TEST(ConflictBasedSearchServiceTest, CallPlanWithConstraintsTimesOut) {
  rclcpp::NodeOptions options = cbsNodeOptions();
  options.append_parameter_override("service_timeout_s", 0.01);
  auto node = std::make_shared<rclcpp::Node>("conflict_based_search", options);
  conflict_based_search::ConflictBasedSearch cbs(node);
  cbs.createServiceClients();

  quad_msgs::msg::RobotPlanConstraints constraints;
  quad_msgs::msg::RobotPlan plan;
  double length = 0.0;

  EXPECT_FALSE(cbs.callPlanWithConstraints("robot_1", constraints, false, plan,
                                           length));
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

TEST(ConflictBasedSearchServiceTest, InitialPlansRetryThenSucceed) {
  auto node = makeNode("conflict_based_search_initial_retry_test");
  conflict_based_search::ConflictBasedSearch cbs(node);

  int robot_1_calls = 0;
  auto srv1 = node->create_service<PlanSrv>(
      "/robot_1/plan_with_constraints",
      [&](const std::shared_ptr<PlanSrv::Request>,
          std::shared_ptr<PlanSrv::Response> response) {
        ++robot_1_calls;
        response->success = robot_1_calls >= 3;
        response->plan = oneStatePlan("robot_1", 0.0);
        response->path_length = 1.0;
      });
  auto srv2 = node->create_service<PlanSrv>(
      "/robot_2/plan_with_constraints",
      [](const std::shared_ptr<PlanSrv::Request>,
         std::shared_ptr<PlanSrv::Response> response) {
        response->success = true;
        response->plan = oneStatePlan("robot_2", 0.0);
        response->path_length = 2.0;
      });

  cbs.createServiceClients();

  conflict_based_search::CBSNode root;
  root.robot_names = {"robot_1", "robot_2"};

  ASSERT_TRUE(cbs.requestInitialPlans(root));
  EXPECT_EQ(robot_1_calls, 3);
  EXPECT_DOUBLE_EQ(root.cost, 3.0);
}

TEST(ConflictBasedSearchServiceTest, InitialPlansFailAfterRetryExhaustion) {
  auto node = makeNode("conflict_based_search_initial_exhaustion_test");
  conflict_based_search::ConflictBasedSearch cbs(node);

  int calls = 0;
  auto srv = node->create_service<PlanSrv>(
      "/robot_1/plan_with_constraints",
      [&](const std::shared_ptr<PlanSrv::Request>,
          std::shared_ptr<PlanSrv::Response> response) {
        ++calls;
        response->success = false;
      });

  cbs.createServiceClients();

  conflict_based_search::CBSNode root;
  root.robot_names = {"robot_1"};

  EXPECT_FALSE(cbs.requestInitialPlans(root));
  EXPECT_EQ(calls, 5);
  EXPECT_TRUE(root.robot_plan_map.empty());
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

TEST(ConflictBasedSearchServiceTest,
     BodyDimensionOverridesSeedInitialConstraints) {
  rclcpp::NodeOptions options = cbsNodeOptions();
  options.append_parameter_override("robot_names",
                                    std::vector<std::string>{"robot_1"});
  options.append_parameter_override("body_length", 1.2);
  options.append_parameter_override("body_width", 0.8);
  options.append_parameter_override("body_height", 0.4);
  auto node = std::make_shared<rclcpp::Node>("conflict_based_search", options);
  conflict_based_search::ConflictBasedSearch cbs(node);

  auto srv1 = node->create_service<PlanSrv>(
      "/robot_1/plan_with_constraints",
      [](const std::shared_ptr<PlanSrv::Request> request,
         std::shared_ptr<PlanSrv::Response> response) {
        EXPECT_DOUBLE_EQ(request->constraints.length, 1.2);
        EXPECT_DOUBLE_EQ(request->constraints.width, 0.8);
        EXPECT_DOUBLE_EQ(request->constraints.height, 0.4);
        response->plan = oneStatePlan("robot_1", 0.0);
        response->path_length = 1.0;
        response->success = true;
      });

  cbs.createServiceClients();

  conflict_based_search::CBSNode root;
  root.robot_names = {"robot_1"};

  ASSERT_TRUE(cbs.requestInitialPlans(root));
  EXPECT_DOUBLE_EQ(root.constraints["robot_1"].length, 1.2);
  EXPECT_DOUBLE_EQ(root.constraints["robot_1"].width, 0.8);
  EXPECT_DOUBLE_EQ(root.constraints["robot_1"].height, 0.4);
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

TEST(ConflictBasedSearchServiceTest, ReplanCanDisableWarmStart) {
  rclcpp::NodeOptions options = cbsNodeOptions();
  options.append_parameter_override("robot_names",
                                    std::vector<std::string>{"robot_1"});
  options.append_parameter_override("warm_start", false);
  auto node = std::make_shared<rclcpp::Node>("conflict_based_search", options);
  conflict_based_search::ConflictBasedSearch cbs(node);

  auto srv1 = node->create_service<PlanSrv>(
      "/robot_1/plan_with_constraints",
      [](const std::shared_ptr<PlanSrv::Request> request,
         std::shared_ptr<PlanSrv::Response> response) {
        EXPECT_FALSE(request->warm_start);
        response->plan = oneStatePlan("robot_1", 0.0);
        response->path_length = 3.0;
        response->success = true;
      });

  cbs.createServiceClients();

  conflict_based_search::CBSNode child;
  child.robot_names = {"robot_1"};
  child.constraints["robot_1"] = quad_msgs::msg::RobotPlanConstraints();

  ASSERT_TRUE(cbs.requestReplan(child, "robot_1"));
  EXPECT_DOUBLE_EQ(child.cost, 3.0);
}

TEST(ConflictBasedSearchServiceTest, ReplanFailurePreservesExistingPlan) {
  auto node = makeNode("conflict_based_search_replan_failure_test");
  conflict_based_search::ConflictBasedSearch cbs(node);

  auto srv1 = node->create_service<PlanSrv>(
      "/robot_1/plan_with_constraints",
      [](const std::shared_ptr<PlanSrv::Request>,
         std::shared_ptr<PlanSrv::Response> response) {
        response->success = false;
      });

  cbs.createServiceClients();

  conflict_based_search::CBSNode child;
  child.robot_names = {"robot_1", "robot_2"};
  child.robot_plan_map["robot_1"] = oneStatePlan("robot_1", 0.0);
  child.cost_map["robot_1"] = 1.0;
  child.constraints["robot_1"] = quad_msgs::msg::RobotPlanConstraints();

  EXPECT_FALSE(cbs.requestReplan(child, "robot_1"));
  EXPECT_DOUBLE_EQ(child.cost_map["robot_1"], 1.0);
  EXPECT_EQ(child.robot_plan_map["robot_1"].states.front().header.stamp.sec, 0);
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

TEST(ConflictBasedSearchPublishTest, PublishPlansHandlesMultipleAndEmptyPlans) {
  auto node = makeNode("conflict_based_search_publish_multi_test");
  conflict_based_search::ConflictBasedSearch cbs(node);
  auto listener = std::make_shared<rclcpp::Node>("cbs_multi_plan_listener");

  std::optional<quad_msgs::msg::RobotPlan> robot_1_plan;
  std::optional<quad_msgs::msg::RobotPlan> robot_2_plan;
  auto sub1 = listener->create_subscription<quad_msgs::msg::RobotPlan>(
      "/robot_1/global_plan", 10,
      [&](const quad_msgs::msg::RobotPlan::SharedPtr msg) {
        robot_1_plan = *msg;
      });
  auto sub2 = listener->create_subscription<quad_msgs::msg::RobotPlan>(
      "/robot_2/global_plan", 10,
      [&](const quad_msgs::msg::RobotPlan::SharedPtr msg) {
        robot_2_plan = *msg;
      });

  conflict_based_search::CBSNode solution;
  solution.robot_names = {"robot_1", "robot_2"};
  solution.robot_plan_map["robot_1"] = oneStatePlan("robot_1", 5.0);
  solution.robot_plan_map["robot_2"] = quad_msgs::msg::RobotPlan();

  cbs.publishPlans(solution);

  EXPECT_TRUE(spinUntilPlan(listener, node, robot_1_plan));
  EXPECT_FALSE(spinUntilPlan(listener, node, robot_2_plan,
                             std::chrono::milliseconds(200)));
}

TEST(ConflictBasedSearchRunTest, RunPublishesConflictFreeInitialPlans) {
  auto node = makeNode("conflict_based_search_run_clear_test");
  conflict_based_search::ConflictBasedSearch cbs(node);
  auto listener = std::make_shared<rclcpp::Node>("cbs_run_clear_listener");

  int robot_1_calls = 0;
  int robot_2_calls = 0;
  auto srv1 = node->create_service<PlanSrv>(
      "/robot_1/plan_with_constraints",
      [&](const std::shared_ptr<PlanSrv::Request> request,
          std::shared_ptr<PlanSrv::Response> response) {
        ++robot_1_calls;
        EXPECT_FALSE(request->warm_start);
        response->success = true;
        response->plan = oneStatePlan("robot_1", 0.0);
        response->path_length = 1.0;
      });
  auto srv2 = node->create_service<PlanSrv>(
      "/robot_2/plan_with_constraints",
      [&](const std::shared_ptr<PlanSrv::Request> request,
          std::shared_ptr<PlanSrv::Response> response) {
        ++robot_2_calls;
        EXPECT_FALSE(request->warm_start);
        response->success = true;
        response->plan = oneStatePlan("robot_2", 0.0);
        response->path_length = 2.0;
      });

  std::optional<quad_msgs::msg::RobotPlan> robot_1_plan;
  std::optional<quad_msgs::msg::RobotPlan> robot_2_plan;
  auto sub1 = listener->create_subscription<quad_msgs::msg::RobotPlan>(
      "/robot_1/global_plan", 10,
      [&](const quad_msgs::msg::RobotPlan::SharedPtr msg) {
        robot_1_plan = *msg;
      });
  auto sub2 = listener->create_subscription<quad_msgs::msg::RobotPlan>(
      "/robot_2/global_plan", 10,
      [&](const quad_msgs::msg::RobotPlan::SharedPtr msg) {
        robot_2_plan = *msg;
      });

  cbs.run();

  EXPECT_TRUE(spinUntilPlan(listener, node, robot_1_plan));
  EXPECT_TRUE(spinUntilPlan(listener, node, robot_2_plan));
  EXPECT_EQ(robot_1_calls, 1);
  EXPECT_EQ(robot_2_calls, 1);
}

TEST(ConflictBasedSearchRunTest, RunBranchesAndPublishesResolvedChild) {
  auto node = makeNode("conflict_based_search_run_replan_test");
  conflict_based_search::ConflictBasedSearch cbs(node);
  auto listener = std::make_shared<rclcpp::Node>("cbs_run_replan_listener");

  int robot_1_calls = 0;
  int robot_2_calls = 0;
  bool saw_robot_1_constraint = false;
  bool saw_robot_2_constraint = false;
  auto srv1 = node->create_service<PlanSrv>(
      "/robot_1/plan_with_constraints",
      [&](const std::shared_ptr<PlanSrv::Request> request,
          std::shared_ptr<PlanSrv::Response> response) {
        ++robot_1_calls;
        saw_robot_1_constraint =
            saw_robot_1_constraint || !request->constraints.pos_x.empty();
        response->success = true;
        response->plan = (robot_1_calls == 1)
                             ? makePlan({makeState(0.0, 0.0, 0.0, 0.3, 0.0)})
                             : makePlan({makeState(0.0, 0.0, -2.0, 0.3, 0.0)});
        response->path_length = static_cast<double>(robot_1_calls);
      });
  auto srv2 = node->create_service<PlanSrv>(
      "/robot_2/plan_with_constraints",
      [&](const std::shared_ptr<PlanSrv::Request> request,
          std::shared_ptr<PlanSrv::Response> response) {
        ++robot_2_calls;
        saw_robot_2_constraint =
            saw_robot_2_constraint || !request->constraints.pos_x.empty();
        response->success = robot_2_calls == 1;
        response->plan = makePlan({makeState(0.0, 0.0, 0.0, 0.3, 0.0)});
        response->path_length = 1.0;
      });

  std::optional<quad_msgs::msg::RobotPlan> robot_1_plan;
  auto sub1 = listener->create_subscription<quad_msgs::msg::RobotPlan>(
      "/robot_1/global_plan", 10,
      [&](const quad_msgs::msg::RobotPlan::SharedPtr msg) {
        robot_1_plan = *msg;
      });

  cbs.run();

  ASSERT_TRUE(spinUntilPlan(listener, node, robot_1_plan));
  ASSERT_EQ(robot_1_plan->states.size(), 1u);
  EXPECT_NEAR(robot_1_plan->states.front().body.pose.position.y, -2.0, 1e-6);
  EXPECT_EQ(robot_1_calls, 2);
  EXPECT_EQ(robot_2_calls, 2);
  EXPECT_TRUE(saw_robot_1_constraint);
  EXPECT_TRUE(saw_robot_2_constraint);
}

TEST(ConflictBasedSearchRunTest, RunPublishesBestKnownWhenIterationCapHits) {
  rclcpp::NodeOptions options = cbsNodeOptions();
  options.append_parameter_override("max_iterations", 1);
  auto node = std::make_shared<rclcpp::Node>("conflict_based_search", options);
  conflict_based_search::ConflictBasedSearch cbs(node);
  auto listener = std::make_shared<rclcpp::Node>("cbs_run_cap_listener");

  int robot_1_calls = 0;
  int robot_2_calls = 0;
  auto colliding_plan =
      makePlan({makeState(0.0, 0.0, 0.0, 0.3, 0.0),
                makeState(1.0, 0.0, 0.0, 0.3, 0.0)});
  auto srv1 = node->create_service<PlanSrv>(
      "/robot_1/plan_with_constraints",
      [&](const std::shared_ptr<PlanSrv::Request>,
          std::shared_ptr<PlanSrv::Response> response) {
        ++robot_1_calls;
        response->success = true;
        response->plan = colliding_plan;
        response->path_length = 1.0;
      });
  auto srv2 = node->create_service<PlanSrv>(
      "/robot_2/plan_with_constraints",
      [&](const std::shared_ptr<PlanSrv::Request>,
          std::shared_ptr<PlanSrv::Response> response) {
        ++robot_2_calls;
        response->success = true;
        response->plan = colliding_plan;
        response->path_length = 1.0;
      });

  std::optional<quad_msgs::msg::RobotPlan> robot_1_plan;
  auto sub1 = listener->create_subscription<quad_msgs::msg::RobotPlan>(
      "/robot_1/global_plan", 10,
      [&](const quad_msgs::msg::RobotPlan::SharedPtr msg) {
        robot_1_plan = *msg;
      });

  cbs.run();

  EXPECT_TRUE(spinUntilPlan(listener, node, robot_1_plan));
  EXPECT_EQ(robot_1_calls, 2);
  EXPECT_EQ(robot_2_calls, 2);
}
