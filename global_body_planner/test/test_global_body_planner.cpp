#include <gtest/gtest.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include "global_body_planner/gbpl.hpp"
#include "global_body_planner/global_body_planner.hpp"
#include "global_body_planner/global_body_planner_test_fixture.hpp"

namespace {

constexpr double kTol = 1e-6;

quad_msgs::msg::RobotState makeRobotState(double x, double y, double z,
                                          double vx = 0.0,
                                          double vy = 0.0,
                                          double vz = 0.0) {
  quad_msgs::msg::RobotState state;
  state.body.pose.position.x = x;
  state.body.pose.position.y = y;
  state.body.pose.position.z = z;
  state.body.pose.orientation.w = 1.0;
  state.body.twist.linear.x = vx;
  state.body.twist.linear.y = vy;
  state.body.twist.linear.z = vz;
  return state;
}

grid_map_msgs::msg::GridMap makeTerrainMsg(double height = 0.0) {
  grid_map::GridMap map({"z_inpainted", "z_smooth", "normal_vectors_x",
                         "normal_vectors_y", "normal_vectors_z",
                         "smooth_normal_vectors_x", "smooth_normal_vectors_y",
                         "smooth_normal_vectors_z", "traversability"});
  map.setGeometry(grid_map::Length(10.0, 10.0), 0.05);
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    map.at("z_inpainted", *it) = height;
    map.at("z_smooth", *it) = height;
    map.at("normal_vectors_x", *it) = 0.0;
    map.at("normal_vectors_y", *it) = 0.0;
    map.at("normal_vectors_z", *it) = 1.0;
    map.at("smooth_normal_vectors_x", *it) = 0.0;
    map.at("smooth_normal_vectors_y", *it) = 0.0;
    map.at("smooth_normal_vectors_z", *it) = 1.0;
    map.at("traversability", *it) = 1.0;
  }

  return *grid_map::GridMapRosConverter::toMessage(map);
}

planning_utils::State makeState(double x, double y, double z, double vx,
                                double vy, double vz) {
  planning_utils::State s;
  s.pos << x, y, z;
  s.vel << vx, vy, vz;
  return s;
}

bool spinUntilPlan(const std::shared_ptr<rclcpp::Node>& node,
                   const std::shared_ptr<rclcpp::Node>& listener,
                   const std::optional<quad_msgs::msg::RobotPlan>& plan,
                   std::chrono::milliseconds timeout =
                       std::chrono::seconds(1)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!plan && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return plan.has_value();
}

bool spinUntilBool(const std::shared_ptr<rclcpp::Node>& node,
                   const std::shared_ptr<rclcpp::Node>& listener,
                   const std::optional<std_msgs::msg::Bool>& msg,
                   std::chrono::milliseconds timeout =
                       std::chrono::seconds(1)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return msg.has_value();
}

}  // namespace

TEST(GlobalBodyPlannerNodeTest, ConstructsWithYamlConfiguration) {
  auto node = std::make_shared<rclcpp::Node>(
      "global_body_planner",
      global_body_planner_test::plannerNodeOptions(true));

  EXPECT_NO_THROW({
    GlobalBodyPlanner global_body_planner(node);
  });
}

TEST(GlobalBodyPlannerNodeTest, TerrainStateAndGoalCallbacksUpdatePlannerData) {
  auto node = std::make_shared<rclcpp::Node>(
      "global_body_planner",
      global_body_planner_test::plannerNodeOptions(true));
  GlobalBodyPlanner planner(node);

  auto terrain_msg = std::make_shared<grid_map_msgs::msg::GridMap>(
      makeTerrainMsg(1.25));
  planner.terrainMapCallback(terrain_msg);

  EXPECT_TRUE(planner.map_recieved_);
  EXPECT_NEAR(planner.goal_state_.pos.z(), 1.25 + planner.planner_config_.h_nom,
              kTol);

  auto state_msg = std::make_shared<quad_msgs::msg::RobotState>(
      makeRobotState(0.2, -0.3, 1.55, 0.4, 0.0, 0.0));
  planner.robotStateCallback(state_msg);

  EXPECT_NEAR(planner.robot_state_.pos.x(), 0.2, kTol);
  EXPECT_NEAR(planner.robot_state_.pos.y(), -0.3, kTol);
  EXPECT_NEAR(planner.robot_state_.pos.z(), 1.55, kTol);
  EXPECT_NEAR(planner.robot_state_.vel.x(), 0.4, kTol);

  auto goal_msg = std::make_shared<geometry_msgs::msg::PointStamped>();
  goal_msg->header.stamp = node->now();
  goal_msg->point.x = 2.0;
  goal_msg->point.y = 0.5;
  planner.current_plan_.invalidate();
  planner.goalStateCallback(goal_msg);

  EXPECT_NEAR(planner.goal_state_.pos.x(), 2.0, kTol);
  EXPECT_NEAR(planner.goal_state_.pos.y(), 0.5, kTol);
  EXPECT_NEAR(planner.goal_state_.pos.z(), 1.25 + planner.planner_config_.h_nom,
              kTol);
}

TEST(GlobalBodyPlannerNodeTest, DuplicateGoalStampIsIgnored) {
  auto node = std::make_shared<rclcpp::Node>(
      "global_body_planner",
      global_body_planner_test::plannerNodeOptions(true));
  GlobalBodyPlanner planner(node);
  planner.terrainMapCallback(
      std::make_shared<grid_map_msgs::msg::GridMap>(makeTerrainMsg()));

  auto goal_msg = std::make_shared<geometry_msgs::msg::PointStamped>();
  goal_msg->header.stamp = node->now();
  goal_msg->point.x = 1.0;
  goal_msg->point.y = 2.0;
  planner.goalStateCallback(goal_msg);

  auto duplicate = std::make_shared<geometry_msgs::msg::PointStamped>();
  duplicate->header.stamp = goal_msg->header.stamp;
  duplicate->point.x = 4.0;
  duplicate->point.y = 5.0;
  planner.goalStateCallback(duplicate);

  EXPECT_NEAR(planner.goal_state_.pos.x(), 1.0, kTol);
  EXPECT_NEAR(planner.goal_state_.pos.y(), 2.0, kTol);
}

TEST(GlobalBodyPlannerNodeTest, PublishCurrentPlanPublishesBothPlanTopics) {
  auto node = std::make_shared<rclcpp::Node>(
      "global_body_planner",
      global_body_planner_test::plannerNodeOptions(true));
  auto listener = std::make_shared<rclcpp::Node>("global_body_plan_listener");
  GlobalBodyPlanner planner(node);
  planner.terrainMapCallback(
      std::make_shared<grid_map_msgs::msg::GridMap>(makeTerrainMsg()));

  std::optional<quad_msgs::msg::RobotPlan> global_plan;
  std::optional<quad_msgs::msg::RobotPlan> discrete_plan;
  auto plan_sub = listener->create_subscription<quad_msgs::msg::RobotPlan>(
      "global_plan", 10,
      [&](const quad_msgs::msg::RobotPlan::SharedPtr msg) {
        global_plan = *msg;
      });
  auto discrete_sub = listener->create_subscription<quad_msgs::msg::RobotPlan>(
      "global_plan_discrete", 10,
      [&](const quad_msgs::msg::RobotPlan::SharedPtr msg) {
        discrete_plan = *msg;
      });

  planning_utils::State s1 = makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  planning_utils::State s2 = makeState(1.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  planning_utils::StateActionResult result;
  GBPL gbpl;
  ASSERT_EQ(gbpl.attemptConnect(s1, s2, 2.0, result,
                                planner.planner_config_, FORWARD),
            REACHED);

  std::vector<planning_utils::State> states{s1, s2};
  std::vector<planning_utils::Action> actions{result.a_new};
  planning_utils::FullState start_state =
      planning_utils::stateToFullState(s1, 0, 0, 0, 0, 0, 0);
  planner.current_plan_.loadPlanData(VALID, start_state, 0.0, states, actions,
                                     planner.dt_, 0.0,
                                     planner.planner_config_);
  planner.reset_publish_delay_ = 0.0;
  planner.publish_after_reset_delay_ = true;
  planner.reset_time_ = node->now() - rclcpp::Duration::from_seconds(1.0);

  planner.publishCurrentPlan();

  ASSERT_TRUE(spinUntilPlan(node, listener, global_plan));
  ASSERT_TRUE(spinUntilPlan(node, listener, discrete_plan));
  EXPECT_EQ(global_plan->header.frame_id, planner.map_frame_);
  EXPECT_EQ(discrete_plan->header.frame_id, planner.map_frame_);
  EXPECT_EQ(global_plan->global_plan_timestamp,
            planner.current_plan_.getPublishedTimestamp());
  EXPECT_EQ(planner.planner_status_, 1);
  EXPECT_FALSE(planner.publish_after_reset_delay_);
}

TEST(GlobalBodyPlannerNodeTest,
     PlanWithConstraintsClearsServiceScopeConstraintsAndDoesNotPublish) {
  auto node = std::make_shared<rclcpp::Node>(
      "global_body_planner",
      global_body_planner_test::plannerNodeOptions(true));
  auto listener =
      std::make_shared<rclcpp::Node>("global_body_service_listener");
  GlobalBodyPlanner planner(node);
  planner.terrainMapCallback(
      std::make_shared<grid_map_msgs::msg::GridMap>(makeTerrainMsg()));
  planner.robotStateCallback(std::make_shared<quad_msgs::msg::RobotState>(
      makeRobotState(20.0, 20.0, 0.3)));

  std::optional<quad_msgs::msg::RobotPlan> published_plan;
  auto plan_sub = listener->create_subscription<quad_msgs::msg::RobotPlan>(
      "global_plan", 10,
      [&](const quad_msgs::msg::RobotPlan::SharedPtr msg) {
        published_plan = *msg;
      });

  auto request =
      std::make_shared<quad_msgs::srv::PlanWithConstraints::Request>();
  request->warm_start = true;
  request->constraints.length = 0.7;
  request->constraints.width = 0.35;
  request->constraints.height = 0.35;
  request->constraints.pos_x = {0.0, 1.0};
  request->constraints.pos_y = {0.0};
  request->constraints.pos_z = {0.3};
  request->constraints.yaw = {0.0};
  request->constraints.t_start = {0.0};
  request->constraints.t_end = {1.0};
  auto response =
      std::make_shared<quad_msgs::srv::PlanWithConstraints::Response>();

  planner.planWithConstraintsCallback(request, response);

  EXPECT_TRUE(planner.cbs_mode_);
  EXPECT_FALSE(response->success);
  EXPECT_TRUE(planner.planner_config_.dynamic_constraints.empty());
  EXPECT_FALSE(spinUntilPlan(node, listener, published_plan,
                             std::chrono::milliseconds(100)));
}

TEST(GlobalBodyPlannerNodeTest, CallPlannerPublishesGoalReachedWhenAtGoal) {
  auto node = std::make_shared<rclcpp::Node>(
      "global_body_planner",
      global_body_planner_test::plannerNodeOptions(true));
  auto listener = std::make_shared<rclcpp::Node>("goal_reached_listener");
  GlobalBodyPlanner planner(node);
  planner.terrainMapCallback(
      std::make_shared<grid_map_msgs::msg::GridMap>(makeTerrainMsg()));

  std::optional<std_msgs::msg::Bool> goal_reached;
  auto goal_sub = listener->create_subscription<std_msgs::msg::Bool>(
      "goal_reached", 10,
      [&](const std_msgs::msg::Bool::SharedPtr msg) { goal_reached = *msg; });

  planning_utils::FullState at_goal = planning_utils::stateToFullState(
      makeState(0.0, 0.0, planner.planner_config_.h_nom, 0.0, 0.0, 0.0), 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0);
  planner.start_state_ = at_goal;
  planner.goal_state_ = at_goal;

  EXPECT_FALSE(planner.callPlanner());

  ASSERT_TRUE(spinUntilBool(node, listener, goal_reached));
  EXPECT_TRUE(goal_reached->data);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
