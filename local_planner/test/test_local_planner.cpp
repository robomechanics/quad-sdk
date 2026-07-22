#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <grid_map_ros/grid_map_ros.hpp>

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

grid_map::GridMap makeTerrain(double height = 0.0) {
  grid_map::GridMap map({"z_inpainted", "z_smooth", "normal_vectors_x",
                         "normal_vectors_y", "normal_vectors_z",
                         "smooth_normal_vectors_x", "smooth_normal_vectors_y",
                         "smooth_normal_vectors_z", "traversability"});
  map.setGeometry(grid_map::Length(4.0, 4.0), 0.1);
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
  return map;
}

quad_msgs::msg::RobotPlan makeRobotPlan(rclcpp::Time start_time,
                                        int num_states = 8) {
  quad_msgs::msg::RobotPlan plan;
  plan.header.frame_id = "map";
  plan.header.stamp = start_time;
  plan.global_plan_timestamp = start_time;
  for (int i = 0; i < num_states; ++i) {
    Eigen::VectorXd body = Eigen::VectorXd::Zero(12);
    body[0] = 0.1 * i;
    body[1] = -0.05 * i;
    body[2] = 0.30;
    body[5] = 0.02 * i;
    body[6] = 0.1;

    quad_msgs::msg::RobotState state;
    state.header.stamp = start_time + rclcpp::Duration::from_seconds(0.03 * i);
    state.body = quad_utils::eigenToBodyStateMsg(body);
    plan.states.push_back(state);
    plan.plan_indices.push_back(i);
    plan.primitive_ids.push_back(i % 4);
  }
  return plan;
}

grid_map_msgs::msg::GridMap::SharedPtr makeTerrainMsg(
    const grid_map::GridMap& terrain) {
  return grid_map_msgs::msg::GridMap::SharedPtr(
      grid_map::GridMapRosConverter::toMessage(terrain).release());
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

TEST(LocalPlannerTest, InitFootstepPlannerClampsInvalidGrfWeight) {
  auto high_node = makeNode(
      {rclcpp::Parameter("local_footstep_planner.grf_weight", 4.0)});
  LocalPlanner high_planner(high_node);
  EXPECT_DOUBLE_EQ(high_planner.local_footstep_planner_->grf_weight_, 1.0);

  auto low_node = makeNode(
      {rclcpp::Parameter("local_footstep_planner.grf_weight", -2.0)});
  LocalPlanner low_planner(low_node);
  EXPECT_DOUBLE_EQ(low_planner.local_footstep_planner_->grf_weight_, 0.0);
}

TEST(LocalPlannerTest, TerrainMapCallbackUpdatesPlannerAndFootstepMaps) {
  auto node = makeNode();
  LocalPlanner planner(node);
  const auto terrain = makeTerrain(0.23);
  auto msg = makeTerrainMsg(terrain);

  planner.terrainMapCallback(msg);

  EXPECT_FALSE(planner.terrain_.isEmpty());
  EXPECT_NEAR(planner.terrain_grid_.atPosition("z_smooth", {0.0, 0.0}), 0.23,
              kTol);
  EXPECT_NEAR(
      planner.local_footstep_planner_->terrain_grid_.atPosition("z_smooth",
                                                                {0.0, 0.0}),
      0.23, kTol);
  EXPECT_NEAR(planner.local_footstep_planner_->getTerrainHeight(0.0, 0.0),
              0.23, kTol);
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

TEST(LocalPlannerTest, GetReferenceFromGlobalPlanTracksIndexAndHoldsEnd) {
  auto node = makeNode();
  LocalPlanner planner(node);
  const auto terrain = makeTerrain();
  auto terrain_msg = makeTerrainMsg(terrain);
  planner.terrainMapCallback(terrain_msg);

  auto state = std::make_shared<quad_msgs::msg::RobotState>(makePopulatedState());
  state->body.pose.position.x = 0.20;
  state->body.pose.position.y = -0.10;
  planner.robotStateCallback(state);

  const rclcpp::Time start =
      node->now() - rclcpp::Duration::from_seconds(2.0 * planner.dt_ + 1e-4);
  auto plan = std::make_shared<quad_msgs::msg::RobotPlan>(makeRobotPlan(start, 4));
  planner.robotPlanCallback(plan);

  planner.getReference();

  EXPECT_EQ(planner.current_plan_index_, 2);
  EXPECT_EQ(planner.plan_index_diff_, 2);
  EXPECT_NEAR(planner.body_plan_(0, 0), 0.20, kTol);
  EXPECT_NEAR(planner.ref_body_plan_(0, 0), 0.20, kTol);
  EXPECT_NEAR(planner.ref_body_plan_(1, 0), 0.30, kTol);
  EXPECT_NEAR(planner.ref_body_plan_(2, 0), 0.30, kTol);
  EXPECT_EQ(planner.ref_primitive_plan_(0), 2);
  EXPECT_EQ(planner.ref_primitive_plan_(1), 3);
  EXPECT_EQ(planner.ref_primitive_plan_(2), 3);
}

TEST(LocalPlannerTest, GetReferenceFromTwistHandlesStepStandAndStaleCommand) {
  auto node = makeNode({rclcpp::Parameter("local_planner.use_twist_input", true)});
  LocalPlanner planner(node);
  const auto terrain = makeTerrain(0.12);
  auto terrain_msg = makeTerrainMsg(terrain);
  planner.terrainMapCallback(terrain_msg);

  auto state = std::make_shared<quad_msgs::msg::RobotState>(makePopulatedState());
  state->body.pose.position.x = 0.0;
  state->body.pose.position.y = 0.0;
  state->body.pose.position.z = 0.39;
  planner.robotStateCallback(state);

  auto twist = std::make_shared<geometry_msgs::msg::Twist>();
  twist->linear.x = 2.0;
  planner.cmdVelCallback(twist);
  planner.getReference();

  EXPECT_EQ(planner.control_mode_, STEP);
  EXPECT_GT(planner.ref_body_plan_(1, 0), planner.ref_body_plan_(0, 0));
  EXPECT_NEAR(planner.ref_body_plan_(0, 2), planner.z_des_ + 0.12, kTol);

  planner.cmd_vel_[0] = 0.4;
  planner.last_cmd_vel_msg_time_ =
      node->now() - rclcpp::Duration::from_seconds(
                        planner.last_cmd_vel_msg_time_max_ + 1.0);
  for (auto& foot : state->feet.feet) {
    foot.position.x = state->body.pose.position.x;
    foot.position.y = state->body.pose.position.y;
  }
  planner.robotStateCallback(state);
  planner.getReference();

  EXPECT_EQ(planner.control_mode_, STAND);
  EXPECT_NEAR(planner.cmd_vel_.norm(), 0.0, kTol);
}

TEST(LocalPlannerTest, PublishLocalPlanPublishesBodyFeetAndGrfs) {
  auto node = makeNode();
  LocalPlanner planner(node);
  std::shared_ptr<quad_msgs::msg::RobotPlan> local_plan_msg;
  std::shared_ptr<quad_msgs::msg::MultiFootPlanDiscrete> discrete_msg;
  std::shared_ptr<quad_msgs::msg::MultiFootPlanContinuous> continuous_msg;
  auto local_sub = node->create_subscription<quad_msgs::msg::RobotPlan>(
      "local_plan", 10,
      [&](quad_msgs::msg::RobotPlan::SharedPtr msg) { local_plan_msg = msg; });
  auto discrete_sub =
      node->create_subscription<quad_msgs::msg::MultiFootPlanDiscrete>(
          "foot_plan_discrete", 10,
          [&](quad_msgs::msg::MultiFootPlanDiscrete::SharedPtr msg) {
            discrete_msg = msg;
          });
  auto continuous_sub =
      node->create_subscription<quad_msgs::msg::MultiFootPlanContinuous>(
          "foot_plan_continuous", 10,
          [&](quad_msgs::msg::MultiFootPlanContinuous::SharedPtr msg) {
            continuous_msg = msg;
          });

  planner.map_frame_ = "map";
  planner.current_state_timestamp_ = node->now();
  planner.initial_timestamp_ = planner.current_state_timestamp_;
  planner.current_plan_index_ = 5;
  planner.first_element_duration_ = 0.02;
  planner.compute_time_ = 3.5;
  planner.N_current_ = 3;
  planner.body_plan_ = Eigen::MatrixXd::Zero(3, 12);
  planner.body_plan_.col(2).setConstant(0.35);
  planner.foot_positions_world_ = Eigen::MatrixXd::Zero(3, 12);
  planner.foot_velocities_world_ = Eigen::MatrixXd::Zero(3, 12);
  planner.foot_accelerations_world_ = Eigen::MatrixXd::Zero(3, 12);
  planner.grf_plan_ = Eigen::MatrixXd::Zero(2, 12);
  planner.grf_plan_(0, 2) = 10.0;
  planner.grf_plan_(1, 5) = 11.0;
  planner.ref_primitive_plan_ = Eigen::VectorXi::Zero(3);
  planner.ref_primitive_plan_(0) = 7;
  planner.ref_primitive_plan_(1) = 8;
  planner.contact_schedule_ = {
      {true, false, true, false},
      {true, true, false, false},
      {true, true, true, true},
  };
  for (int i = 0; i < 3; ++i) {
    for (int foot = 0; foot < 4; ++foot) {
      planner.foot_positions_world_(i, 3 * foot + 0) = 0.1 * foot;
      planner.foot_positions_world_(i, 3 * foot + 1) = -0.1 * foot;
      planner.foot_positions_world_(i, 3 * foot + 2) = 0.02;
    }
  }

  planner.publishLocalPlan();

  ASSERT_TRUE(spinUntilMessage(node, local_plan_msg));
  ASSERT_TRUE(spinUntilMessage(node, discrete_msg));
  ASSERT_TRUE(spinUntilMessage(node, continuous_msg));
  ASSERT_EQ(local_plan_msg->states.size(), 2u);
  ASSERT_EQ(local_plan_msg->grfs.size(), 2u);
  EXPECT_EQ(local_plan_msg->header.frame_id, "map");
  EXPECT_EQ(local_plan_msg->plan_indices[0], 5u);
  EXPECT_EQ(local_plan_msg->plan_indices[1], 6u);
  EXPECT_EQ(local_plan_msg->primitive_ids[0], 7u);
  EXPECT_EQ(local_plan_msg->primitive_ids[1], 8u);
  ASSERT_EQ(local_plan_msg->grfs[0].contact_states.size(), 4u);
  EXPECT_TRUE(local_plan_msg->grfs[0].contact_states[0]);
  EXPECT_FALSE(local_plan_msg->grfs[0].contact_states[1]);
  ASSERT_EQ(continuous_msg->states.size(), 3u);
  EXPECT_EQ(continuous_msg->states[0].traj_index, 5u);
  EXPECT_EQ(continuous_msg->states[1].traj_index, 6u);
  ASSERT_EQ(discrete_msg->feet.size(), 4u);
  ASSERT_EQ(discrete_msg->feet[1].footholds.size(), 1u);
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
