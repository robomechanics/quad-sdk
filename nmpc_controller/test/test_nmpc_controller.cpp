#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <grid_map_core/grid_map_core.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nmpc_controller/nmpc_controller.hpp"

namespace {

constexpr double kTol = 1e-9;

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
    const char* source_dir = std::getenv("NMPC_SOURCE_DIR");
    if (source_dir == nullptr) {
      throw std::runtime_error("Missing NMPC source env");
    }
    return runXacro(std::string(source_dir) +
                    "/quad_simulator/go2_description/models/go2/urdf/"
                    "go2.urdf.xacro");
  }();
  return urdf;
}

double go2Mass() {
  static const double mass = []() {
    const char* robot_params = std::getenv("NMPC_ROBOT_PARAMS");
    if (robot_params == nullptr) {
      throw std::runtime_error("Missing NMPC robot parameter file env");
    }

    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", robot_params});
    auto node = std::make_shared<rclcpp::Node>("global_body_planner", options);
    double loaded_mass = 0.0;
    if (!node->has_parameter("global_body_planner.mass")) {
      node->declare_parameter<double>("global_body_planner.mass");
    }
    if (!node->get_parameter("global_body_planner.mass", loaded_mass)) {
      throw std::runtime_error("Missing global_body_planner.mass");
    }
    return loaded_mass;
  }();
  return mass;
}

rclcpp::NodeOptions nodeOptions(
    const std::vector<rclcpp::Parameter>& extra = {}) {
  const char* nmpc_params = std::getenv("NMPC_TEST_PARAMS");
  const char* local_params = std::getenv("NMPC_LOCAL_PLANNER_PARAMS");
  const char* robot_params = std::getenv("NMPC_ROBOT_PARAMS");
  if (nmpc_params == nullptr || local_params == nullptr ||
      robot_params == nullptr) {
    throw std::runtime_error("Missing NMPC test parameter file env");
  }

  std::vector<rclcpp::Parameter> overrides = {
      rclcpp::Parameter("robot_description", go2RobotDescription()),
      rclcpp::Parameter("global_body_planner.mass", go2Mass()),
  };
  overrides.insert(overrides.end(), extra.begin(), extra.end());

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", nmpc_params,
                     "--params-file", local_params, "--params-file",
                     robot_params});
  options.parameter_overrides(overrides);
  return options;
}

std::shared_ptr<rclcpp::Node> makeNode(
    const std::vector<rclcpp::Parameter>& extra = {}) {
  auto node = std::make_shared<rclcpp::Node>("local_planner", nodeOptions(extra));
  if (!node->has_parameter("robot_description")) {
    node->declare_parameter<std::string>("robot_description",
                                         go2RobotDescription());
  }
  return node;
}

std::vector<std::vector<bool>> stanceSchedule(int horizon) {
  return std::vector<std::vector<bool>>(
      horizon, std::vector<bool>{true, true, true, true});
}

Eigen::VectorXd initialState(const NMPCController& controller) {
  Eigen::VectorXd state =
      Eigen::VectorXd::Zero(controller.config_.x_dim_complex);
  state[2] = 0.27;
  return state;
}

Eigen::MatrixXd referenceTrajectory(const NMPCController& controller) {
  Eigen::MatrixXd ref =
      Eigen::MatrixXd::Zero(controller.N_, controller.config_.x_dim_simple);
  ref.col(2).setConstant(0.27);
  return ref;
}

grid_map::GridMap flatTerrain() {
  grid_map::GridMap terrain({"z_inpainted", "normal_vectors_x",
                             "normal_vectors_y", "normal_vectors_z"});
  terrain.setGeometry(grid_map::Length(4.0, 4.0), 0.1);
  for (grid_map::GridMapIterator it(terrain); !it.isPastEnd(); ++it) {
    terrain.at("z_inpainted", *it) = 0.0;
    terrain.at("normal_vectors_x", *it) = 0.0;
    terrain.at("normal_vectors_y", *it) = 0.0;
    terrain.at("normal_vectors_z", *it) = 1.0;
  }
  return terrain;
}

}  // namespace

TEST(NMPCControllerTest, ConstructorLoadsYamlConfigurationAndSolverState) {
  auto node = makeNode();
  NMPCController controller(node, 2, "go2");

  EXPECT_EQ(controller.robot_ns_, "go2");
  EXPECT_EQ(controller.robot_id_, 2);
  EXPECT_EQ(controller.N_, 26);
  EXPECT_EQ(controller.N_max_, 26);
  EXPECT_EQ(controller.N_min_, 10);
  EXPECT_NEAR(controller.dt_, 0.03, kTol);
  EXPECT_TRUE(controller.require_init_);
  EXPECT_FALSE(controller.enable_variable_horizon_);
  EXPECT_FALSE(controller.enable_mixed_complexity_);
  EXPECT_FALSE(controller.enable_adaptive_complexity_);

  ASSERT_NE(GetRawPtr(controller.mynlp_), nullptr);
  ASSERT_NE(GetRawPtr(controller.app_), nullptr);
  EXPECT_EQ(controller.config_.x_dim_simple, 12);
  EXPECT_EQ(controller.config_.u_dim_simple, 12);
  EXPECT_EQ(controller.config_.g_dim_simple, 28);
  EXPECT_EQ(controller.config_.x_dim_complex, 60);
  EXPECT_EQ(controller.config_.u_dim_complex, 36);
  EXPECT_EQ(controller.config_.g_dim_complex, 108);
  EXPECT_EQ(controller.config_.x_dim_null, 48);
  EXPECT_EQ(controller.config_.u_dim_null, 24);
}

TEST(NMPCControllerTest, RobotIdSelectsGo2ForGo2Id) {
  auto node = makeNode();
  NMPCController controller(node, 2, "go2");

  EXPECT_EQ(controller.robot_ns_, "go2");
  EXPECT_EQ(controller.robot_id_, 2);
  EXPECT_EQ(controller.mynlp_->default_system_, GO2);
}

TEST(NMPCControllerTest, Go2ForcesMixedComplexityOff) {
  auto node = makeNode({
      rclcpp::Parameter("nmpc_controller.enable_mixed_complexity", true),
      rclcpp::Parameter("nmpc_controller.enable_adaptive_complexity", true),
  });
  NMPCController controller(node, 2, "go2");

  EXPECT_FALSE(controller.enable_mixed_complexity_);
  EXPECT_FALSE(controller.enable_adaptive_complexity_);
  EXPECT_EQ(controller.mynlp_->default_system_, GO2);
}

TEST(NMPCControllerTest, ConstructorRejectsMalformedConfigVectors) {
  auto node = makeNode({
      rclcpp::Parameter("nmpc_controller.body.x_lb",
                        std::vector<double>{-1.0, -1.0}),
  });

  EXPECT_THROW(NMPCController controller(node, 2, "go2"), std::runtime_error);
}

TEST(NMPCControllerTest, HorizonLengthShrinksOnSlowSolveAndRecovers) {
  auto node = makeNode({
      rclcpp::Parameter("nmpc_controller.enable_variable_horizon", true),
  });
  NMPCController controller(node, 2, "go2");

  controller.diagnostics_.compute_time = 0.08;
  controller.updateHorizonLength();
  EXPECT_EQ(controller.N_, 24);

  controller.diagnostics_.compute_time = 0.0;
  controller.updateHorizonLength();
  EXPECT_EQ(controller.N_, 25);

  controller.N_ = controller.N_max_;
  controller.updateHorizonLength();
  EXPECT_EQ(controller.N_, controller.N_max_);
}

TEST(NMPCControllerTest, AdaptiveComplexityPromotesViolatingElements) {
  auto node = makeNode();
  NMPCController controller(node, 2, "go2");
  controller.is_adaptive_complexity_sparse_ = false;

  Eigen::MatrixXd state_traj =
      Eigen::MatrixXd::Zero(controller.N_, controller.config_.x_dim_complex);
  Eigen::MatrixXd control_traj =
      Eigen::MatrixXd::Zero(controller.N_ - 1, controller.config_.u_dim_complex);
  state_traj(1, 2) = -1.0;

  const Eigen::VectorXi schedule = controller.updateAdaptiveComplexitySchedule(
      state_traj, control_traj, state_traj, control_traj);

  ASSERT_EQ(schedule.size(), controller.N_);
  EXPECT_EQ(schedule[0], 1);
  EXPECT_EQ(schedule[1], 1);
  EXPECT_GT(schedule.sum(), 0);
}

TEST(NMPCControllerTest, ComputeLegPlanUpdatesSolverInputsAndTrajectories) {
  auto node = makeNode({
      rclcpp::Parameter("nmpc_controller.enable_variable_horizon", true),
  });
  NMPCController controller(node, 2, "go2");
  controller.app_->Options()->SetIntegerValue("max_iter", 1);

  Eigen::MatrixXd foot_body =
      Eigen::MatrixXd::Constant(controller.N_, controller.mynlp_->n_foot_ / 2,
                                0.05);
  Eigen::MatrixXd foot_world =
      Eigen::MatrixXd::Constant(controller.N_, controller.mynlp_->n_foot_ / 2,
                                0.10);
  Eigen::MatrixXd foot_velocity =
      Eigen::MatrixXd::Constant(controller.N_, controller.mynlp_->n_foot_ / 2,
                                0.01);
  Eigen::VectorXd ground = Eigen::VectorXd::Zero(controller.N_);
  Eigen::MatrixXd state_traj;
  Eigen::MatrixXd control_traj;

  const bool success = controller.computeLegPlan(
      initialState(controller), referenceTrajectory(controller), foot_body,
      foot_world, foot_velocity, stanceSchedule(controller.N_), ground, 0.01, 0,
      flatTerrain(), state_traj, control_traj);

  EXPECT_FALSE(success);
  EXPECT_EQ(state_traj.rows(), controller.N_);
  EXPECT_EQ(state_traj.cols(), controller.config_.x_dim_simple);
  EXPECT_EQ(control_traj.rows(), controller.N_ - 1);
  EXPECT_EQ(control_traj.cols(), controller.config_.u_dim_simple);
  EXPECT_TRUE(state_traj.allFinite());
  EXPECT_TRUE(control_traj.allFinite());
  EXPECT_TRUE(controller.mynlp_->foot_pos_body_.isApprox(-foot_body));
  EXPECT_TRUE(controller.mynlp_->foot_pos_world_.isApprox(foot_world));
  EXPECT_TRUE(controller.mynlp_->foot_vel_world_.isApprox(foot_velocity));
  EXPECT_NEAR(controller.mynlp_->first_element_duration_, 0.01, kTol);
  EXPECT_EQ(controller.diagnostics_.horizon_length, controller.N_);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const rcutils_ret_t logging_ret = rcutils_logging_set_logger_level(
      "local_planner", RCUTILS_LOG_SEVERITY_FATAL);
  (void)logging_ret;
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
