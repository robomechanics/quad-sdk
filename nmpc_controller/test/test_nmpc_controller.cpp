#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

#include <array>
#include <cstdio>
#include <cstdlib>
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

std::string spiritRobotDescription() {
  static const std::string urdf = []() {
    const char* source_dir = std::getenv("NMPC_SOURCE_DIR");
    if (source_dir == nullptr) {
      throw std::runtime_error("Missing NMPC source env");
    }
    return runXacro(std::string(source_dir) +
                    "/quad_simulator/spirit_description/models/spirit/urdf/"
                    "spirit.urdf.xacro");
  }();
  return urdf;
}

double spiritMass() {
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
      rclcpp::Parameter("robot_description", spiritRobotDescription()),
      rclcpp::Parameter("global_body_planner.mass", spiritMass()),
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
                                         spiritRobotDescription());
  }
  return node;
}

}  // namespace

TEST(NMPCControllerTest, ConstructorLoadsYamlConfigurationAndSolverState) {
  auto node = makeNode();
  NMPCController controller(node, 0, "spirit");

  EXPECT_EQ(controller.robot_ns_, "spirit");
  EXPECT_EQ(controller.robot_id_, 0);
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

TEST(NMPCControllerTest, RobotIdDefaultsToSpiritForUnknownId) {
  auto node = makeNode();
  NMPCController controller(node, 99, "unknown");

  EXPECT_EQ(controller.robot_ns_, "spirit");
  EXPECT_EQ(controller.robot_id_, 99);
  EXPECT_EQ(controller.mynlp_->default_system_, SPIRIT);
}

TEST(NMPCControllerTest, HorizonLengthShrinksOnSlowSolveAndRecovers) {
  auto node = makeNode({
      rclcpp::Parameter("nmpc_controller.enable_variable_horizon", true),
  });
  NMPCController controller(node, 0, "spirit");

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
  NMPCController controller(node, 0, "spirit");
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
