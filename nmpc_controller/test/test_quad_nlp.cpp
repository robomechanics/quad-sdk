#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cmath>
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

rclcpp::NodeOptions nodeOptions() {
  const char* nmpc_params = std::getenv("NMPC_TEST_PARAMS");
  const char* local_params = std::getenv("NMPC_LOCAL_PLANNER_PARAMS");
  const char* robot_params = std::getenv("NMPC_ROBOT_PARAMS");
  if (nmpc_params == nullptr || local_params == nullptr ||
      robot_params == nullptr) {
    throw std::runtime_error("Missing NMPC test parameter file env");
  }

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", nmpc_params,
                     "--params-file", local_params, "--params-file",
                     robot_params});
  options.parameter_overrides({
      rclcpp::Parameter("robot_description", spiritRobotDescription()),
      rclcpp::Parameter("global_body_planner.mass", spiritMass()),
  });
  return options;
}

std::shared_ptr<rclcpp::Node> makeNode() {
  auto node = std::make_shared<rclcpp::Node>("local_planner", nodeOptions());
  if (!node->has_parameter("robot_description")) {
    node->declare_parameter<std::string>("robot_description",
                                         spiritRobotDescription());
  }
  return node;
}

std::unique_ptr<NMPCController> makeController() {
  return std::make_unique<NMPCController>(makeNode(), 0, "spirit");
}

std::vector<std::vector<bool>> stanceSchedule(int horizon) {
  return std::vector<std::vector<bool>>(horizon,
                                        std::vector<bool>{true, true, true, true});
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

void updateSolverOnce(NMPCController& controller) {
  Eigen::MatrixXd feet =
      Eigen::MatrixXd::Zero(controller.N_, controller.mynlp_->n_foot_ / 2);
  Eigen::VectorXd ground = Eigen::VectorXd::Zero(controller.N_);
  Eigen::VectorXi complexity = Eigen::VectorXi::Zero(controller.N_);
  controller.mynlp_->update_solver(initialState(controller),
                                   referenceTrajectory(controller), feet,
                                   stanceSchedule(controller.N_), complexity,
                                   ground, controller.dt_, 0, true);
}

}  // namespace

TEST(NLPDiagnosticsTest, LoadsMessageFields) {
  NLPDiagnostics diagnostics;
  diagnostics.compute_time = 0.12;
  diagnostics.cost = 3.4;
  diagnostics.iterations = 5;
  diagnostics.horizon_length = 7;
  diagnostics.complexity_schedule = Eigen::VectorXi::LinSpaced(3, 0, 2);
  diagnostics.element_times = Eigen::VectorXd::LinSpaced(3, 0.0, 0.2);

  quad_msgs::msg::RobotPlanDiagnostics msg;
  diagnostics.loadDiagnosticsMsg(msg);

  EXPECT_DOUBLE_EQ(msg.compute_time, 0.12);
  EXPECT_DOUBLE_EQ(msg.cost, 3.4);
  EXPECT_EQ(msg.iterations, 5);
  EXPECT_EQ(msg.horizon_length, 7);
  ASSERT_EQ(msg.complexity_schedule.size(), 3u);
  EXPECT_EQ(msg.complexity_schedule[0], 0u);
  EXPECT_EQ(msg.complexity_schedule[1], 1u);
  EXPECT_EQ(msg.complexity_schedule[2], 2u);
  ASSERT_EQ(msg.element_times.size(), 3u);
  EXPECT_NEAR(msg.element_times[2], 0.2, kTol);
}

TEST(QuadNLPTest, ConstructorBuildsStructureAndGeneratedFunctionTables) {
  auto controller = makeController();
  auto nlp = controller->mynlp_;

  Ipopt::Index n = 0;
  Ipopt::Index m = 0;
  Ipopt::Index nnz_jac_g = 0;
  Ipopt::Index nnz_h_lag = 0;
  Ipopt::TNLP::IndexStyleEnum index_style;
  EXPECT_TRUE(nlp->get_nlp_info(n, m, nnz_jac_g, nnz_h_lag, index_style));

  EXPECT_EQ(n, nlp->n_vars_);
  EXPECT_EQ(m, nlp->n_constraints_);
  EXPECT_EQ(index_style, Ipopt::TNLP::C_STYLE);
  EXPECT_GT(nnz_jac_g, 0);
  EXPECT_GT(nnz_h_lag, 0);

  ASSERT_EQ(nlp->eval_vec_.size(), static_cast<size_t>(nlp->num_sys_id_));
  for (int sys = 0; sys < nlp->num_sys_id_; ++sys) {
    for (int func = 0; func < nlp->num_func_id_; ++func) {
      EXPECT_NE(nlp->eval_vec_[sys][func], nullptr);
      EXPECT_NE(nlp->eval_work_vec_[sys][func], nullptr);
      EXPECT_NE(nlp->eval_sparsity_vec_[sys][func], nullptr);
    }
  }
}

TEST(QuadNLPTest, BoundsAndStartingPointMatchCachedState) {
  auto controller = makeController();
  auto nlp = controller->mynlp_;

  Ipopt::Index n = 0;
  Ipopt::Index m = 0;
  Ipopt::Index nnz_jac_g = 0;
  Ipopt::Index nnz_h_lag = 0;
  Ipopt::TNLP::IndexStyleEnum index_style;
  ASSERT_TRUE(nlp->get_nlp_info(n, m, nnz_jac_g, nnz_h_lag, index_style));

  std::vector<Ipopt::Number> x_l(n), x_u(n), g_l(m), g_u(m);
  EXPECT_TRUE(nlp->get_bounds_info(n, x_l.data(), x_u.data(), m, g_l.data(),
                                   g_u.data()));
  Eigen::Map<Eigen::VectorXd> x_l_vec(x_l.data(), n);
  Eigen::Map<Eigen::VectorXd> x_u_vec(x_u.data(), n);
  EXPECT_TRUE(nlp->get_primal_state_var(x_l_vec, 0).isApprox(nlp->x_current_));
  EXPECT_TRUE(nlp->get_primal_state_var(x_u_vec, 0).isApprox(nlp->x_current_));
  EXPECT_NEAR(nlp->get_primal_body_control_var(x_l_vec, 0)(2, 0), 10.0, kTol);
  EXPECT_NEAR(nlp->get_primal_body_control_var(x_u_vec, 0)(2, 0), 150.0,
              kTol);

  std::vector<Ipopt::Number> x(n), z_l(n), z_u(n), lambda(m);
  EXPECT_TRUE(nlp->get_starting_point(n, true, x.data(), true, z_l.data(),
                                      z_u.data(), m, true, lambda.data()));
  Eigen::Map<Eigen::VectorXd> x_vec(x.data(), n);
  Eigen::Map<Eigen::VectorXd> lambda_vec(lambda.data(), m);
  EXPECT_TRUE(x_vec.isApprox(nlp->w0_));
  EXPECT_TRUE(lambda_vec.isApprox(nlp->lambda0_));
}

TEST(QuadNLPTest, ObjectiveGradientAndConstraintsEvaluateFiniteValues) {
  auto controller = makeController();
  auto nlp = controller->mynlp_;
  updateSolverOnce(*controller);

  Ipopt::Index n = 0;
  Ipopt::Index m = 0;
  Ipopt::Index nnz_jac_g = 0;
  Ipopt::Index nnz_h_lag = 0;
  Ipopt::TNLP::IndexStyleEnum index_style;
  ASSERT_TRUE(nlp->get_nlp_info(n, m, nnz_jac_g, nnz_h_lag, index_style));

  double objective = 0.0;
  ASSERT_TRUE(nlp->eval_f(n, nlp->w0_.data(), true, objective));
  EXPECT_TRUE(std::isfinite(objective));

  Eigen::VectorXd grad = Eigen::VectorXd::Zero(n);
  ASSERT_TRUE(nlp->eval_grad_f(n, nlp->w0_.data(), true, grad.data()));
  EXPECT_TRUE(grad.allFinite());
  EXPECT_NEAR(nlp->get_slack_state_var(grad, 0)(0, 0), nlp->panic_weights_,
              kTol);

  Eigen::VectorXd constraints = Eigen::VectorXd::Zero(m);
  ASSERT_TRUE(nlp->eval_g(n, nlp->w0_.data(), true, m, constraints.data()));
  EXPECT_TRUE(constraints.allFinite());
}

TEST(QuadNLPTest, UpdateSolverAppliesContactScheduleAndGroundHeight) {
  auto controller = makeController();
  auto nlp = controller->mynlp_;

  Eigen::MatrixXd feet = Eigen::MatrixXd::Zero(controller->N_, nlp->n_foot_ / 2);
  Eigen::VectorXd ground = Eigen::VectorXd::LinSpaced(controller->N_, 0.0, 0.25);
  Eigen::VectorXi complexity = Eigen::VectorXi::Zero(controller->N_);
  auto schedule = stanceSchedule(controller->N_);
  schedule[0][0] = false;

  nlp->update_solver(initialState(*controller), referenceTrajectory(*controller),
                     feet, schedule, complexity, ground, 0.01, 0, true);

  EXPECT_EQ(nlp->contact_sequence_(0, 0), 0);
  EXPECT_EQ(nlp->contact_sequence_(1, 0), 1);
  EXPECT_NEAR(nlp->first_element_duration_, 0.01, kTol);
  EXPECT_TRUE(nlp->ground_height_.transpose().isApprox(ground));

  Eigen::VectorXd x_lb, x_ub, u_lb, u_ub, g_lb, g_ub;
  ASSERT_TRUE(nlp->get_bounds_info_single_complex_fe(0, x_lb, x_ub, u_lb, u_ub,
                                                     g_lb, g_ub));
  EXPECT_NEAR(u_lb[2], 0.0, kTol);
  EXPECT_NEAR(u_ub[2], 0.0, kTol);
  EXPECT_NEAR(u_lb[5], 10.0, kTol);
}

TEST(QuadNLPTest, TrajectoryHelpersReturnComplexModelTrajectories) {
  auto controller = makeController();
  auto nlp = controller->mynlp_;
  updateSolverOnce(*controller);

  Eigen::MatrixXd lifted_states(controller->N_, controller->config_.x_dim_complex);
  Eigen::MatrixXd lifted_controls(controller->N_ - 1,
                                  controller->config_.u_dim_complex);
  Eigen::MatrixXd heuristic_states(controller->N_,
                                   controller->config_.x_dim_complex);
  Eigen::MatrixXd heuristic_controls(controller->N_ - 1,
                                     controller->config_.u_dim_complex);

  nlp->get_lifted_trajectory(lifted_states, lifted_controls);
  nlp->get_heuristic_trajectory(heuristic_states, heuristic_controls);

  EXPECT_EQ(lifted_states.rows(), controller->N_);
  EXPECT_EQ(lifted_controls.cols(), controller->config_.u_dim_complex);
  EXPECT_TRUE(lifted_states.allFinite());
  EXPECT_TRUE(lifted_controls.allFinite());
  EXPECT_TRUE(heuristic_states.allFinite());
  EXPECT_TRUE(heuristic_controls.allFinite());
}
