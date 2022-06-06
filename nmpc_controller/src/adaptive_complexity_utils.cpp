#include "nmpc_controller/nmpc_controller.h"

Eigen::VectorXi NMPCController::updateAdaptiveComplexitySchedule(
    const Eigen::MatrixXd &state_traj_heuristic,
    const Eigen::MatrixXd &control_traj_heuristic,
    const Eigen::MatrixXd &state_traj_lifted,
    const Eigen::MatrixXd &control_traj_lifted) {
  // quad_utils::FunctionTimer timer(__FUNCTION__);

  // Declare decision and constraint vars
  Eigen::VectorXi adaptive_complexity_schedule(N_);
  adaptive_complexity_schedule.setZero();

  // Initialize max violation data for sparse AC
  double max_violation_val = -2e19;
  int max_violation_fe = -1;
  int max_violation_index = -1;

  // Initialize vectors of max violation
  Eigen::VectorXd max_state_violation(N_), max_control_violation(N_ - 1),
      max_constraint_violation(N_ - 1);
  max_state_violation.fill(-2e19);
  max_control_violation.fill(-2e19);
  max_constraint_violation.fill(-2e19);

  // Initialize vector for bounds and violations
  Eigen::VectorXd x_lb, x_ub, u_lb, u_ub, g_lb, g_ub;
  Eigen::VectorXd x_lb_viol, x_ub_viol, u_lb_viol, u_ub_viol, g_lb_viol,
      g_ub_viol;

  // Assume lift was valid
  bool valid_lift = true;

  // Get optimization tolerances
  double var_tol, constr_tol;
  app_->Options()->GetNumericValue("tol", var_tol, "");
  app_->Options()->GetNumericValue("constr_viol_tol", constr_tol, "");

  // Load the initial state
  Eigen::VectorXd x0, u, x1, g;
  x0 = state_traj_heuristic.row(0);

  // Loop through heuristic trajectory to evaluate constraints
  for (int i = 0; i < N_ - 1; i++) {
    u = control_traj_heuristic.row(i);
    x1 = state_traj_heuristic.row(i + 1);

    // Get constraint values and bounds
    g = mynlp_->eval_g_single_complex_fe(i, x0, u, x1);
    mynlp_->get_bounds_info_single_complex_fe(i, x_lb, x_ub, u_lb, u_ub, g_lb,
                                              g_ub);

    // Compute constraint violations (positive = violation)
    x_lb_viol = x_lb - x1;
    x_ub_viol = x1 - x_ub;
    u_lb_viol = u_lb - u;
    u_ub_viol = u - u_ub;
    g_lb_viol = g_lb - g;
    g_ub_viol = g - g_ub;

    // Update maximum violations
    max_state_violation[i + 1] = (x_lb_viol.cwiseMax(x_ub_viol)).maxCoeff();
    max_control_violation[i] = (u_lb_viol.cwiseMax(u_ub_viol)).maxCoeff();
    max_constraint_violation[i] = (g_lb_viol.cwiseMax(g_ub_viol)).maxCoeff();

    double eps = 1e-4;

    // Check state bounds
    for (int j = 0; j < x1.size(); j++) {
      double current_state_violation = std::max(x_lb_viol[j], x_ub_viol[j]);

      if (current_state_violation > max_violation_val) {
        max_violation_val = current_state_violation;
        max_violation_fe = i + 1;
        max_violation_index = j;
      }

      if (current_state_violation > var_tol) {
        if (mynlp_->n_vec_[i + 1] < config_.x_dim_complex) {
          valid_lift = false;
          ROS_WARN(
              "Lift not valid, state bound %d violated in FE %d: %5.3f <= "
              "%5.3f <= "
              "%5.3f",
              j, i, x_lb[j] - var_tol, x1[j], x_ub[j] + var_tol);
        }
        adaptive_complexity_schedule[i] = 1;
        adaptive_complexity_schedule[i + 1] = 1;
        ROS_WARN(
            "State bound %d violated in FE %d: %5.3f <= "
            "%5.3f <= "
            "%5.3f",
            j, i, x_lb[j] - var_tol, x1[j], x_ub[j] + var_tol);
        std::cout << "x1 = " << x1 << std::endl;
      } else if (std::abs(current_state_violation - var_tol) < eps) {
        ROS_WARN(
            "State bound %d is active in FE %d: %5.3f <= "
            "%5.3f <= "
            "%5.3f",
            j, i, x_lb[j] - var_tol, x1[j], x_ub[j] + var_tol);
      }
    }

    // Check control bounds
    for (int j = 0; j < u.size(); j++) {
      double current_control_violation = std::max(u_lb_viol[j], u_ub_viol[j]);

      if (current_control_violation > max_violation_val) {
        max_violation_val = current_control_violation;
        max_violation_fe = i;
        max_violation_index = j;
      }

      if (current_control_violation > var_tol) {
        if (mynlp_->n_vec_[i] < config_.x_dim_complex) {
          valid_lift = false;
          ROS_WARN(
              "Lift not valid, control bound %d violated in FE %d: %5.3f <= "
              "%5.3f <= "
              "%5.3f",
              j, i, u_lb[j] - var_tol, u[j], u_ub[j] + var_tol);
        }
        adaptive_complexity_schedule[i] = 1;
        adaptive_complexity_schedule[i + 1] = 1;
        ROS_WARN(
            "Control bound %d violated in FE %d: %5.3f <= "
            "%5.3f <= "
            "%5.3f",
            j, i, u_lb[j] - var_tol, u[j], u_ub[j] + var_tol);
      } else if (std::abs(current_control_violation - var_tol) < eps) {
        ROS_WARN(
            "Control bound %d is active in FE %d: %5.3f <= "
            "%5.3f <= "
            "%5.3f",
            j, i, u_lb[j] - var_tol, u[j], u_ub[j] + var_tol);
      }
    }

    // Check constraints
    for (int j = 0; j < g.size(); j++) {
      double current_constraint_violation =
          std::max(g_lb_viol[j], g_ub_viol[j]);

      if (current_constraint_violation > max_violation_val) {
        max_violation_val = current_constraint_violation;
        max_violation_fe = i;
        max_violation_index = j;
      }

      if (current_constraint_violation > constr_tol) {
        if (mynlp_->n_vec_[i] < config_.x_dim_complex) {
          valid_lift = false;
          ROS_WARN(
              "Lift not valid, constraint %s violated in FE %d: %5.3f <= %5.3f "
              "<= "
              "%5.3f",
              mynlp_->constr_names_[COMPLEX_TO_COMPLEX][j].c_str(), i,
              g_lb[j] - constr_tol, g[j], g_ub[j] + constr_tol);
        }
        adaptive_complexity_schedule[i] = 1;
        adaptive_complexity_schedule[i + 1] = 1;
        ROS_WARN(
            "Constraint %s violated in FE %d: %5.3f <= %5.3f "
            "<= "
            "%5.3f",
            mynlp_->constr_names_[COMPLEX_TO_COMPLEX][j].c_str(), i,
            g_lb[j] - constr_tol, g[j], g_ub[j] + constr_tol);
      } else if (std::abs(current_constraint_violation - var_tol) < eps) {
        ROS_WARN(
            "Constraint %s is active in FE %d: %5.3f <= %5.3f "
            "<= "
            "%5.3f",
            mynlp_->constr_names_[COMPLEX_TO_COMPLEX][j].c_str(), i,
            g_lb[j] - constr_tol, g[j], g_ub[j] + constr_tol);
      }
    }

    // std::cout << "g_ub_viol = " << g_ub_viol.segment(84, 12).transpose()
    //           << std::endl;

    x0 = x1;
  }

  // If using sparse adaptive complexity, select the subset of elements to keep
  // complex
  if ((!valid_lift) && is_adaptive_complexity_sparse_) {
    adaptive_complexity_schedule.setZero();
    adaptive_complexity_schedule[max_violation_fe - 1] = 1;
    adaptive_complexity_schedule[max_violation_fe] = 1;
  }
  return adaptive_complexity_schedule;
}

void NMPCController::updateHorizonLength() {
  // if (N_ % 2 == 0) {
  //   N_ += 3;
  // } else {
  //   N_ -= 3;
  // }
  // N_ = 20 + std::floor(5 * cos(M_PI * ros::Time::now().toSec()));

  if (diagnostics_.compute_time > dt_) {
    N_ =
        std::max(N_ - (int)std::floor(diagnostics_.compute_time / dt_), N_min_);
  } else {
    N_ = std::min(N_ + 1, N_max_);
  }

  N_ = std::max(std::min(N_, N_max_), N_min_);
  std::cout << "N_ = " << N_ << std::endl;
}
