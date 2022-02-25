#include "nmpc_controller/nmpc_controller.h"

NMPCController::NMPCController(int type) {
  type_ = type;

  switch (type_) {
    case 0:
      // Leg controller
      param_ns_ = "leg";
      break;
    case 1:
      // Centralized tail controller
      param_ns_ = "centralized_tail";
      break;
    case 2:
      // Distributed tail controller
      param_ns_ = "distributed_tail";
      break;
    case 3:
      // Decentralized tail controller
      param_ns_ = "decentralized_tail";
      break;
    default:
      param_ns_ = "leg";
      break;
  }

  // Load MPC/system parameters
  double mu;
  ros::param::get("/nmpc_controller/" + param_ns_ + "/horizon_length", N_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_dimension", n_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_dimension", m_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/step_length", dt_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/friction_coefficient",
                  mu);

  // Load MPC cost weighting and bounds
  std::vector<double> state_weights, control_weights, state_weights_factors,
      control_weights_factors, state_lower_bound, state_upper_bound,
      state_lower_bound_null, state_upper_bound_null, control_lower_bound,
      control_upper_bound;
  double panic_weights;
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_weights",
                  state_weights);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_weights",
                  control_weights);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/panic_weights",
                  panic_weights);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_weights_factors",
                  state_weights_factors);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_weights_factors",
                  control_weights_factors);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_lower_bound",
                  state_lower_bound);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_upper_bound",
                  state_upper_bound);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_lower_bound",
                  control_lower_bound);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_upper_bound",
                  control_upper_bound);

  ros::param::get("/nmpc_controller/leg_complex/null_space_dimension", n_null_);
  ros::param::get("/nmpc_controller/leg_complex/state_lower_bound",
                  state_lower_bound_null);
  ros::param::get("/nmpc_controller/leg_complex/state_upper_bound",
                  state_upper_bound_null);
  ros::param::get("/nmpc_controller/takeoff_state_weight_factor",
                  takeoff_state_weight_factor_);

  Eigen::Map<Eigen::VectorXd> Q(state_weights.data(), n_),
      R(control_weights.data(), m_), Q_factor(state_weights_factors.data(), N_),
      R_factor(control_weights_factors.data(), N_),
      x_min(state_lower_bound.data(), n_), x_max(state_upper_bound.data(), n_),
      x_min_null(state_lower_bound_null.data(), state_lower_bound_null.size()),
      x_max_null(state_upper_bound_null.data(), state_upper_bound_null.size()),
      u_min(control_lower_bound.data(), m_),
      u_max(control_upper_bound.data(), m_);

  Eigen::VectorXd x_min_complex(n_ + n_null_), x_max_complex(n_ + n_null_);
  x_min_complex.segment(0, n_) = x_min;
  x_min_complex.segment(n_, n_null_).fill(-2e19);  // = x_min_null;
  x_max_complex.segment(0, n_) = x_max;
  x_max_complex.segment(n_, n_null_).fill(2e19);  // = x_max_null;

  mynlp_ = new quadNLP(type_, N_, n_, n_null_, m_, dt_, mu, panic_weights, Q, R,
                       Q_factor, R_factor, x_min, x_max, x_min_complex,
                       x_max_complex, u_min, u_max);

  app_ = IpoptApplicationFactory();

  // app_->Options()->SetIntegerValue("max_iter", 100);
  // app_->Options()->SetStringValue("print_timing_statistics", "yes");
  app_->Options()->SetStringValue("linear_solver", "ma57");
  app_->Options()->SetIntegerValue("print_level", 5);
  // app_->Options()->SetStringValue("mu_strategy", "adaptive");
  // app_->Options()->SetStringValue("nlp_scaling_method", "none");
  app_->Options()->SetStringValue("fixed_variable_treatment",
                                  "make_parameter_nodual");
  app_->Options()->SetNumericValue("tol", 1e-6);
  app_->Options()->SetNumericValue("dual_inf_tol", 1e10);
  app_->Options()->SetNumericValue("constr_viol_tol", 1e-2);
  app_->Options()->SetNumericValue("compl_inf_tol", 1e-2);
  app_->Options()->SetNumericValue("warm_start_bound_push", 1e-6);
  app_->Options()->SetNumericValue("warm_start_slack_bound_push", 1e-6);
  app_->Options()->SetNumericValue("warm_start_mult_bound_push", 1e-6);

  app_->Options()->SetNumericValue("max_wall_time", 10.0 * dt_);
  app_->Options()->SetNumericValue("max_cpu_time", 10.0 * dt_);

  ApplicationReturnStatus status;
  status = app_->Initialize();
  if (status != Solve_Succeeded) {
    std::cout << std::endl
              << std::endl
              << "*** Error during NMPC initialization!" << std::endl;
    return;
  }

  require_init_ = true;
}

bool NMPCController::computeLegPlan(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions,
    const Eigen::MatrixXd &foot_velocities,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::VectorXd &ref_ground_height,
    const double &first_element_duration, const bool &same_plan_index,
    const Eigen::VectorXi &ref_primitive_id,
    const Eigen::VectorXi &complexity_schedule, Eigen::MatrixXd &state_traj,
    Eigen::MatrixXd &control_traj) {
  // Update the complexity
  mynlp_->update_complexity_schedule(complexity_schedule);

  // Local planner will send a reference traj with N+1 rows
  mynlp_->update_solver(initial_state, ref_traj, foot_positions,
                        contact_schedule, ref_ground_height,
                        first_element_duration, same_plan_index, require_init_);
  require_init_ = false;

  // mynlp_->feet_location_ = foot_positions;
  mynlp_->foot_pos_world_ = foot_positions;
  mynlp_->foot_vel_world_ = foot_velocities;

  for (int i = 0; i < ref_primitive_id.size() - 1; i++) {
    if (ref_primitive_id(i, 0) == 1 && ref_primitive_id(i + 1, 0) == 2) {
      mynlp_->Q_factor_(i, 0) =
          mynlp_->Q_factor_(i, 0) * takeoff_state_weight_factor_;
      ROS_WARN_THROTTLE(0.5, "leap detected, increasing weights");
    }
  }

  bool success = this->computePlan(initial_state, ref_traj, foot_positions,
                                   contact_schedule, state_traj, control_traj);

  return success;
}

bool NMPCController::computeCentralizedTailPlan(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::VectorXd &tail_initial_state,
    const Eigen::MatrixXd &tail_ref_traj,
    const Eigen::VectorXd &ref_ground_height, Eigen::MatrixXd &state_traj,
    Eigen::MatrixXd &control_traj, Eigen::MatrixXd &tail_state_traj,
    Eigen::MatrixXd &tail_control_traj) {
  return true;
}

bool NMPCController::computeDistributedTailPlan(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::VectorXd &tail_initial_state,
    const Eigen::MatrixXd &tail_ref_traj, const Eigen::MatrixXd &state_traj,
    const Eigen::MatrixXd &control_traj,
    const Eigen::VectorXd &ref_ground_height,
    const double &first_element_duration, const bool &same_plan_index,
    Eigen::MatrixXd &tail_state_traj, Eigen::MatrixXd &tail_control_traj) {
  Eigen::MatrixXd ref_traj_with_tail(N_ + 1, 16), state_traj_with_tail(N_, 16),
      control_traj_with_tail(N_, 14);
  Eigen::VectorXd initial_state_with_tail(16);

  ref_traj_with_tail.setZero();
  ref_traj_with_tail.leftCols(6) = ref_traj.leftCols(6);
  ref_traj_with_tail.block(0, 6, N_ + 1, 2) = tail_ref_traj.leftCols(2);
  ref_traj_with_tail.block(0, 8, N_ + 1, 6) = ref_traj.rightCols(6);
  ref_traj_with_tail.block(0, 14, N_ + 1, 2) = tail_ref_traj.rightCols(2);

  initial_state_with_tail.setZero();
  initial_state_with_tail.head(6) = initial_state.head(6);
  initial_state_with_tail.segment(6, 2) = tail_initial_state.head(2);
  initial_state_with_tail.segment(8, 6) = initial_state.tail(6);
  initial_state_with_tail.segment(14, 2) = tail_initial_state.tail(2);

  mynlp_->update_solver(initial_state_with_tail,
                        ref_traj_with_tail.bottomRows(N_), foot_positions,
                        contact_schedule, state_traj.bottomRows(N_),
                        control_traj, ref_ground_height.tail(N_),
                        first_element_duration, same_plan_index, require_init_);
  require_init_ = false;

  bool success = this->computePlan(
      initial_state_with_tail, ref_traj_with_tail, foot_positions,
      contact_schedule, state_traj_with_tail, control_traj_with_tail);

  tail_state_traj.leftCols(2) = state_traj_with_tail.block(0, 6, N_ + 1, 2);
  tail_state_traj.rightCols(2) = state_traj_with_tail.block(0, 14, N_ + 1, 2);

  tail_control_traj = control_traj_with_tail.leftCols(2);

  return success;
}

bool NMPCController::computePlan(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions,
    const std::vector<std::vector<bool>> &contact_schedule,
    Eigen::MatrixXd &state_traj, Eigen::MatrixXd &control_traj) {
  ApplicationReturnStatus status;
  app_->Options()->SetNumericValue("mu_init", mynlp_->mu0_);
  if (mynlp_->warm_start_) {
    app_->Options()->SetStringValue("warm_start_init_point", "yes");
  } else {
    app_->Options()->SetStringValue("warm_start_init_point", "no");
  }

  status = app_->OptimizeTNLP(mynlp_);

  state_traj = Eigen::MatrixXd::Zero(N_ + 1, n_);
  control_traj = Eigen::MatrixXd::Zero(N_, m_);

  state_traj.row(0) = mynlp_->get_state_var(mynlp_->w0_, 0).transpose();
  for (int i = 0; i < N_; ++i) {
    control_traj.row(i) = mynlp_->get_control_var(mynlp_->w0_, i).transpose();
    state_traj.row(i + 1) =
        mynlp_->get_state_var(mynlp_->w0_, i + 1).transpose();
  }

  if (status == Solve_Succeeded) {
    mynlp_->warm_start_ = true;

    return true;
  } else {
    mynlp_->mu0_ = 1e-1;
    mynlp_->warm_start_ = false;
    // require_init_ = true;

    ROS_WARN_STREAM(param_ns_ << " solving fail");

    return false;
  }
}
