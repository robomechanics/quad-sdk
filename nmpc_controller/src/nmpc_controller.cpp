#include "nmpc_controller/nmpc_controller.h"

NMPCController::NMPCController(int type) {
  // Define NMPC type
  type_ = type;

  // Define parameter namespace
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
      control_lower_bound, control_upper_bound;
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
  Eigen::Map<Eigen::MatrixXd> Q(state_weights.data(), n_, 1),
      R(control_weights.data(), m_, 1),
      Q_factor(state_weights_factors.data(), N_, 1),
      R_factor(control_weights_factors.data(), N_, 1),
      x_min(state_lower_bound.data(), n_, 1),
      x_max(state_upper_bound.data(), n_, 1),
      u_min(control_lower_bound.data(), m_, 1),
      u_max(control_upper_bound.data(), m_, 1);

  mynlp_ = new quadNLP(type_, N_, n_, m_, dt_, mu, panic_weights, Q, R,
                       Q_factor, R_factor, x_min, x_max, u_min, u_max);

  app_ = IpoptApplicationFactory();

  // app_->Options()->SetIntegerValue("max_iter", 100);
  // app_->Options()->SetStringValue("print_timing_statistics", "yes");
  app_->Options()->SetStringValue("linear_solver", "ma57");
  app_->Options()->SetIntegerValue("print_level", 0);
  // app_->Options()->SetStringValue("mu_strategy", "adaptive");
  // app_->Options()->SetStringValue("nlp_scaling_method", "none");
  app_->Options()->SetStringValue("fixed_variable_treatment",
                                  "make_parameter_nodual");
  app_->Options()->SetNumericValue("first_hessian_perturbation", 1);
  app_->Options()->SetNumericValue("ma57_pre_alloc", 2.0);
  app_->Options()->SetNumericValue("tol", 1e-3);
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
}

bool NMPCController::computeLegPlan(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::VectorXd &ref_ground_height,
    const double &first_element_duration, const bool &same_plan_index,
    Eigen::MatrixXd &state_traj, Eigen::MatrixXd &control_traj) {
  // Local planner will send a reference traj with N+1 rows
  mynlp_->update_solver(initial_state, ref_traj.bottomRows(N_), foot_positions,
                        contact_schedule, ref_ground_height.tail(N_),
                        first_element_duration, same_plan_index);

  // Once update, the initialization is done
  mynlp_->require_init_ = false;

  // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  // ROS_INFO_STREAM("initial_state");
  // ROS_INFO_STREAM(mynlp_->x_current_.transpose().format(CleanFmt));
  // ROS_INFO_STREAM("ref_traj");
  // ROS_INFO_STREAM(mynlp_->x_reference_.transpose().format(CleanFmt));
  // ROS_INFO_STREAM("foot_positions");
  // ROS_INFO_STREAM(mynlp_->feet_location_.transpose().format(CleanFmt));
  // ROS_INFO_STREAM("contact_sequence_");
  // ROS_INFO_STREAM(mynlp_->contact_sequence_.transpose().format(CleanFmt));
  // ROS_INFO_STREAM("ref_ground_height");
  // ROS_INFO_STREAM(mynlp_->ground_height_.transpose().format(CleanFmt));
  // ROS_INFO_STREAM("first_element_duration");
  // ROS_INFO_STREAM(mynlp_->first_element_duration_);
  // ROS_INFO_STREAM("same_plan_index");
  // ROS_INFO_STREAM(same_plan_index);
  // Eigen::Map<Eigen::MatrixXd> www(mynlp_->w0_.block(0, 0, N_ * (n_ + m_),
  // 0).data(), n_ + m_, N_); ROS_INFO_STREAM("w");
  // ROS_INFO_STREAM(www.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> ww(mynlp_->w0_.block(N_ * (n_ + m_), 0, 2 * N_
  // * n_, 0).data(), n_, 2 * N_); ROS_INFO_STREAM("slack");
  // ROS_INFO_STREAM(ww.transpose().format(CleanFmt));

  bool success = this->computePlan(initial_state, ref_traj, foot_positions,
                                   contact_schedule, state_traj, control_traj);

  // Eigen::Map<Eigen::MatrixXd> wwwww(mynlp_->w0_.block(0, 0, N_ * (n_ + m_),
  // 0).data(), n_ + m_, N_); ROS_INFO_STREAM("wsolve");
  // ROS_INFO_STREAM(wwwww.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> wwww(mynlp_->w0_.block(N_ * (n_ + m_), 0, 2 *
  // N_ * n_, 0).data(), n_, 2 * N_); ROS_INFO_STREAM("slacksolve");
  // ROS_INFO_STREAM(wwww.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> zwl(mynlp_->z_L0_.block(0, 0, N_ * (n_ + m_),
  // 0).data(), n_ + m_, N_); ROS_INFO_STREAM("zwl");
  // ROS_INFO_STREAM(zwl.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> zsl(mynlp_->z_L0_.block(N_ * (n_ + m_), 0, 2 *
  // N_ * n_, 0).data(), n_, 2 * N_); ROS_INFO_STREAM("zsl");
  // ROS_INFO_STREAM(zsl.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> zwu(mynlp_->z_U0_.block(0, 0, N_ * (n_ + m_),
  // 0).data(), n_ + m_, N_); ROS_INFO_STREAM("zwu");
  // ROS_INFO_STREAM(zwu.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> zsu(mynlp_->z_U0_.block(N_ * (n_ + m_), 0, 2 *
  // N_ * n_, 0).data(), n_, 2 * N_); ROS_INFO_STREAM("zsu");
  // ROS_INFO_STREAM(zsu.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> lambdaaaa(mynlp_->lambda0_.data(), n_ + 16,
  // N_); ROS_INFO_STREAM("lambdaaaa");
  // ROS_INFO_STREAM(lambdaaaa.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> gggg(mynlp_->g0_.data(), n_ + 16, N_);
  // ROS_INFO_STREAM("gggg");
  // ROS_INFO_STREAM(gggg.transpose().format(CleanFmt));

  // if (!success)
  // {
  //   throw std::exception();
  // }

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

  mynlp_->update_solver(
      initial_state_with_tail, ref_traj_with_tail.bottomRows(N_),
      foot_positions, contact_schedule, state_traj.bottomRows(N_), control_traj,
      ref_ground_height.tail(N_), first_element_duration, same_plan_index);

  // Once update, the initialization is done
  mynlp_->require_init_ = false;

  // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  // ROS_INFO_STREAM("initial_state");
  // ROS_INFO_STREAM(mynlp_->x_current_.transpose().format(CleanFmt));
  // ROS_INFO_STREAM("ref_traj");
  // ROS_INFO_STREAM(mynlp_->x_reference_.transpose().format(CleanFmt));
  // ROS_INFO_STREAM("foot_positions");
  // ROS_INFO_STREAM(mynlp_->feet_location_.transpose().format(CleanFmt));
  // ROS_INFO_STREAM("contact_sequence_");
  // ROS_INFO_STREAM(mynlp_->contact_sequence_.transpose().format(CleanFmt));
  // ROS_INFO_STREAM("state_traj");
  // ROS_INFO_STREAM(state_traj.format(CleanFmt));
  // ROS_INFO_STREAM("control_traj");
  // ROS_INFO_STREAM(control_traj.format(CleanFmt));
  // ROS_INFO_STREAM("ref_ground_height");
  // ROS_INFO_STREAM(mynlp_->ground_height_.transpose().format(CleanFmt));
  // ROS_INFO_STREAM("first_element_duration");
  // ROS_INFO_STREAM(mynlp_->first_element_duration_);
  // ROS_INFO_STREAM("same_plan_index");
  // ROS_INFO_STREAM(same_plan_index);
  // Eigen::Map<Eigen::MatrixXd> www(mynlp_->w0_.block(0, 0, N_ * (n_ + m_),
  // 0).data(), n_ + m_, N_); ROS_INFO_STREAM("w");
  // ROS_INFO_STREAM(www.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> ww(mynlp_->w0_.block(N_ * (n_ + m_), 0, 2 * N_
  // * n_, 0).data(), n_, 2 * N_); ROS_INFO_STREAM("slack");
  // ROS_INFO_STREAM(ww.transpose().format(CleanFmt));

  bool success = this->computePlan(
      initial_state_with_tail, ref_traj_with_tail, foot_positions,
      contact_schedule, state_traj_with_tail, control_traj_with_tail);

  tail_state_traj.leftCols(2) = state_traj_with_tail.block(0, 6, N_ + 1, 2);
  tail_state_traj.rightCols(2) = state_traj_with_tail.block(0, 14, N_ + 1, 2);

  tail_control_traj = control_traj_with_tail.leftCols(2);

  // Eigen::Map<Eigen::MatrixXd> wwwww(mynlp_->w0_.block(0, 0, N_ * (n_ + m_),
  // 0).data(), n_ + m_, N_); ROS_INFO_STREAM("wsolve");
  // ROS_INFO_STREAM(wwwww.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> wwww(mynlp_->w0_.block(N_ * (n_ + m_), 0, 2 *
  // N_ * n_, 0).data(), n_, 2 * N_); ROS_INFO_STREAM("slacksolve");
  // ROS_INFO_STREAM(wwww.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> zwl(mynlp_->z_L0_.block(0, 0, N_ * (n_ + m_),
  // 0).data(), n_ + m_, N_); ROS_INFO_STREAM("zwl");
  // ROS_INFO_STREAM(zwl.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> zsl(mynlp_->z_L0_.block(N_ * (n_ + m_), 0, 2 *
  // N_ * n_, 0).data(), n_, 2 * N_); ROS_INFO_STREAM("zsl");
  // ROS_INFO_STREAM(zsl.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> zwu(mynlp_->z_U0_.block(0, 0, N_ * (n_ + m_),
  // 0).data(), n_ + m_, N_); ROS_INFO_STREAM("zwu");
  // ROS_INFO_STREAM(zwu.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> zsu(mynlp_->z_U0_.block(N_ * (n_ + m_), 0, 2 *
  // N_ * n_, 0).data(), n_, 2 * N_); ROS_INFO_STREAM("zsu");
  // ROS_INFO_STREAM(zsu.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> lambdaaaa(mynlp_->lambda0_.data(), n_ + 16,
  // N_); ROS_INFO_STREAM("lambdaaaa");
  // ROS_INFO_STREAM(lambdaaaa.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> gggg(mynlp_->g0_.data(), n_ + 16, N_);
  // ROS_INFO_STREAM("gggg");
  // ROS_INFO_STREAM(gggg.transpose().format(CleanFmt));

  // if (!success)
  // {
  //   throw std::exception();
  // }

  return success;
}

// This is only used for test
bool NMPCController::computeDistributedTailFullPlan(
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

  mynlp_->update_solver(
      initial_state_with_tail, ref_traj_with_tail.bottomRows(N_),
      foot_positions, contact_schedule, state_traj.bottomRows(N_), control_traj,
      ref_ground_height.tail(N_), first_element_duration, same_plan_index);

  // Once update, the initialization is done
  mynlp_->require_init_ = false;

  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  ROS_INFO_STREAM("initial_state");
  ROS_INFO_STREAM(mynlp_->x_current_.transpose().format(CleanFmt));
  ROS_INFO_STREAM("ref_traj");
  ROS_INFO_STREAM(mynlp_->x_reference_.transpose().format(CleanFmt));
  ROS_INFO_STREAM("foot_positions");
  ROS_INFO_STREAM(mynlp_->feet_location_.transpose().format(CleanFmt));
  ROS_INFO_STREAM("contact_sequence_");
  ROS_INFO_STREAM(mynlp_->contact_sequence_.transpose().format(CleanFmt));
  ROS_INFO_STREAM("state_traj");
  ROS_INFO_STREAM(state_traj.format(CleanFmt));
  ROS_INFO_STREAM("control_traj");
  ROS_INFO_STREAM(control_traj.format(CleanFmt));
  // ROS_INFO_STREAM("ref_ground_height");
  // ROS_INFO_STREAM(mynlp_->ground_height_.transpose().format(CleanFmt));
  // ROS_INFO_STREAM("first_element_duration");
  // ROS_INFO_STREAM(mynlp_->first_element_duration_);
  // ROS_INFO_STREAM("same_plan_index");
  // ROS_INFO_STREAM(same_plan_index);
  // Eigen::Map<Eigen::MatrixXd> www(mynlp_->w0_.block(0, 0, N_ * (n_ + m_),
  // 0).data(), n_ + m_, N_); ROS_INFO_STREAM("w");
  // ROS_INFO_STREAM(www.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> ww(mynlp_->w0_.block(N_ * (n_ + m_), 0, 2 * N_
  // * n_, 0).data(), n_, 2 * N_); ROS_INFO_STREAM("slack");
  // ROS_INFO_STREAM(ww.transpose().format(CleanFmt));

  bool success = this->computePlan(
      initial_state_with_tail, ref_traj_with_tail, foot_positions,
      contact_schedule, state_traj_with_tail, control_traj_with_tail);

  tail_state_traj = state_traj_with_tail;
  tail_control_traj = control_traj_with_tail;

  Eigen::Map<Eigen::MatrixXd> wwwww(
      mynlp_->w0_.block(0, 0, N_ * (n_ + m_), 0).data(), n_ + m_, N_);
  ROS_INFO_STREAM("wsolve");
  ROS_INFO_STREAM(wwwww.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> wwww(mynlp_->w0_.block(N_ * (n_ + m_), 0, 2 *
  // N_ * n_, 0).data(), n_, 2 * N_); ROS_INFO_STREAM("slacksolve");
  // ROS_INFO_STREAM(wwww.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> zwl(mynlp_->z_L0_.block(0, 0, N_ * (n_ + m_),
  // 0).data(), n_ + m_, N_); ROS_INFO_STREAM("zwl");
  // ROS_INFO_STREAM(zwl.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> zsl(mynlp_->z_L0_.block(N_ * (n_ + m_), 0, 2 *
  // N_ * n_, 0).data(), n_, 2 * N_); ROS_INFO_STREAM("zsl");
  // ROS_INFO_STREAM(zsl.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> zwu(mynlp_->z_U0_.block(0, 0, N_ * (n_ + m_),
  // 0).data(), n_ + m_, N_); ROS_INFO_STREAM("zwu");
  // ROS_INFO_STREAM(zwu.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> zsu(mynlp_->z_U0_.block(N_ * (n_ + m_), 0, 2 *
  // N_ * n_, 0).data(), n_, 2 * N_); ROS_INFO_STREAM("zsu");
  // ROS_INFO_STREAM(zsu.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> lambdaaaa(mynlp_->lambda0_.data(), n_ + 16,
  // N_); ROS_INFO_STREAM("lambdaaaa");
  // ROS_INFO_STREAM(lambdaaaa.transpose().format(CleanFmt));
  // Eigen::Map<Eigen::MatrixXd> gggg(mynlp_->g0_.data(), n_ + 16, N_);
  // ROS_INFO_STREAM("gggg");
  // ROS_INFO_STREAM(gggg.transpose().format(CleanFmt));

  // if (!success)
  // {
  //   throw std::exception();
  // }

  return success;
}

bool NMPCController::computePlan(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions,
    const std::vector<std::vector<bool>> &contact_schedule,
    Eigen::MatrixXd &state_traj, Eigen::MatrixXd &control_traj) {
  ApplicationReturnStatus status;

  // Set initial barrier parameter for warmstart
  app_->Options()->SetNumericValue("mu_init", mynlp_->mu0_);

  // Set warmstart
  if (mynlp_->warm_start_) {
    app_->Options()->SetStringValue("warm_start_init_point", "yes");
  } else {
    app_->Options()->SetStringValue("warm_start_init_point", "no");
  }

  status = app_->OptimizeTNLP(mynlp_);

  if (status == Solve_Succeeded) {
    mynlp_->warm_start_ = true;

    Eigen::MatrixXd x(n_, N_);
    Eigen::MatrixXd u(m_, N_);

    for (int i = 0; i < N_; ++i) {
      u.block(0, i, m_, 1) = mynlp_->w0_.block(i * (n_ + m_), 0, m_, 1);
      x.block(0, i, n_, 1) = mynlp_->w0_.block(i * (n_ + m_) + m_, 0, n_, 1);
    }

    // Convert solution back to local planner structure
    state_traj = Eigen::MatrixXd::Zero(N_ + 1, n_);
    state_traj.topRows(1) = initial_state.transpose();
    control_traj = Eigen::MatrixXd::Zero(N_, m_);

    state_traj.bottomRows(N_) = x.transpose();
    control_traj = u.transpose();

    return true;
  } else {
    mynlp_->mu0_ = 1e-1;
    mynlp_->warm_start_ = false;
    mynlp_->require_init_ = true;

    ROS_WARN_STREAM(param_ns_ << " solving fail");

    return false;
  }
}
