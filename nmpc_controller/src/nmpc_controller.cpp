#include "nmpc_controller/nmpc_controller.h"

NMPCController::NMPCController(int type) {
  type_ = type;

  switch (type_) {
    case 0:
      // Leg controller
      param_ns_ = "leg";
      break;
    case 1:
      // Leg controller
      param_ns_ = "leg_with_feet";
      break;
    case 2:
      // Centralized tail controller
      param_ns_ = "centralized_tail";
      break;
    case 3:
      // Distributed tail controller
      param_ns_ = "distributed_tail";
      break;
    case 4:
      // Decentralized tail controller
      param_ns_ = "decentralized_tail";
      break;
    default:
      param_ns_ = "leg";
      break;
  }

  // Load MPC/system parameters
  double mu;
  if (param_ns_ == "leg" || param_ns_ == "leg_with_feet") {
    ros::param::get("/local_planner/horizon_length", N_);
    ros::param::get("/local_planner/timestep", dt_);
  } else {
    ros::param::get("/nmpc_controller/" + param_ns_ + "/horizon_length", N_);
    ros::param::get("/nmpc_controller/" + param_ns_ + "/step_length", dt_);
  }
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_dimension", n_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_dimension", m_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/friction_coefficient",
                  mu);

  // Load MPC cost weighting and bounds
  std::vector<double> state_weights, control_weights, state_lower_bound,
      state_upper_bound, control_lower_bound, control_upper_bound;
  std::vector<int> fixed_complex_idxs;
  double panic_weights, constraint_panic_weights, Q_temporal_factor,
      R_temporal_factor;
  ;
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_weights",
                  state_weights);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_weights",
                  control_weights);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/panic_weights",
                  panic_weights);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/constraint_panic_weights",
                  constraint_panic_weights);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/Q_temporal_factor",
                  Q_temporal_factor);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/R_temporal_factor",
                  R_temporal_factor);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_lower_bound",
                  state_lower_bound);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_upper_bound",
                  state_upper_bound);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_lower_bound",
                  control_lower_bound);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_upper_bound",
                  control_upper_bound);

  int fixed_complex_head, fixed_complex_tail;
  std::vector<double> state_lower_bound_null_hard, state_upper_bound_null_hard,
      state_lower_bound_null_soft, state_upper_bound_null_soft;
  ros::param::get("/nmpc_controller/leg_complex/null_space_dimension", n_null_);
  ros::param::get("/nmpc_controller/leg_complex/state_lower_bound_hard",
                  state_lower_bound_null_hard);
  ros::param::get("/nmpc_controller/leg_complex/state_upper_bound_hard",
                  state_upper_bound_null_hard);
  ros::param::get("/nmpc_controller/leg_complex/state_lower_bound_soft",
                  state_lower_bound_null_soft);
  ros::param::get("/nmpc_controller/leg_complex/state_upper_bound_soft",
                  state_upper_bound_null_soft);
  ros::param::get("/nmpc_controller/leg_complex/fixed_complex_idxs",
                  fixed_complex_idxs);
  ros::param::get("/nmpc_controller/leg_complex/fixed_complex_head",
                  fixed_complex_head);
  ros::param::get("/nmpc_controller/leg_complex/fixed_complex_tail",
                  fixed_complex_tail);
  ros::param::get("/nmpc_controller/takeoff_state_weight_factor",
                  takeoff_state_weight_factor_);

  Q_temporal_factor = std::pow(Q_temporal_factor, 1.0 / (N_ - 2));
  R_temporal_factor = std::pow(R_temporal_factor, 1.0 / (N_ - 2));

  Eigen::Map<Eigen::VectorXd> Q(state_weights.data(), n_),
      R(control_weights.data(), m_), x_min(state_lower_bound.data(), n_),
      x_max(state_upper_bound.data(), n_),
      x_min_null_hard(state_lower_bound_null_hard.data(),
                      state_lower_bound_null_hard.size()),
      x_max_null_hard(state_upper_bound_null_hard.data(),
                      state_upper_bound_null_hard.size()),
      x_min_null_soft(state_lower_bound_null_soft.data(),
                      state_lower_bound_null_soft.size()),
      x_max_null_soft(state_upper_bound_null_soft.data(),
                      state_upper_bound_null_soft.size()),
      u_min(control_lower_bound.data(), m_),
      u_max(control_upper_bound.data(), m_);

  // Load fixed complexity schedule
  Eigen::VectorXi fixed_complexity_schedule(N_);
  fixed_complexity_schedule.setZero();
  adaptive_complexity_schedule_ = fixed_complexity_schedule;
  for (int idx : fixed_complex_idxs) {
    if (idx >= 0 && idx <= N_) {
      fixed_complexity_schedule[idx] = 1;
    }
  }
  if (fixed_complex_head > 0) {
    fixed_complexity_schedule.head(std::min(fixed_complex_head, N_)).fill(1);
  }
  if (fixed_complex_tail > 0) {
    fixed_complexity_schedule.tail(std::min(fixed_complex_tail, N_)).fill(1);
  }
  std::cout << "fixed_complexity_schedule = "
            << fixed_complexity_schedule.transpose() << std::endl;

  Eigen::VectorXd x_min_complex_hard(n_ + n_null_),
      x_max_complex_hard(n_ + n_null_), x_min_complex_soft(n_ + n_null_),
      x_max_complex_soft(n_ + n_null_);
  x_min_complex_hard.segment(0, n_) = x_min;
  x_min_complex_hard.segment(n_, n_null_) = x_min_null_hard;
  x_max_complex_hard.segment(0, n_) = x_max;
  x_max_complex_hard.segment(n_, n_null_) = x_max_null_hard;
  x_min_complex_soft.segment(0, n_) = x_min;
  x_min_complex_soft.segment(n_, n_null_) = x_min_null_soft;
  x_max_complex_soft.segment(0, n_) = x_max;
  x_max_complex_soft.segment(n_, n_null_) = x_max_null_soft;

  mynlp_ = new quadNLP(
      type_, N_, n_, n_null_, m_, dt_, mu, panic_weights,
      constraint_panic_weights, Q, R, Q_temporal_factor, R_temporal_factor,
      x_min, x_max, x_min_complex_hard, x_max_complex_hard, x_min_complex_soft,
      x_max_complex_soft, u_min, u_max, fixed_complexity_schedule);

  app_ = IpoptApplicationFactory();

  // app_->Options()->SetIntegerValue("max_iter", 100);
  // app_->Options()->SetStringValue("print_timing_statistics", "yes");
  app_->Options()->SetStringValue("linear_solver", "ma57");
  app_->Options()->SetIntegerValue("print_level", 5);
  app_->Options()->SetNumericValue("ma57_pre_alloc", 1.5);
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

  app_->Options()->SetNumericValue("max_wall_time", 40.0 * dt_);
  app_->Options()->SetNumericValue("max_cpu_time", 40.0 * dt_);

  ApplicationReturnStatus status;
  status = app_->Initialize();
  if (status != Solve_Succeeded) {
    std::cout << std::endl
              << std::endl
              << "*** Error during NMPC initialization!" << std::endl;
    return;
  }

  require_init_ = true;

  quadKD_ = std::make_shared<quad_utils::QuadKD>();
}

bool NMPCController::computeLegPlan(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    Eigen::MatrixXd &foot_positions, Eigen::MatrixXd &foot_velocities,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::VectorXd &ref_ground_height,
    const double &first_element_duration, const bool &same_plan_index,
    const grid_map::GridMap &terrain, Eigen::MatrixXd &state_traj,
    Eigen::MatrixXd &control_traj) {
  // Local planner will send a reference traj with N+1 rows
  mynlp_->foot_pos_world_ = foot_positions;
  mynlp_->foot_vel_world_ = foot_velocities;
  mynlp_->terrain_ = terrain;
  mynlp_->update_solver(initial_state, ref_traj, foot_positions,
                        contact_schedule, adaptive_complexity_schedule_,
                        ref_ground_height, first_element_duration,
                        same_plan_index, require_init_);
  require_init_ = false;

  bool success = this->computePlan(initial_state, ref_traj, foot_positions,
                                   contact_schedule, state_traj, control_traj);

  foot_positions = state_traj.middleCols(mynlp_->n_body_, mynlp_->n_foot_ / 2);
  foot_velocities = state_traj.rightCols(mynlp_->n_foot_ / 2);
  state_traj.conservativeResize(N_, mynlp_->n_body_);

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
  // mynlp_->mu0_ = 0.1;
  // mynlp_->warm_start_ = false;
  app_->Options()->SetNumericValue("mu_init", mynlp_->mu0_);
  if (mynlp_->warm_start_) {
    // std::cout << "Warm start on" << std::endl;
    app_->Options()->SetStringValue("warm_start_init_point", "yes");
  } else {
    // std::cout << "Warm start off" << std::endl;
    app_->Options()->SetStringValue("warm_start_init_point", "no");
  }

  quad_utils::FunctionTimer timer("solve");
  status = app_->OptimizeTNLP(mynlp_);
  double t_solve = timer.reportSilent();

  state_traj = Eigen::MatrixXd::Zero(N_, n_);
  Eigen::MatrixXd state_null_traj = Eigen::MatrixXd::Zero(N_, n_null_);
  Eigen::MatrixXd state_null_traj_lift = Eigen::MatrixXd::Zero(N_, n_null_);
  control_traj = Eigen::MatrixXd::Zero(N_ - 1, m_);

  state_traj.row(0) =
      mynlp_->get_primal_state_var(mynlp_->w0_, 0).head(n_).transpose();

  for (int i = 1; i < N_; ++i) {
    control_traj.row(i - 1) =
        mynlp_->get_primal_control_var(mynlp_->w0_, i - 1).transpose();
    state_traj.row(i) =
        mynlp_->get_primal_state_var(mynlp_->w0_, i).head(n_).transpose();
  }

  // if (status == Solve_Succeeded && t_solve < 5.0 * dt_) {
  if (status == Solve_Succeeded) {
    mynlp_->warm_start_ = true;
    // Eigen::VectorXd constr_vars = evalLiftedTrajectoryConstraints();

    // std::cout << "current body state = \n"
    //           << mynlp_->x_current_.segment(0, n_).transpose() << std::endl;
    // std::cout << "current joint pos = \n"
    //           << mynlp_->x_current_.segment(n_, n_null_ / 2).transpose()
    //           << std::endl;
    // std::cout
    //     << "current joint vel = \n"
    //     << mynlp_->x_current_.segment(N_ + n_null_ / 2, n_null_ /
    //     2).transpose()
    //     << std::endl;
    // std::cout << "x_reference_ = \n"
    //           << mynlp_->x_reference_.transpose() << std::endl;
    // std::cout << "state_traj body = \n"
    //           << state_traj.leftCols(mynlp_->n_body_) << std::endl;

    // std::cout << "state_traj foot pos = \n"
    //           << state_traj.middleCols(mynlp_->n_body_, mynlp_->n_foot_ / 2)
    //           << std::endl;
    // std::cout << "state_traj foot vel = \n"
    //           << state_traj.rightCols(mynlp_->n_foot_ / 2) << std::endl;

    // std::cout << "control_traj body = \n"
    //           << control_traj.leftCols(mynlp_->m_body_) << std::endl;
    // std::cout << "control_traj foot = \n"
    //           << control_traj.rightCols(mynlp_->m_foot_) << std::endl;

    // std::cout << "foot_positions = \n" << mynlp_->foot_pos_world_ <<
    // std::endl;
    // std::cout << "foot_velocities = \n" << mynlp_->foot_vel_world_ <<
    // std::endl;
    // std::cout << "foot_pos error = \n"
    //           << (state_traj.middleCols(mynlp_->n_body_, mynlp_->n_foot_ / 2)
    //           -
    //               mynlp_->foot_pos_world_)
    //           << std::endl;
    // std::cout << "foot_vel error = \n"
    //           << (state_traj.rightCols(mynlp_->n_foot_ / 2) -
    //               mynlp_->foot_vel_world_)
    //           << std::endl
    //           << std::endl;

    // std::cout << "joint_positions = \n"
    //           << state_null_traj.leftCols(n_null_ / 2) << std::endl;
    // std::cout << "joint_velocities = \n"
    //           << state_null_traj.rightCols(n_null_ / 2) << std::endl;

    return true;
  } else {
    mynlp_->mu0_ = 1e-1;
    mynlp_->warm_start_ = false;
    require_init_ = true;

    ROS_WARN_STREAM(param_ns_ << " solving fail");
    if (t_solve >= 5.0 * dt_) {
      ROS_WARN_STREAM("timeout");
    }

    Eigen::VectorXi constr_vals =
        evalLiftedTrajectoryConstraints(state_null_traj);

    std::cout << "current body state = \n"
              << mynlp_->x_current_.segment(0, n_).transpose() << std::endl;
    std::cout << "current joint pos = \n"
              << mynlp_->x_current_.segment(n_, n_null_ / 2).transpose()
              << std::endl;
    std::cout
        << "current joint vel = \n"
        << mynlp_->x_current_.segment(N_ + n_null_ / 2, n_null_ / 2).transpose()
        << std::endl;
    std::cout << "x_reference_ = \n"
              << mynlp_->x_reference_.transpose() << std::endl;
    std::cout << "state_traj body = \n"
              << state_traj.leftCols(mynlp_->n_body_) << std::endl;

    std::cout << "state_traj foot pos = \n"
              << state_traj.middleCols(mynlp_->n_body_, mynlp_->n_foot_ / 2)
              << std::endl;
    std::cout << "state_traj foot vel = \n"
              << state_traj.rightCols(mynlp_->n_foot_ / 2) << std::endl;

    std::cout << "control_traj body = \n"
              << control_traj.leftCols(mynlp_->m_body_) << std::endl;
    std::cout << "control_traj foot = \n"
              << control_traj.rightCols(mynlp_->m_foot_) << std::endl;

    std::cout << "foot_positions = \n" << mynlp_->foot_pos_world_ << std::endl;
    std::cout << "foot_velocities = \n" << mynlp_->foot_vel_world_ << std::endl;
    std::cout << "foot_pos error = \n"
              << (state_traj.middleCols(mynlp_->n_body_, mynlp_->n_foot_ / 2) -
                  mynlp_->foot_pos_world_)
              << std::endl;
    std::cout << "foot_vel error = \n"
              << (state_traj.rightCols(mynlp_->n_foot_ / 2) -
                  mynlp_->foot_vel_world_)
              << std::endl
              << std::endl;

    std::cout << "joint_positions = \n"
              << state_null_traj.leftCols(n_null_ / 2) << std::endl;
    std::cout << "joint_velocities = \n"
              << state_null_traj.rightCols(n_null_ / 2) << std::endl;

    throw std::runtime_error("Solve failed, exiting for debug");
    return false;
  }
}

Eigen::VectorXi NMPCController::evalLiftedTrajectoryConstraints(
    Eigen::MatrixXd &state_null_traj) {
  // quad_utils::FunctionTimer timer(__FUNCTION__);

  // Declare decision and constraint vars
  Eigen::VectorXi adaptive_complexity_schedule(N_);
  adaptive_complexity_schedule.setZero();

  Eigen::VectorXd max_constraint_violation(N_);
  max_constraint_violation.fill(-2e19);
  double max_constraint_violation_val = -2e19;
  int max_constraint_violation_fe = -1;
  int max_constraint_violation_index = -1;

  Eigen::VectorXd x0, u, x1;
  Eigen::VectorXd joint_positions(12), joint_velocities(12), joint_torques(12);
  Eigen::VectorXd constr_vals, params(12), lb_violation, ub_violation;
  bool valid_solve = true;
  bool valid_lift = true;

  // Load current state data
  x0 = mynlp_->get_primal_state_var(mynlp_->w0_, 0);

  Eigen::VectorXd x0_body, u_body;
  x0_body = x0.head(mynlp_->n_body_);

  if (x0.size() < mynlp_->n_complex_) {
    quadKD_->convertCentroidalToFullBody(
        x0_body, mynlp_->foot_pos_world_.row(0), mynlp_->foot_vel_world_.row(0),
        mynlp_->get_primal_body_control_var(mynlp_->w0_, 0), joint_positions,
        joint_velocities, joint_torques);
    x0.conservativeResize(mynlp_->n_complex_);
    x0.segment(n_, n_null_ / 2) = joint_positions;
    x0.segment(n_ + n_null_ / 2, n_null_ / 2) = joint_velocities;
  }
  state_null_traj.row(0) = x0.segment(n_, n_null_);

  double var_tol, constr_tol;
  app_->Options()->GetNumericValue("tol", var_tol, "");
  app_->Options()->GetNumericValue("constr_viol_tol", constr_tol, "");

  // Loop through trajectory, lifting as needed and evaluating constraints
  for (int i = 0; i < N_ - 1; i++) {
    // std::cout << "FE " << i << ", sys_id = " << mynlp_->sys_id_schedule_[i]
    //           << std::endl;

    u = mynlp_->get_primal_control_var(mynlp_->w0_, i);
    x1 = mynlp_->get_primal_state_var(mynlp_->w0_, i + 1);

    if (x1.size() < mynlp_->n_complex_) {
      // std::cout << "state is lifted" << std::endl;
      quadKD_->convertCentroidalToFullBody(
          x1.head(mynlp_->n_body_), mynlp_->foot_pos_world_.row(i + 1),
          mynlp_->foot_vel_world_.row(i + 1), u.head(mynlp_->n_body_),
          joint_positions, joint_velocities, joint_torques);
      x1.conservativeResize(mynlp_->n_complex_);
      x1.segment(n_, n_null_ / 2) = joint_positions;
      x1.segment(n_ + n_null_ / 2, n_null_ / 2) = joint_velocities;
    }

    state_null_traj.row(i + 1) = x1.segment(n_, n_null_);

    double dt = (i == 0) ? mynlp_->first_element_duration_ : dt_;
    params = mynlp_->foot_pos_world_.row(i + 1);

    constr_vals = mynlp_->eval_g_single_fe(COMPLEX, dt, x0, u, x1, params);
    lb_violation = mynlp_->g_min_complex_hard_ - constr_vals;
    ub_violation = constr_vals - mynlp_->g_max_complex_hard_;

    max_constraint_violation[i + 1] =
        (lb_violation.cwiseMax(ub_violation)).maxCoeff();
    // for (int j = 0;
    //      j < mynlp_->relaxed_primal_constraint_idxs_in_element_.size(); j++)
    //      {
    //   std::cout
    //       << "knee " << j << " height:"
    //       <<
    //       -constr_vals[mynlp_->relaxed_primal_constraint_idxs_in_element_[j]]
    //       << std::endl;
    // }
    // if (i + 1 >= 11 && i + 1 <= 13) {
    //   std::cout << "i = " << i + 1 << std::endl;
    //   std::cout << "body pos = " << x1.segment(0, 12).transpose() <<
    //   std::endl; std::cout << "joint pos = " << x1.segment(12,
    //   12).transpose()
    //             << std::endl;
    //   std::cout << "joint vel = " << x1.segment(24, 12).transpose()
    //             << std::endl;
    // }
    // std::cout << "foot_pos = " << mynlp_->foot_pos_world_.row(i + 1)
    //           << std::endl;
    // std::cout << "foot_vel = " << mynlp_->foot_vel_world_.row(i + 1)
    //           << std::endl;
    // std::cout << "grfs = " << u.transpose() << std::endl;
    // std::cout << "lb violation = " << lb_violation << std::endl;
    // std::cout << "ub violation = " << ub_violation << std::endl;

    for (int j = 0; j < constr_vals.size(); j++) {
      double current_constraint_violation =
          std::max(lb_violation[j], ub_violation[j]);

      if (current_constraint_violation > max_constraint_violation_val) {
        max_constraint_violation_val = current_constraint_violation;
        max_constraint_violation_fe = i + 1;
        max_constraint_violation_index = j;
      }

      if (current_constraint_violation > constr_tol) {
        if (mynlp_->n_vec_[i + 1] == mynlp_->n_complex_) {
          printf(
              "Constraint %s violated in FE %d: %5.3f <= %5.3f <= "
              "%5.3f\n",
              mynlp_->constr_names_[COMPLEX][j].c_str(), i,
              mynlp_->g_min_complex_hard_[j] - constr_tol, constr_vals[j],
              mynlp_->g_max_complex_hard_[j] + constr_tol);

          // std::cout << "x0 = \n" << x0 << std::endl;
          // std::cout << "u = \n" << u << std::endl;
          // std::cout << "x1 = \n" << x1 << std::endl;
          // std::cout << "constr_vals = \n" << constr_vals << std::endl;

          valid_solve = false;
        } else {
          valid_lift = false;
        }
        adaptive_complexity_schedule[i + 1] = 1;
      }
    }

    for (int j = 0; j < x1.size(); j++) {
      if (x1[j] < mynlp_->x_min_complex_hard_[j] - var_tol ||
          x1[j] > mynlp_->x_max_complex_hard_[j] + var_tol) {
        if (mynlp_->n_vec_[i + 1] == mynlp_->n_complex_) {
          printf(
              "Var bound %d violated in FE %d: %5.3f <= %5.3f <= "
              "%5.3f\n",
              j, i, mynlp_->x_min_complex_hard_[j], x1[j] - var_tol,
              mynlp_->x_max_complex_hard_[j] + var_tol);

          valid_solve = false;
        } else {
          valid_lift = false;
        }
        adaptive_complexity_schedule[i + 1] = 1;
      }
    }
    x0 = x1;
  }

  if (!valid_lift) {
    std::cout << "Invalid: " << adaptive_complexity_schedule.transpose()
              << std::endl;
    // mynlp_->warm_start_ = false;
    // mynlp_->mu0_ = 1e-1;
  }
  if (!valid_solve) {
    ROS_WARN(
        "Invalid region detected where there shouldnt be (possibly due to "
        "constraint relaxation).");
    // throw std::runtime_error(
    //     "Invalid region detected where there shouldnt be, exiting.");
  }

  if (max_constraint_violation_val >= constr_tol) {
    // std::cout << "Max violation vector = "
    //           << max_constraint_violation.transpose() << std::endl;
    std::cout << "Max violation is constraint "
              << mynlp_->constr_names_[COMPLEX][max_constraint_violation_index]
              << " at FE " << max_constraint_violation_fe
              << " with val = " << max_constraint_violation_val << std::endl;
  }
  // timer.reportStatistics();
  // return constr_vals;
  return adaptive_complexity_schedule;
}
