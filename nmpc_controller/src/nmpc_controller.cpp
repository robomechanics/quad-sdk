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
  if (param_ns_ == "leg") {
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
  std::vector<double> state_weights, control_weights, state_weights_factors,
      control_weights_factors, state_lower_bound, state_upper_bound,
      state_lower_bound_null, state_upper_bound_null, control_lower_bound,
      control_upper_bound;
  std::vector<int> fixed_complex_idxs;
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

  int fixed_complex_head, fixed_complex_tail;
  ros::param::get("/nmpc_controller/leg_complex/null_space_dimension", n_null_);
  ros::param::get("/nmpc_controller/leg_complex/state_lower_bound",
                  state_lower_bound_null);
  ros::param::get("/nmpc_controller/leg_complex/state_upper_bound",
                  state_upper_bound_null);
  ros::param::get("/nmpc_controller/leg_complex/fixed_complex_idxs",
                  fixed_complex_idxs);
  ros::param::get("/nmpc_controller/leg_complex/fixed_complex_head",
                  fixed_complex_head);
  ros::param::get("/nmpc_controller/leg_complex/fixed_complex_tail",
                  fixed_complex_tail);
  ros::param::get("/nmpc_controller/takeoff_state_weight_factor",
                  takeoff_state_weight_factor_);

  Eigen::Map<Eigen::VectorXd> Q(state_weights.data(), n_),
      R(control_weights.data(), m_), Q_factor(state_weights_factors.data(), N_),
      R_factor(control_weights_factors.data(), N_ - 1),
      x_min(state_lower_bound.data(), n_), x_max(state_upper_bound.data(), n_),
      x_min_null(state_lower_bound_null.data(), state_lower_bound_null.size()),
      x_max_null(state_upper_bound_null.data(), state_upper_bound_null.size()),
      u_min(control_lower_bound.data(), m_),
      u_max(control_upper_bound.data(), m_);

  // Load fixed complexity schedule
  Eigen::VectorXi fixed_complexity_schedule(N_);
  fixed_complexity_schedule.setZero();
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

  // TODO(jcnorby) add back in
  Eigen::VectorXd x_min_complex(n_ + n_null_), x_max_complex(n_ + n_null_);
  x_min_complex.segment(0, n_) = x_min;
  x_min_complex.segment(n_, n_null_) = x_min_null;
  x_max_complex.segment(0, n_) = x_max;
  x_max_complex.segment(n_, n_null_) = x_max_null;

  mynlp_ = new quadNLP(type_, N_, n_, n_null_, m_, dt_, mu, panic_weights, Q, R,
                       Q_factor, R_factor, x_min, x_max, x_min_complex,
                       x_max_complex, u_min, u_max, fixed_complexity_schedule);

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

  app_->Options()->SetNumericValue("max_wall_time", 4.0 * dt_);
  app_->Options()->SetNumericValue("max_cpu_time", 4.0 * dt_);

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
    const Eigen::MatrixXd &foot_positions,
    const Eigen::MatrixXd &foot_velocities,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::VectorXd &ref_ground_height,
    const double &first_element_duration, const bool &same_plan_index,
    const Eigen::VectorXi &ref_primitive_id,
    const Eigen::VectorXi &complexity_schedule, Eigen::MatrixXd &state_traj,
    Eigen::MatrixXd &control_traj) {
  // Local planner will send a reference traj with N+1 rows
  mynlp_->update_solver(initial_state, ref_traj, foot_positions,
                        contact_schedule, complexity_schedule,
                        ref_ground_height, first_element_duration,
                        same_plan_index, require_init_);
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

  // std::cout << "New solve" << std::endl;
  // std::cout << "ref_traj.size = " << ref_traj.rows() << ", " <<
  // ref_traj.cols()
  //           << std::endl;
  // std::cout << "ref_traj = \n" << ref_traj << std::endl;
  // std::cout << "mynlp_->foot_pos_world_.size = "
  //           << mynlp_->foot_pos_world_.rows() << ", "
  //           << mynlp_->foot_pos_world_.cols() << std::endl;
  // std::cout << "mynlp_->foot_pos_world_ = \n"
  //           << mynlp_->foot_pos_world_ << std::endl;
  // std::cout << "mynlp_->foot_vel_world_.size = "
  //           << mynlp_->foot_vel_world_.rows() << ", "
  //           << mynlp_->foot_vel_world_.cols() << std::endl;
  // std::cout << "mynlp_->foot_vel_world_ = \n"
  //           << mynlp_->foot_vel_world_ << std::endl;
  // std::cout << "state_traj.size = " << state_traj.rows() << ", "
  //           << state_traj.cols() << std::endl;
  // std::cout << "state_traj = \n" << state_traj << std::endl;
  // std::cout << "control_traj.size = " << control_traj.rows() << ", "
  //           << control_traj.cols() << std::endl;
  // std::cout << "control_traj = \n" << control_traj << std::endl;

  // if (!success) {
  //   throw std::runtime_error("solving fail, stopping");
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

  status = app_->OptimizeTNLP(mynlp_);

  state_traj = Eigen::MatrixXd::Zero(N_, n_);
  Eigen::MatrixXd state_null_traj = Eigen::MatrixXd::Zero(N_, n_null_);
  Eigen::MatrixXd state_null_traj_lift = Eigen::MatrixXd::Zero(N_, n_null_);
  control_traj = Eigen::MatrixXd::Zero(N_ - 1, m_);

  state_traj.row(0) =
      mynlp_->get_state_var(mynlp_->w0_, 0).head(n_).transpose();

  // state_null_traj.row(0) =
  //     mynlp_->get_state_var(mynlp_->w0_, 0).tail(n_null_).transpose();

  // Eigen::VectorXd joint_positions(12), joint_velocities(12),
  // joint_torques(12);

  // quadKD_->convertCentroidalToFullBody(
  //     state_traj.row(0), foot_positions.row(0),
  //     mynlp_->foot_vel_world_.row(0), control_traj.row(0), joint_positions,
  //     joint_velocities, joint_torques);

  // state_null_traj_lift.row(0).head(12) = joint_positions;
  // state_null_traj_lift.row(0).tail(12) = joint_velocities;

  for (int i = 1; i < N_; ++i) {
    control_traj.row(i - 1) =
        mynlp_->get_control_var(mynlp_->w0_, i - 1).transpose();
    state_traj.row(i) =
        mynlp_->get_state_var(mynlp_->w0_, i).head(n_).transpose();

    // if (mynlp_->n_vec_[i + 1] > n_) {
    //   state_null_traj.row(i + 1) =
    //       mynlp_->get_state_var(mynlp_->w0_, i +
    //       1).tail(n_null_).transpose();
    // } else {
    //   quadKD_->convertCentroidalToFullBody(
    //       state_traj.row(i + 1), foot_positions.row(i + 1),
    //       mynlp_->foot_vel_world_.row(i + 1), control_traj.row(i),
    //       joint_positions, joint_velocities, joint_torques);

    //   state_null_traj.row(i + 1).head(12) = joint_positions;
    //   state_null_traj.row(i + 1).tail(12) = joint_velocities;
    // }

    // quadKD_->convertCentroidalToFullBody(
    //     state_traj.row(i + 1), mynlp_->foot_pos_world_.row(i + 1),
    //     mynlp_->foot_vel_world_.row(i + 1), control_traj.row(i),
    //     joint_positions, joint_velocities, joint_torques);

    // state_null_traj_lift.row(i + 1).head(12) = joint_positions;
    // state_null_traj_lift.row(i + 1).tail(12) = joint_velocities;
  }

  // std::cout << "state_traj = \n" << state_traj << std::endl;
  // std::cout << "control_traj = \n" << control_traj << std::endl;
  // std::cout << "state_null_traj pos = \n"
  //           << state_null_traj.leftCols(n_null_ / 2) << std::endl;
  // std::cout << "state_null_traj vel = \n"
  //           << state_null_traj.rightCols(n_null_ / 2) << std::endl;
  // std::cout << "state_null_traj_lift pos = \n"
  //           << state_null_traj_lift.leftCols(n_null_ / 2) << std::endl;
  // std::cout << "state_null_traj_lift vel = \n"
  //           << state_null_traj_lift.rightCols(n_null_ / 2) << std::endl;

  // std::cout << "state_null_traj pos diff = \n"
  //           << state_null_traj.leftCols(n_null_ / 2) -
  //                  state_null_traj_lift.leftCols(n_null_ / 2)
  //           << std::endl;

  // std::cout << "state_null_traj vel diff = \n"
  //           << state_null_traj.rightCols(n_null_ / 2) -
  //                  state_null_traj_lift.rightCols(n_null_ / 2)
  //           << std::endl;
  if (status == Solve_Succeeded) {
    mynlp_->warm_start_ = true;
    Eigen::VectorXd constr_vars = evalLiftedTrajectoryConstraints();

    return true;
  } else {
    mynlp_->mu0_ = 1e-1;
    mynlp_->warm_start_ = false;
    require_init_ = true;

    ROS_WARN_STREAM(param_ns_ << " solving fail");
    std::cout << "current body state = \n"
              << mynlp_->x_current_.segment(0, 12).transpose() << std::endl;
    std::cout << "current joint pos = \n"
              << mynlp_->x_current_.segment(12, 12).transpose() << std::endl;
    std::cout << "current joint vel = \n"
              << mynlp_->x_current_.segment(24, 12).transpose() << std::endl;
    std::cout << "x_reference_ = \n"
              << mynlp_->x_reference_.transpose() << std::endl;
    std::cout << "state_traj = \n" << state_traj << std::endl;
    std::cout << "control_traj = \n" << control_traj << std::endl;
    std::cout << "foot_positions = \n" << mynlp_->foot_pos_world_ << std::endl;
    std::cout << "foot_velocities = \n" << mynlp_->foot_vel_world_ << std::endl;

    throw std::runtime_error("Solve failed, exiting for debug");
    return false;
  }
}

Eigen::VectorXd NMPCController::evalLiftedTrajectoryConstraints() {
  // quad_utils::FunctionTimer timer(__FUNCTION__);

  // Declare decision and constraint vars
  Eigen::VectorXi adaptive_complexity_horizon(N_);
  adaptive_complexity_horizon.setZero();
  Eigen::VectorXd x0, u, x1;
  Eigen::VectorXd joint_positions(12), joint_velocities(12), joint_torques(12);
  Eigen::VectorXd constr_vars, params(24), lb_violation, ub_violation;
  bool valid_solve = true;
  bool valid_lift = true;

  // Load current state data
  x0 = mynlp_->get_state_var(mynlp_->w0_, 0);
  if (x0.size() < mynlp_->n_complex_) {
    quadKD_->convertCentroidalToFullBody(
        x0, mynlp_->foot_pos_world_.row(0), mynlp_->foot_vel_world_.row(0),
        mynlp_->get_control_var(mynlp_->w0_, 0), joint_positions,
        joint_velocities, joint_torques);
    x0.conservativeResize(mynlp_->n_complex_);
    x0.segment(12, 12) = joint_positions;
    x0.segment(24, 12) = joint_velocities;
  }
  double var_tol, constr_tol;
  app_->Options()->GetNumericValue("tol", var_tol, "");
  app_->Options()->GetNumericValue("constr_viol_tol", constr_tol, "");

  // Loop through trajectory, lifting as needed and evaluating constraints
  for (int i = 0; i < N_ - 1; i++) {
    u = mynlp_->get_control_var(mynlp_->w0_, i);
    x1 = mynlp_->get_state_var(mynlp_->w0_, i + 1);

    if (x1.size() < mynlp_->n_complex_) {
      quadKD_->convertCentroidalToFullBody(
          x1, mynlp_->foot_pos_world_.row(i + 1),
          mynlp_->foot_vel_world_.row(i + 1), u, joint_positions,
          joint_velocities, joint_torques);
      x1.conservativeResize(mynlp_->n_complex_);
      x1.segment(12, 12) = joint_positions;
      x1.segment(24, 12) = joint_velocities;
    }

    double dt = (i == 0) ? mynlp_->first_element_duration_ : dt_;
    params.head(12) = mynlp_->foot_pos_world_.row(i + 1);
    params.tail(12) = mynlp_->foot_vel_world_.row(i + 1);

    constr_vars = mynlp_->eval_g_single_fe(COMPLEX, dt, x0, u, x1, params);
    lb_violation = mynlp_->g_min_complex_ - constr_vars;
    ub_violation = constr_vars - mynlp_->g_max_complex_;
    // std::cout << "lb violation = " << lb_violation << std::endl;
    // std::cout << "ub violation = " << ub_violation << std::endl;

    for (int j = 0; j < constr_vars.size(); j++) {
      if (lb_violation[j] > constr_tol || ub_violation[j] > constr_tol) {
        if (mynlp_->n_vec_[i + 1] == mynlp_->n_complex_) {
          printf(
              "Constraint %s violated in FE %d: %5.3f <= %5.3f <= "
              "%5.3f\n",
              mynlp_->constr_names_[COMPLEX][j].c_str(), i,
              mynlp_->g_min_complex_[j] - constr_tol, constr_vars[j],
              mynlp_->g_max_complex_[j] + constr_tol);
          // std::cout << "x0 = \n" << x0 << std::endl;
          // std::cout << "u = \n" << u << std::endl;
          // std::cout << "x1 = \n" << x1 << std::endl;
          // std::cout << "constr_vars = \n" << constr_vars << std::endl;
          valid_solve = false;
        } else {
          valid_lift = false;
        }
        adaptive_complexity_horizon[i + 1] = 1;
      }
    }

    for (int j = 0; j < x1.size(); j++) {
      if (x1[j] < mynlp_->x_min_complex_[j] - var_tol ||
          x1[j] > mynlp_->x_max_complex_[j] + var_tol) {
        if (mynlp_->n_vec_[i + 1] == mynlp_->n_complex_) {
          printf(
              "Var bound %d violated in FE %d: %5.3f <= %5.3f <= "
              "%5.3f\n",
              j, i, mynlp_->x_min_complex_[j], x1[j] - var_tol,
              mynlp_->x_max_complex_[j] + var_tol);

          valid_solve = false;
        } else {
          valid_lift = false;
        }
      }
    }
    if (!valid_solve) {
      throw std::runtime_error(
          "Invalid region detected where there shouldnt be, exiting.");
    }
    x0 = x1;
  }

  if (!valid_lift) {
    std::cout << adaptive_complexity_horizon.transpose() << std::endl;
  }

  // timer.reportStatistics();
  return constr_vars;
}
