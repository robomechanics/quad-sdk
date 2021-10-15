#include "nmpc_controller/nmpc_controller.h"

NMPCController::NMPCController(ros::NodeHandle &nh, int robot_id) {
  nh_ = nh;
  robot_id_ = robot_id;
  SystemID default_system;

  switch (robot_id_) {
    case 0:
      robot_ns_ = "spirit";
      default_system = SPIRIT;
      break;
    case 1:
      robot_ns_ = "a1";
      default_system = A1;
      break;
    case 2:
      robot_ns_ = "new_platform";
      default_system = NEW_PLATFORM;
      break;
    default:
      robot_ns_ = "spirit";
      default_system = SPIRIT;
      break;
  }

  // Load rosparams from parameter server
  ros::param::get("nmpc_controller/" + param_ns_ + "/update_rate", update_rate_);

  // Load MPC/system parameters
  ros::param::get("/nmpc_controller/" + param_ns_ + "/horizon_length", N_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_dimension", n_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_dimension", m_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/step_length", dt_);

  // Load MPC cost weighting and bounds
  std::vector<double> state_weights,
      control_weights,
      state_lower_bound,
      state_upper_bound,
      control_lower_bound,
      control_upper_bound;
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_weights", state_weights);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_weights", control_weights);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_lower_bound", state_lower_bound);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_upper_bound", state_upper_bound);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_lower_bound", control_lower_bound);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_upper_bound", control_upper_bound);
  Eigen::Map<Eigen::MatrixXd> Q(state_weights.data(), n_, 1),
      R(control_weights.data(), m_, 1),
      x_min(state_lower_bound.data(), n_, 1),
      x_max(state_upper_bound.data(), n_, 1),
      u_min(control_lower_bound.data(), m_, 1),
      u_max(control_upper_bound.data(), m_, 1);

  // Convert kinematics
quadKD_ = std::make_shared<spirit_utils::QuadKD>();

  mynlp_ = new spiritNLP(
      type_,
      N_,
      n_,
      m_,
      dt_,
      Q,
      R,
      x_min,
      x_max,
      u_min,
      u_max);

    // Add to complex if specified
    if (components_in_complex[i]) {
      config_.x_dim_complex += x_dim;
      config_.u_dim_complex += u_dim;
      config_.g_dim_complex += g_dim;
      x_lb_complex.insert(x_lb_complex.end(), x_lb.begin(), x_lb.end());
      x_ub_complex.insert(x_ub_complex.end(), x_ub.begin(), x_ub.end());
      x_lb_complex_soft.insert(x_lb_complex_soft.end(), x_lb_soft.begin(),
                               x_lb_soft.end());
      x_ub_complex_soft.insert(x_ub_complex_soft.end(), x_ub_soft.begin(),
                               x_ub_soft.end());
      u_lb_complex.insert(u_lb_complex.end(), u_lb.begin(), u_lb.end());
      u_ub_complex.insert(u_ub_complex.end(), u_ub.begin(), u_ub.end());
      g_lb_complex.insert(g_lb_complex.end(), g_lb.begin(), g_lb.end());
      g_ub_complex.insert(g_ub_complex.end(), g_ub.begin(), g_ub.end());

      // Add to cost if specified
      if (components_in_cost[i]) {
        config_.x_dim_cost_complex += x_dim;
        config_.u_dim_cost_complex += u_dim;
        x_weights_complex.insert(x_weights_complex.end(), x_weights.begin(),
                                 x_weights.end());
        u_weights_complex.insert(u_weights_complex.end(), u_weights.begin(),
                                 u_weights.end());
      }
    }
  }
  config_.x_dim_null = config_.x_dim_complex - config_.x_dim_simple;
  config_.u_dim_null = config_.u_dim_complex - config_.u_dim_simple;

  // Load data from param vectors into config struct
  config_.Q_complex = Eigen::Map<Eigen::VectorXd>(x_weights_complex.data(),
                                                  config_.x_dim_cost_complex);
  config_.R_complex = Eigen::Map<Eigen::VectorXd>(u_weights_complex.data(),
                                                  config_.u_dim_cost_complex);
  config_.x_min_complex =
      Eigen::Map<Eigen::VectorXd>(x_lb_complex.data(), config_.x_dim_complex);
  config_.x_max_complex =
      Eigen::Map<Eigen::VectorXd>(x_ub_complex.data(), config_.x_dim_complex);
  config_.x_min_complex_soft = Eigen::Map<Eigen::VectorXd>(
      x_lb_complex_soft.data(), config_.x_dim_complex);
  config_.x_max_complex_soft = Eigen::Map<Eigen::VectorXd>(
      x_ub_complex_soft.data(), config_.x_dim_complex);
  config_.u_min_complex =
      Eigen::Map<Eigen::VectorXd>(u_lb_complex.data(), config_.u_dim_complex);
  config_.u_max_complex =
      Eigen::Map<Eigen::VectorXd>(u_ub_complex.data(), config_.u_dim_complex);
  config_.g_min_complex =
      Eigen::Map<Eigen::VectorXd>(g_lb_complex.data(), config_.g_dim_complex);
  config_.g_max_complex =
      Eigen::Map<Eigen::VectorXd>(g_ub_complex.data(), config_.g_dim_complex);

  // Construct fixed and adaptive complexity schedules
  Eigen::VectorXi fixed_complexity_schedule(N_);
  fixed_complexity_schedule.setZero();
  adaptive_complexity_schedule_ = fixed_complexity_schedule;
  ros::param::get("/nmpc_controller/enable_mixed_complexity",
                  enable_mixed_complexity_);

  // Adaptive complexity is only supported for Spirit
  if (robot_ns_ != "spirit") enable_mixed_complexity_ = false;

  // If mixed complexity is enabled, load the desired structures
  if (enable_mixed_complexity_) {
    default_system = SIMPLE_TO_SIMPLE;
    ros::param::get("/nmpc_controller/enable_adaptive_complexity",
                    enable_adaptive_complexity_);
    // Define and load adaptive complexity parameters
    std::vector<int> fixed_complex_idxs;
    int fixed_complex_head, fixed_complex_tail;

    ros::param::get("/nmpc_controller/fixed_complex_idxs", fixed_complex_idxs);
    ros::param::get("/nmpc_controller/fixed_complex_head", fixed_complex_head);
    ros::param::get("/nmpc_controller/fixed_complex_tail", fixed_complex_tail);
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
    std::cout << "Mixed complexity enabled, fixed schedule = "
              << fixed_complexity_schedule.transpose() << std::endl;
  }

  mynlp_ = new quadNLP(default_system, N_, dt_, mu, panic_weights,
                       constraint_panic_weights, Q_temporal_factor,
                       R_temporal_factor, fixed_complexity_schedule, config_);

  app_ = IpoptApplicationFactory();

  app_->Options()->SetStringValue("print_timing_statistics", "no");
  app_->Options()->SetStringValue("linear_solver", "ma27");
  app_->Options()->SetIntegerValue("print_level", 0); // default=0, verbose=5
  app_->Options()->SetNumericValue("ma57_pre_alloc", 1.5);
  app_->Options()->SetStringValue("fixed_variable_treatment",
                                  "make_parameter_nodual");
  app_->Options()->SetNumericValue("tol", 1e-3);
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
    const Eigen::MatrixXd &foot_positions_body,
    Eigen::MatrixXd &foot_positions_world,
    Eigen::MatrixXd &foot_velocities_world,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::VectorXd &ref_ground_height,
    const double &first_element_duration, int plan_index_diff,
    const grid_map::GridMap &terrain, Eigen::MatrixXd &state_traj,
    Eigen::MatrixXd &control_traj) {
  mynlp_->foot_pos_body_ = -foot_positions_body;
  mynlp_->foot_pos_world_ = foot_positions_world;
  mynlp_->foot_vel_world_ = foot_velocities_world;
  mynlp_->terrain_ = terrain;

  mynlp_->update_solver(initial_state, ref_traj, foot_positions_body,
                        contact_schedule, adaptive_complexity_schedule_,
                        ref_ground_height, first_element_duration,
                        plan_index_diff, require_init_);
  require_init_ = false;

  bool success = this->computePlan(initial_state, ref_traj, contact_schedule,
                                   foot_positions_world, foot_velocities_world,
                                   state_traj, control_traj);

  if (enable_variable_horizon_) updateHorizonLength();

  return success;
}

bool NMPCController::computePlan(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const std::vector<std::vector<bool>> &contact_schedule,
    Eigen::MatrixXd &foot_positions, Eigen::MatrixXd &foot_velocities,
    Eigen::MatrixXd &state_traj, Eigen::MatrixXd &control_traj) {
  // Update solver settings
  ApplicationReturnStatus status;
  app_->Options()->SetNumericValue("mu_init", mynlp_->mu0_);
  app_->Options()->SetStringValue("warm_start_init_point",
                                  ((mynlp_->warm_start_) ? "yes" : "no"));

  // Start timer for diagnostics and solve
  quad_utils::FunctionTimer timer("nlp_solver");
  status = app_->OptimizeTNLP(mynlp_);
  diagnostics_ = mynlp_->diagnostics_;
  diagnostics_.compute_time = timer.reportSilent();

  // Load the state and control trajectories
  state_traj = Eigen::MatrixXd::Zero(N_, config_.x_dim_simple);
  control_traj = Eigen::MatrixXd::Zero(N_ - 1, config_.u_dim_simple);
  state_traj.row(0) = mynlp_->get_primal_state_var(mynlp_->w0_, 0)
                          .head(config_.x_dim_simple)
                          .transpose();

  for (int i = 0; i < N_ - 1; ++i) {
    control_traj.row(i) = mynlp_->get_primal_control_var(mynlp_->w0_, i)
                              .head(config_.u_dim_simple)
                              .transpose();
    state_traj.row(i + 1) = mynlp_->get_primal_state_var(mynlp_->w0_, i + 1)
                                .head(config_.x_dim_simple)
                                .transpose();
  }

  // If mixed complexity is enabled, retrieve the lifted and heuristic
  // trajectories
  if (enable_mixed_complexity_) {
    Eigen::MatrixXd state_traj_heuristic(N_, config_.x_dim_complex);
    Eigen::MatrixXd control_traj_heuristic(N_ - 1, config_.u_dim_complex);
    mynlp_->get_heuristic_trajectory(state_traj_heuristic,
                                     control_traj_heuristic);

    Eigen::MatrixXd state_traj_lifted(N_, config_.x_dim_complex);
    Eigen::MatrixXd control_traj_lifted(N_ - 1, config_.u_dim_complex);
    mynlp_->get_lifted_trajectory(state_traj_lifted, control_traj_lifted);

    if (enable_adaptive_complexity_) {
      adaptive_complexity_schedule_ = updateAdaptiveComplexitySchedule(
          state_traj_heuristic, control_traj_heuristic, state_traj_lifted,
          control_traj_lifted);
    }

    foot_positions.topRows(state_traj_lifted.rows()) =
        state_traj_lifted.middleCols(n_body_, n_foot_ / 2);
    foot_velocities.topRows(state_traj_lifted.rows()) =
        state_traj_lifted.middleCols(n_body_ + n_foot_ / 2, n_foot_ / 2);
  }

    mynlp_->w0_.setZero();
    mynlp_->z_L0_.setZero();
    mynlp_->z_U0_.setZero();
    mynlp_->lambda0_.setZero();

    ROS_INFO_STREAM(param_ns_ << " solving fail");
    return false;
  }
}
