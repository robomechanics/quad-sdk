#include "nmpc_controller/nmpc_controller.h"

NMPCController::NMPCController() {
  // Load parameters set by local planner
  ros::param::get("/local_planner/horizon_length", N_);
  ros::param::get("/local_planner/timestep", dt_);

  // Load system parameters
  double mu, panic_weights, constraint_panic_weights, Q_temporal_factor,
      R_temporal_factor;
  ros::param::get("/nmpc_controller/friction_coefficient", mu);
  ros::param::get("/nmpc_controller/panic_weights", panic_weights);
  ros::param::get("/nmpc_controller/constraint_panic_weights",
                  constraint_panic_weights);
  ros::param::get("/nmpc_controller/Q_temporal_factor", Q_temporal_factor);
  ros::param::get("/nmpc_controller/R_temporal_factor", R_temporal_factor);
  Q_temporal_factor = std::pow(Q_temporal_factor, 1.0 / (N_ - 2));
  R_temporal_factor = std::pow(R_temporal_factor, 1.0 / (N_ - 2));

  // Determine whether to let horizon length vary or not
  ros::param::get("/nmpc_controller/enable_variable_horizon",
                  enable_variable_horizon_);
  ros::param::get("/nmpc_controller/min_horizon_length", N_min_);
  N_max_ = N_;

  // Define the components, their order, and which are simple
  std::vector<std::string> components = {"body", "feet", "joints"};
  std::vector<bool> components_in_simple = {true, false, false};
  std::vector<bool> components_in_complex = {true, true, true};
  std::vector<bool> components_in_cost = {true, true, false};

  std::vector<double> x_weights_complex, u_weights_complex, x_lb_complex,
      x_ub_complex, x_lb_complex_soft, x_ub_complex_soft, u_lb_complex,
      u_ub_complex, g_lb_complex, g_ub_complex;

  // Loop through each system component and read parameters
  for (int i = 0; i < components.size(); i++) {
    double x_dim, u_dim, g_dim;
    std::vector<double> x_weights, u_weights, x_lb, x_ub, u_lb, u_ub, g_lb,
        g_ub, x_lb_soft, x_ub_soft;

    // Read component parameters
    std::string component = components[i];
    ros::param::get("/nmpc_controller/" + component + "/x_dim", x_dim);
    ros::param::get("/nmpc_controller/" + component + "/u_dim", u_dim);
    ros::param::get("/nmpc_controller/" + component + "/g_dim", g_dim);
    ros::param::get("/nmpc_controller/" + component + "/x_weights", x_weights);
    ros::param::get("/nmpc_controller/" + component + "/u_weights", u_weights);
    ros::param::get("/nmpc_controller/" + component + "/x_lb", x_lb);
    ros::param::get("/nmpc_controller/" + component + "/x_ub", x_ub);
    ros::param::get("/nmpc_controller/" + component + "/x_lb_soft", x_lb_soft);
    ros::param::get("/nmpc_controller/" + component + "/x_ub_soft", x_ub_soft);
    ros::param::get("/nmpc_controller/" + component + "/u_lb", u_lb);
    ros::param::get("/nmpc_controller/" + component + "/u_ub", u_ub);
    ros::param::get("/nmpc_controller/" + component + "/g_lb", g_lb);
    ros::param::get("/nmpc_controller/" + component + "/g_ub", g_ub);

    // Make sure the bounds are the correct size
    if (x_dim != x_lb.size()) throw std::runtime_error("x_lb wrong size");
    if (x_dim != x_ub.size()) throw std::runtime_error("x_ub wrong size");
    if (u_dim != u_lb.size()) throw std::runtime_error("u_lb wrong size");
    if (u_dim != u_ub.size()) throw std::runtime_error("u_ub wrong size");
    if (g_dim != g_lb.size()) throw std::runtime_error("g_lb wrong size");
    if (g_dim != g_ub.size()) throw std::runtime_error("g_ub wrong size");

    // Add to simple if specified
    if (components_in_simple[i]) {
      assert(components_in_complex[i]);
      assert(components_in_cost[i]);
      config_.x_dim_simple += x_dim;
      config_.u_dim_simple += u_dim;
      config_.g_dim_simple += g_dim;
      config_.x_dim_cost_simple += x_dim;
      config_.u_dim_cost_simple += u_dim;
    }

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

  // If mixed complexity is enabled, load the desired structures
  ros::param::get("nmpc_controller/enable_mixed_complexity",
                  enable_mixed_complexity_);
  if (enable_mixed_complexity_) {
    ros::param::get("nmpc_controller/enable_adaptive_complexity",
                    enable_adaptive_complexity_);
    // Define and load adaptive complexity parameters
    std::vector<int> fixed_complex_idxs;
    int fixed_complex_head, fixed_complex_tail;

    ros::param::get("nmpc_controller/fixed_complex_idxs", fixed_complex_idxs);
    ros::param::get("nmpc_controller/fixed_complex_head", fixed_complex_head);
    ros::param::get("nmpc_controller/fixed_complex_tail", fixed_complex_tail);
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

  mynlp_ = new quadNLP(N_, dt_, mu, panic_weights, constraint_panic_weights,
                       Q_temporal_factor, R_temporal_factor,
                       fixed_complexity_schedule, config_);

  app_ = IpoptApplicationFactory();

  app_->Options()->SetStringValue("print_timing_statistics", "no");
  app_->Options()->SetStringValue("linear_solver", "ma57");
  app_->Options()->SetIntegerValue("print_level", 0);
  app_->Options()->SetNumericValue("ma57_pre_alloc", 1.5);
  app_->Options()->SetStringValue("fixed_variable_treatment",
                                  "make_parameter_nodual");
  app_->Options()->SetNumericValue("tol", 1e-2);
  app_->Options()->SetNumericValue("dual_inf_tol", 1e10);
  app_->Options()->SetNumericValue("constr_viol_tol", 1e-2);
  app_->Options()->SetNumericValue("compl_inf_tol", 1e-2);
  app_->Options()->SetNumericValue("warm_start_bound_push", 1e-6);
  app_->Options()->SetNumericValue("warm_start_slack_bound_push", 1e-6);
  app_->Options()->SetNumericValue("warm_start_mult_bound_push", 1e-6);

  // app_->Options()->SetNumericValue("max_wall_time", 20.0 * dt_);
  // app_->Options()->SetNumericValue("max_cpu_time", 20.0 * dt_);
  app_->Options()->SetNumericValue("max_wall_time", 30.0 * 0.01);
  app_->Options()->SetNumericValue("max_cpu_time", 30.0 * 0.01);

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

  // Update warm start info based on solve status
  if (status == Solve_Succeeded) {
    mynlp_->warm_start_ = true;
    return true;
  } else {
    mynlp_->mu0_ = 1e-1;
    mynlp_->warm_start_ = false;
    require_init_ = true;
    ROS_WARN_STREAM("NMPC solving fail");
    throw std::runtime_error("Solver failed");
    return false;
  }
}
