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

  // Define and load adaptive complexity parameters
  std::vector<int> fixed_complex_idxs;
  int fixed_complex_head, fixed_complex_tail;
  bool enable_adaptive_complexity = false;

  ros::param::get("nmpc_controller/enable_adaptive_complexity",
                  enable_adaptive_complexity);
  ros::param::get("nmpc_controller/fixed_complex_idxs", fixed_complex_idxs);
  ros::param::get("nmpc_controller/fixed_complex_head", fixed_complex_head);
  ros::param::get("nmpc_controller/fixed_complex_tail", fixed_complex_tail);

  // Construct fixed and adaptive complexity schedules
  Eigen::VectorXi fixed_complexity_schedule(N_);
  fixed_complexity_schedule.setZero();
  adaptive_complexity_schedule_ = fixed_complexity_schedule;
  if (enable_adaptive_complexity) {
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
    std::cout << "Adaptive complexity enabled, fixed schedule = "
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
    const double &first_element_duration, const bool &same_plan_index,
    const grid_map::GridMap &terrain, Eigen::MatrixXd &state_traj,
    Eigen::MatrixXd &control_traj) {
  // Local planner will send a reference traj with N+1 rows
  mynlp_->foot_pos_body_ = -foot_positions_body;
  mynlp_->foot_pos_world_ = foot_positions_world;
  mynlp_->foot_vel_world_ = foot_velocities_world;
  mynlp_->terrain_ = terrain;

  adaptive_complexity_schedule_.resize(N_);
  adaptive_complexity_schedule_.setZero();

  mynlp_->update_solver(initial_state, ref_traj, foot_positions_body,
                        contact_schedule, adaptive_complexity_schedule_,
                        ref_ground_height, first_element_duration,
                        same_plan_index, require_init_);
  require_init_ = false;

  quad_utils::FunctionTimer timer("nlp_solver");
  bool success = this->computePlan(initial_state, ref_traj, contact_schedule,
                                   foot_positions_world, foot_velocities_world,
                                   state_traj, control_traj);
  diagnostics_ = mynlp_->diagnostics_;
  diagnostics_.compute_time = timer.reportSilent();

  state_traj.conservativeResize(N_, n_body_);

  if (enable_variable_horizon_) updateHorizonLength();

  return success;
}

bool NMPCController::computePlan(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const std::vector<std::vector<bool>> &contact_schedule,
    Eigen::MatrixXd &foot_positions, Eigen::MatrixXd &foot_velocities,
    Eigen::MatrixXd &state_traj, Eigen::MatrixXd &control_traj) {
  ApplicationReturnStatus status;
  app_->Options()->SetNumericValue("mu_init", mynlp_->mu0_);
  if (mynlp_->warm_start_) {
    app_->Options()->SetStringValue("warm_start_init_point", "yes");
  } else {
    app_->Options()->SetStringValue("warm_start_init_point", "no");
  }

  status = app_->OptimizeTNLP(mynlp_);

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

  Eigen::MatrixXd state_traj_lifted(N_, config_.x_dim_complex);
  Eigen::MatrixXd control_traj_lifted(N_ - 1, config_.u_dim_complex);
  mynlp_->get_lifted_trajectory(state_traj_lifted, control_traj_lifted);
  Eigen::VectorXi complexity_schedule =
      updateAdaptiveComplexitySchedule(state_traj_lifted, control_traj_lifted);

  foot_positions.topRows(state_traj_lifted.rows()) =
      state_traj_lifted.middleCols(n_body_, n_foot_ / 2);
  foot_velocities.topRows(state_traj_lifted.rows()) =
      state_traj_lifted.middleCols(n_body_ + n_foot_ / 2, n_foot_ / 2);

  Eigen::MatrixXd foot_control, joint_positions, joint_velocities;
  foot_control = control_traj_lifted.rightCols(m_foot_);
  joint_positions =
      state_traj_lifted.middleCols(n_body_ + n_foot_, n_joints_ / 2);
  joint_velocities = state_traj_lifted.middleCols(
      n_body_ + n_foot_ + n_joints_ / 2, n_joints_ / 2);

  // std::cout << "complexity_schedule = " << complexity_schedule.transpose()
  //           << std::endl;

  if (status == Solve_Succeeded) {
    mynlp_->warm_start_ = true;

    // std::cout << "current body state = \n"
    //           << mynlp_->x_current_.segment(0, n_body_).transpose()
    //           << std::endl;
    // std::cout << "current joint pos = \n"
    //           << mynlp_->x_current_.segment(n_body_ + n_foot_, n_joints_ / 2)
    //                  .transpose()
    //           << std::endl;
    // std::cout << "current joint vel = \n"
    //           << mynlp_->x_current_.tail(n_joints_ / 2).transpose()
    //           << std::endl;

    // std::cout << "body_reference = \n"
    //           << mynlp_->x_reference_.transpose() << std::endl;
    // std::cout << "body_traj = \n" << state_traj << std::endl;
    // std::cout << "body error = \n"
    //           << state_traj - mynlp_->x_reference_.transpose() << std::endl;
    // std::cout << "control_traj body = \n" << control_traj << std::endl;

    // std::cout << "foot pos = \n" << foot_positions << std::endl;
    // std::cout << "foot vel = \n" << foot_velocities << std::endl;
    // std::cout << "foot pos ref = \n" << mynlp_->foot_pos_world_ << std::endl;
    // std::cout << "foot vel ref = \n" << mynlp_->foot_vel_world_ << std::endl;
    // std::cout << "foot pos error = \n"
    //           << foot_positions - mynlp_->foot_pos_world_ << std::endl;
    // std::cout << "foot vel error = \n"
    //           << foot_velocities - mynlp_->foot_vel_world_ << std::endl
    //           << std::endl;
    // std::cout << "control_traj foot = \n" << foot_control << std::endl;

    // std::cout << "joint_positions = \n" << joint_positions << std::endl;
    // std::cout << "joint_velocities = \n" << joint_velocities << std::endl;
    // throw std::runtime_error("Solve succeeded! Exiting for debug");

    return true;
  } else {
    mynlp_->mu0_ = 1e-1;
    mynlp_->warm_start_ = false;
    require_init_ = true;

    ROS_WARN_STREAM("NMPC solving fail");

    // // Get solution and bounds
    // double var_tol, constr_tol;
    // app_->Options()->GetNumericValue("tol", var_tol, "");
    // app_->Options()->GetNumericValue("constr_viol_tol", constr_tol, "");
    // int n, m;
    // n = mynlp_->n_vars_;
    // m = mynlp_->n_constraints_;
    // Eigen::VectorXd x_lb(n), x_ub(n);
    // Eigen::VectorXd g_lb(m), g_ub(m);
    // mynlp_->get_bounds_info(n, x_lb.data(), x_ub.data(), m, g_lb.data(),
    //                         g_ub.data());

    // // Loop though finite elements and check feasibility to see what failed
    // std::cout << "Evaluating constraints" << std::endl;
    // for (int i = 0; i < N_; ++i) {
    //   Eigen::VectorXd x_i = mynlp_->get_primal_state_var(mynlp_->w0_, i);
    //   Eigen::VectorXd x_i_lb = mynlp_->get_primal_state_var(x_lb, i);
    //   Eigen::VectorXd x_i_ub = mynlp_->get_primal_state_var(x_ub, i);

    //   for (int j = 0; j < x_i.size(); j++) {
    //     if ((x_i(j) < (x_i_lb(j) - var_tol)) ||
    //         (x_i(j) > (x_i_ub(j) + var_tol))) {
    //       printf(
    //           "State bound %d violated in FE %d: %5.3f <= %5.3f <= "
    //           "%5.3f\n",
    //           j, i, x_i_lb[j] - var_tol, x_i[j], x_i_ub[j] + var_tol);
    //     }
    //   }

    //   if (i < N_ - 1) {
    //     Eigen::VectorXd u_i = mynlp_->get_primal_control_var(mynlp_->w0_, i);
    //     Eigen::VectorXd u_i_lb = mynlp_->get_primal_control_var(x_lb, i);
    //     Eigen::VectorXd u_i_ub = mynlp_->get_primal_control_var(x_ub, i);

    //     for (int j = 0; j < u_i.size(); j++) {
    //       if ((u_i(j) < (u_i_lb(j) - var_tol)) ||
    //           (u_i(j) > (u_i_ub(j) + var_tol))) {
    //         printf(
    //             "Control bound %d violated in FE %d: %5.3f <= %5.3f <= "
    //             "%5.3f\n",
    //             j, i, u_i_lb[j] - var_tol, u_i[j], u_i_ub[j] + var_tol);
    //       }
    //     }

    //     Eigen::VectorXd g_i =
    //         mynlp_->get_primal_constraint_vals(mynlp_->g0_, i);
    //     Eigen::VectorXd g_i_lb = mynlp_->get_primal_constraint_vals(g_lb, i);
    //     Eigen::VectorXd g_i_ub = mynlp_->get_primal_constraint_vals(g_ub, i);

    //     for (int j = 0; j < g_i.size(); j++) {
    //       if ((g_i(j) < (g_i_lb(j) - constr_tol)) ||
    //           (g_i(j) > (g_i_ub(j) + constr_tol))) {
    //         printf(
    //             "Constraint bound %s violated in FE %d: %5.3f <= %5.3f <= "
    //             "%5.3f\n",
    //             mynlp_->constr_names_[COMPLEX][j].c_str(), i,
    //             g_i_lb[j] - constr_tol, g_i[j], g_i_ub[j] + constr_tol);
    //       }
    //     }
    //   }
    // }
    // std::cout << "Done evaluating constraints" << std::endl;

    // std::cout << "current body state = \n"
    //           << mynlp_->x_current_.segment(0, n_body_).transpose()
    //           << std::endl;
    // std::cout << "current joint pos = \n"
    //           << mynlp_->x_current_.segment(n_body_ + n_foot_, n_joints_ / 2)
    //                  .transpose()
    //           << std::endl;
    // std::cout << "current joint vel = \n"
    //           << mynlp_->x_current_.tail(n_joints_ / 2).transpose()
    //           << std::endl;

    // std::cout << "body_reference = \n"
    //           << mynlp_->x_reference_.transpose() << std::endl;
    // std::cout << "body_traj = \n" << state_traj << std::endl;
    // std::cout << "body error = \n"
    //           << state_traj - mynlp_->x_reference_.transpose() << std::endl;
    // std::cout << "control_traj body = \n" << control_traj << std::endl;

    // std::cout << "foot pos = \n" << foot_positions << std::endl;
    // std::cout << "foot vel = \n" << foot_velocities << std::endl;
    // std::cout << "foot pos ref = \n" << mynlp_->foot_pos_world_ << std::endl;
    // std::cout << "foot vel ref = \n" << mynlp_->foot_vel_world_ << std::endl;
    // std::cout << "foot pos error = \n"
    //           << foot_positions - mynlp_->foot_pos_world_ << std::endl;
    // std::cout << "foot vel error = \n"
    //           << foot_velocities - mynlp_->foot_vel_world_ << std::endl
    //           << std::endl;
    // std::cout << "control_traj foot = \n" << foot_control << std::endl;

    // std::cout << "joint_positions = \n" << joint_positions << std::endl;
    // std::cout << "joint_velocities = \n" << joint_velocities << std::endl;
    // throw std::runtime_error("Solve failed, exiting for debug");
    return false;
  }
}

Eigen::VectorXi NMPCController::updateAdaptiveComplexitySchedule(
    Eigen::MatrixXd &state_traj_lifted, Eigen::MatrixXd &control_traj_lifted) {
  // quad_utils::FunctionTimer timer(__FUNCTION__);

  // Declare decision and constraint vars
  Eigen::VectorXi adaptive_complexity_schedule(N_);
  adaptive_complexity_schedule.setZero();

  Eigen::VectorXd max_constraint_violation(N_);
  max_constraint_violation.fill(-2e19);
  double max_constraint_violation_val = -2e19;
  int max_constraint_violation_fe = -1;
  int max_constraint_violation_index = -1;
  Eigen::VectorXd constr_vals, params(36), lb_violation, ub_violation;

  bool valid_solve = true;
  bool valid_lift = true;

  double var_tol, constr_tol;
  app_->Options()->GetNumericValue("tol", var_tol, "");
  app_->Options()->GetNumericValue("constr_viol_tol", constr_tol, "");

  Eigen::VectorXd x0, u, x1;
  x0 = state_traj_lifted.row(0);

  // Loop through trajectory, lifting as needed and evaluating constraints
  for (int i = 0; i < N_ - 1; i++) {
    u = control_traj_lifted.row(i);
    x1 = state_traj_lifted.row(i + 1);

    constr_vals = mynlp_->eval_g_single_complex_fe(i, x0, u, x1);

    lb_violation = mynlp_->config_.g_min_complex - constr_vals;
    ub_violation = constr_vals - mynlp_->config_.g_max_complex;

    max_constraint_violation[i + 1] =
        (lb_violation.cwiseMax(ub_violation)).maxCoeff();

    for (int j = 0; j < constr_vals.size(); j++) {
      double current_constraint_violation =
          std::max(lb_violation[j], ub_violation[j]);

      if (current_constraint_violation > max_constraint_violation_val) {
        max_constraint_violation_val = current_constraint_violation;
        max_constraint_violation_fe = i + 1;
        max_constraint_violation_index = j;
      }

      if (current_constraint_violation > constr_tol) {
        if (mynlp_->n_vec_[i + 1] == config_.x_dim_complex) {
          printf(
              "Constraint %s violated in FE %d: %5.3f <= %5.3f <= "
              "%5.3f\n",
              mynlp_->constr_names_[COMPLEX][j].c_str(), i,
              mynlp_->config_.g_min_complex[j] - constr_tol, constr_vals[j],
              mynlp_->config_.g_max_complex[j] + constr_tol);

          // std::cout << "x0 = \n" << x0 << std::endl;
          // std::cout << "u = \n" << u << std::endl;
          // std::cout << "x1 = \n" << x1 << std::endl;
          // std::cout << "constr_vals = \n" << constr_vals << std::endl;
          // throw std::runtime_error("stop");

          valid_solve = false;
        } else {
          valid_lift = false;
        }
        adaptive_complexity_schedule[i + 1] = 1;
      }
    }

    for (int j = 0; j < x1.size(); j++) {
      if (x1[j] < mynlp_->config_.x_min_complex[j] - var_tol ||
          x1[j] > mynlp_->config_.x_max_complex[j] + var_tol) {
        if (mynlp_->n_vec_[i + 1] == config_.x_dim_complex) {
          printf(
              "Var bound %d violated in FE %d: %5.3f <= %5.3f <= "
              "%5.3f\n",
              j, i, mynlp_->config_.x_min_complex[j], x1[j] - var_tol,
              mynlp_->config_.x_max_complex[j] + var_tol);

          valid_solve = false;
        } else {
          valid_lift = false;
        }
        adaptive_complexity_schedule[i + 1] = 1;
      }
    }

    x0 = x1;
  }

  // if (!valid_lift) {
  //   std::cout << "Invalid: " << adaptive_complexity_schedule.transpose()
  //             << std::endl;
  //   // mynlp_->warm_start_ = false;
  //   // mynlp_->mu0_ = 1e-1;
  // }
  // if (!valid_solve) {
  //   ROS_WARN(
  //       "Invalid region detected where there shouldnt be (possibly due to "
  //       "constraint relaxation).");
  //   // throw std::runtime_error(
  //   //     "Invalid region detected where there shouldnt be, exiting.");
  // }

  // if (max_constraint_violation_val >= constr_tol) {
  //   // std::cout << "Max violation vector = "
  //   //           << max_constraint_violation.transpose() << std::endl;
  //   std::cout << "Max violation is constraint "
  //             <<
  //             mynlp_->constr_names_[COMPLEX][max_constraint_violation_index]
  //             << " at FE " << max_constraint_violation_fe
  //             << " with val = " << max_constraint_violation_val << std::endl;
  // }
  if (max_constraint_violation_fe >= 0)
    adaptive_complexity_schedule[max_constraint_violation_fe] = 8;
  return adaptive_complexity_schedule;
}

void NMPCController::updateHorizonLength() {
  // if (N_ % 2 == 0) {
  //   N_ += 3;
  // } else {
  //   N_ -= 3;
  // }
  N_ = 20 + std::floor(5 * cos(M_PI * ros::Time::now().toSec()));

  // if (diagnostics_.compute_time > dt_) {
  //   N_ =
  //       std::max(N_ - (int)std::floor(diagnostics_.compute_time / dt_),
  //       N_min_);
  // } else {
  //   N_ = std::min(N_ + 1, N_max_);
  // }

  N_ = std::max(std::min(N_, N_max_), N_min_);
  std::cout << "N_ = " << N_ << std::endl;
}
