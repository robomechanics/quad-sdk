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

  // Define the components, their order, and which are simple
  std::vector<std::string> components = {"body", "feet", "joints"};
  std::vector<bool> components_in_simple = {true, false, false};
  std::vector<bool> components_in_complex = {true, true, true};
  std::vector<bool> components_in_cost = {true, true, false};

  x_dim_simple_ = u_dim_simple_ = g_dim_simple_ = x_dim_cost_simple_ =
      u_dim_cost_simple_ = x_dim_complex_ = u_dim_complex_ = g_dim_complex_ =
          x_dim_cost_complex_ = u_dim_cost_complex_ = 0;
  std::vector<double> x_weights_complex, u_weights_complex, x_lb_complex,
      x_ub_complex, u_lb_complex, u_ub_complex, g_lb_complex, g_ub_complex;

  // Loop through each system component and read parameters
  for (int i = 0; i < components.size(); i++) {
    double x_dim, u_dim, g_dim;
    std::vector<double> x_weights, u_weights, x_lb, x_ub, u_lb, u_ub, g_lb,
        g_ub;

    // Read component parameters
    std::string component = components[i];
    ros::param::get("/nmpc_controller/" + component + "/x_dim", x_dim);
    ros::param::get("/nmpc_controller/" + component + "/u_dim", u_dim);
    ros::param::get("/nmpc_controller/" + component + "/g_dim", g_dim);
    ros::param::get("/nmpc_controller/" + component + "/x_lb", x_lb);
    ros::param::get("/nmpc_controller/" + component + "/x_ub", x_ub);
    ros::param::get("/nmpc_controller/" + component + "/u_lb", u_lb);
    ros::param::get("/nmpc_controller/" + component + "/u_ub", u_ub);
    ros::param::get("/nmpc_controller/" + component + "/g_lb", g_lb);
    ros::param::get("/nmpc_controller/" + component + "/g_ub", g_ub);
    ros::param::get("/nmpc_controller/" + component + "/x_weights", x_weights);
    ros::param::get("/nmpc_controller/" + component + "/u_weights", u_weights);

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
      x_dim_simple_ += x_dim;
      u_dim_simple_ += u_dim;
      g_dim_simple_ += g_dim;
      x_dim_cost_simple_ += x_dim;
      u_dim_cost_simple_ += u_dim;
    }

    // Add to complex if specified
    if (components_in_complex[i]) {
      x_dim_complex_ += x_dim;
      u_dim_complex_ += u_dim;
      g_dim_complex_ += g_dim;
      x_lb_complex.insert(x_lb_complex.end(), x_lb.begin(), x_lb.end());
      x_ub_complex.insert(x_ub_complex.end(), x_ub.begin(), x_ub.end());
      u_lb_complex.insert(u_lb_complex.end(), u_lb.begin(), u_lb.end());
      u_ub_complex.insert(u_ub_complex.end(), u_ub.begin(), u_ub.end());
      g_lb_complex.insert(g_lb_complex.end(), g_lb.begin(), g_lb.end());
      g_ub_complex.insert(g_ub_complex.end(), g_ub.begin(), g_ub.end());

      // Add to cost if specified
      if (components_in_cost[i]) {
        x_dim_cost_complex_ += x_dim;
        u_dim_cost_complex_ += u_dim;
        x_weights_complex.insert(x_weights_complex.end(), x_weights.begin(),
                                 x_weights.end());
        u_weights_complex.insert(u_weights_complex.end(), u_weights.begin(),
                                 u_weights.end());
      }
    }
  }
  x_dim_null_ = x_dim_complex_ - x_dim_simple_;
  u_dim_null_ = u_dim_complex_ - u_dim_simple_;

  Eigen::Map<Eigen::VectorXd> Q_complex(x_weights_complex.data(),
                                        x_dim_cost_complex_),
      R_complex(u_weights_complex.data(), u_dim_cost_complex_),
      x_min_complex(x_lb_complex.data(), x_dim_complex_),
      x_max_complex(x_ub_complex.data(), x_dim_complex_),
      u_min_complex(u_lb_complex.data(), u_dim_complex_),
      u_max_complex(u_ub_complex.data(), u_dim_complex_),
      g_min_complex(g_lb_complex.data(), g_dim_complex_),
      g_max_complex(g_ub_complex.data(), g_dim_complex_);

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

  mynlp_ = new quadNLP(
      N_, dt_, mu, panic_weights, constraint_panic_weights, Q_temporal_factor,
      R_temporal_factor, x_dim_simple_, x_dim_complex_, u_dim_simple_,
      u_dim_complex_, g_dim_simple_, g_dim_complex_, x_dim_cost_simple_,
      x_dim_cost_complex_, u_dim_cost_simple_, u_dim_cost_complex_, Q_complex,
      R_complex, x_min_complex, x_max_complex, u_min_complex, u_max_complex,
      g_min_complex, g_max_complex, fixed_complexity_schedule);

  app_ = IpoptApplicationFactory();

  app_->Options()->SetStringValue("print_timing_statistics", "no");
  app_->Options()->SetStringValue("linear_solver", "ma57");
  app_->Options()->SetIntegerValue("print_level", 5);
  app_->Options()->SetNumericValue("ma57_pre_alloc", 1.5);
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

  quadKD_ = std::make_shared<quad_utils::QuadKD>();
}

bool NMPCController::computeLegPlan(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions_body,
    Eigen::MatrixXd &foot_positions_world, Eigen::MatrixXd &foot_velocities,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::VectorXd &ref_ground_height,
    const double &first_element_duration, const bool &same_plan_index,
    const grid_map::GridMap &terrain, Eigen::MatrixXd &state_traj,
    Eigen::MatrixXd &control_traj) {
  // Local planner will send a reference traj with N+1 rows
  mynlp_->foot_pos_body_ = -foot_positions_body;
  mynlp_->foot_pos_world_ = foot_positions_world;
  mynlp_->foot_vel_world_ = foot_velocities;
  mynlp_->terrain_ = terrain;
  mynlp_->update_solver(initial_state, ref_traj, foot_positions_body,
                        contact_schedule, adaptive_complexity_schedule_,
                        ref_ground_height, first_element_duration,
                        same_plan_index, require_init_);
  require_init_ = false;

  quad_utils::FunctionTimer timer("nlp_solver");
  bool success =
      this->computePlan(initial_state, ref_traj, foot_positions_world,
                        contact_schedule, state_traj, control_traj);
  diagnostics_ = mynlp_->diagnostics_;
  diagnostics_.compute_time = timer.reportSilent();

  state_traj.conservativeResize(N_, mynlp_->n_body_);

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

  state_traj = Eigen::MatrixXd::Zero(N_, x_dim_simple_);
  Eigen::MatrixXd state_null_traj = Eigen::MatrixXd::Zero(N_, x_dim_null_);
  Eigen::MatrixXd control_null_traj = Eigen::MatrixXd::Zero(N_, u_dim_null_);
  Eigen::MatrixXd state_null_traj_lift = Eigen::MatrixXd::Zero(N_, x_dim_null_);
  control_traj = Eigen::MatrixXd::Zero(N_ - 1, u_dim_simple_);

  state_traj.row(0) = mynlp_->get_primal_state_var(mynlp_->w0_, 0)
                          .head(x_dim_simple_)
                          .transpose();

  for (int i = 1; i < N_; ++i) {
    control_traj.row(i - 1) = mynlp_->get_primal_control_var(mynlp_->w0_, i - 1)
                                  .head(u_dim_simple_)
                                  .transpose();
    state_traj.row(i) = mynlp_->get_primal_state_var(mynlp_->w0_, i)
                            .head(x_dim_simple_)
                            .transpose();
  }

  // if (status == Solve_Succeeded && t_solve < 5.0 * dt_) {
  if (status == Solve_Succeeded) {
    mynlp_->warm_start_ = true;

    Eigen::VectorXi constr_vals =
        evalLiftedTrajectoryConstraints(state_null_traj, control_null_traj);

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

    // std::cout << "foot pos = \n"
    //           << state_null_traj.leftCols(n_foot_ / 2) << std::endl;
    // std::cout << "foot vel = \n"
    //           << state_null_traj.middleCols(n_foot_ / 2, n_foot_ / 2)
    //           << std::endl;
    // std::cout << "foot pos ref = \n" << mynlp_->foot_pos_world_ << std::endl;
    // std::cout << "foot vel ref = \n" << mynlp_->foot_vel_world_ << std::endl;
    // std::cout << "foot pos error = \n"
    //           << state_null_traj.leftCols(n_foot_ / 2) -
    //           mynlp_->foot_pos_world_
    //           << std::endl;
    // std::cout << "foot vel error = \n"
    //           << state_null_traj.middleCols(n_foot_ / 2, n_foot_ / 2) -
    //                  mynlp_->foot_vel_world_
    //           << std::endl
    //           << std::endl;
    // std::cout << "control_traj foot = \n" << control_null_traj << std::endl;

    // std::cout << "joint_positions = \n"
    //           << state_null_traj.middleCols(n_foot_, n_joints_ / 2)
    //           << std::endl;
    // std::cout << "joint_velocities = \n"
    //           << state_null_traj.rightCols(n_joints_ / 2) << std::endl;
    // throw std::runtime_error("Solve succeeded! Exiting for debug");

    return true;
  } else {
    mynlp_->mu0_ = 1e-1;
    mynlp_->warm_start_ = false;
    require_init_ = true;

    ROS_WARN_STREAM("NMPC solving fail");

    // Get solution and bounds
    double var_tol, constr_tol;
    app_->Options()->GetNumericValue("tol", var_tol, "");
    app_->Options()->GetNumericValue("constr_viol_tol", constr_tol, "");
    int n, m;
    n = mynlp_->n_vars_;
    m = mynlp_->n_constraints_;
    Eigen::VectorXd x_lb(n), x_ub(n);
    Eigen::VectorXd g_lb(m), g_ub(m);
    mynlp_->get_bounds_info(n, x_lb.data(), x_ub.data(), m, g_lb.data(),
                            g_ub.data());

    // Loop though finite elements and check feasibility to see what failed
    std::cout << "Evaluating constraints" << std::endl;
    for (int i = 0; i < N_; ++i) {
      Eigen::VectorXd x_i = mynlp_->get_primal_state_var(mynlp_->w0_, i);
      Eigen::VectorXd x_i_lb = mynlp_->get_primal_state_var(x_lb, i);
      Eigen::VectorXd x_i_ub = mynlp_->get_primal_state_var(x_ub, i);

      for (int j = 0; j < x_i.size(); j++) {
        if ((x_i(j) < (x_i_lb(j) - var_tol)) ||
            (x_i(j) > (x_i_ub(j) + var_tol))) {
          printf(
              "State bound %d violated in FE %d: %5.3f <= %5.3f <= "
              "%5.3f\n",
              j, i, x_i_lb[j] - var_tol, x_i[j], x_i_ub[j] + var_tol);
        }
      }

      if (i < N_ - 1) {
        Eigen::VectorXd u_i = mynlp_->get_primal_control_var(mynlp_->w0_, i);
        Eigen::VectorXd u_i_lb = mynlp_->get_primal_control_var(x_lb, i);
        Eigen::VectorXd u_i_ub = mynlp_->get_primal_control_var(x_ub, i);

        for (int j = 0; j < u_i.size(); j++) {
          if ((u_i(j) < (u_i_lb(j) - var_tol)) ||
              (u_i(j) > (u_i_ub(j) + var_tol))) {
            printf(
                "Control bound %d violated in FE %d: %5.3f <= %5.3f <= "
                "%5.3f\n",
                j, i, u_i_lb[j] - var_tol, u_i[j], u_i_ub[j] + var_tol);
          }
        }

        Eigen::VectorXd g_i =
            mynlp_->get_primal_constraint_vals(mynlp_->g0_, i);
        Eigen::VectorXd g_i_lb = mynlp_->get_primal_constraint_vals(g_lb, i);
        Eigen::VectorXd g_i_ub = mynlp_->get_primal_constraint_vals(g_ub, i);

        for (int j = 0; j < g_i.size(); j++) {
          if ((g_i(j) < (g_i_lb(j) - constr_tol)) ||
              (g_i(j) > (g_i_ub(j) + constr_tol))) {
            printf(
                "Constraint bound %s violated in FE %d: %5.3f <= %5.3f <= "
                "%5.3f\n",
                mynlp_->constr_names_[COMPLEX][j].c_str(), i,
                g_i_lb[j] - constr_tol, g_i[j], g_i_ub[j] + constr_tol);
          }
        }
      }
    }
    std::cout << "Done evaluating constraints" << std::endl;

    std::cout << "Evaluating lifted trajectory" << std::endl;
    Eigen::VectorXi constr_vals =
        evalLiftedTrajectoryConstraints(state_null_traj, control_null_traj);

    std::cout << "current body state = \n"
              << mynlp_->x_current_.segment(0, n_body_).transpose()
              << std::endl;
    std::cout << "current joint pos = \n"
              << mynlp_->x_current_.segment(n_body_ + n_foot_, n_joints_ / 2)
                     .transpose()
              << std::endl;
    std::cout << "current joint vel = \n"
              << mynlp_->x_current_.tail(n_joints_ / 2).transpose()
              << std::endl;

    std::cout << "body_reference = \n"
              << mynlp_->x_reference_.transpose() << std::endl;
    std::cout << "body_traj = \n" << state_traj << std::endl;
    std::cout << "body error = \n"
              << state_traj - mynlp_->x_reference_.transpose() << std::endl;
    std::cout << "control_traj body = \n" << control_traj << std::endl;

    std::cout << "foot pos = \n"
              << state_null_traj.leftCols(n_foot_ / 2) << std::endl;
    std::cout << "foot vel = \n"
              << state_null_traj.middleCols(n_foot_ / 2, n_foot_ / 2)
              << std::endl;
    std::cout << "foot pos ref = \n" << mynlp_->foot_pos_world_ << std::endl;
    std::cout << "foot vel ref = \n" << mynlp_->foot_vel_world_ << std::endl;
    std::cout << "foot pos error = \n"
              << state_null_traj.leftCols(n_foot_ / 2) - mynlp_->foot_pos_world_
              << std::endl;
    std::cout << "foot vel error = \n"
              << state_null_traj.middleCols(n_foot_ / 2, n_foot_ / 2) -
                     mynlp_->foot_vel_world_
              << std::endl
              << std::endl;
    std::cout << "control_traj foot = \n" << control_null_traj << std::endl;

    std::cout << "joint_positions = \n"
              << state_null_traj.middleCols(n_foot_, n_joints_ / 2)
              << std::endl;
    std::cout << "joint_velocities = \n"
              << state_null_traj.rightCols(n_joints_ / 2) << std::endl;

    throw std::runtime_error("Solve failed, exiting for debug");
    return false;
  }
}

Eigen::VectorXi NMPCController::evalLiftedTrajectoryConstraints(
    Eigen::MatrixXd &state_null_traj, Eigen::MatrixXd &control_null_traj) {
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

  if (x0.size() < x_dim_complex_) {
    quadKD_->convertCentroidalToFullBody(
        x0_body, mynlp_->foot_pos_world_.row(0), mynlp_->foot_vel_world_.row(0),
        mynlp_->get_primal_body_control_var(mynlp_->w0_, 0), joint_positions,
        joint_velocities, joint_torques);
    x0.conservativeResize(mynlp_->n_complex_);
    x0.segment(n_body_, n_foot_ / 2) = mynlp_->foot_pos_world_.row(0);
    x0.segment(n_body_ + n_foot_ / 2, n_foot_ / 2) =
        mynlp_->foot_vel_world_.row(0);
    x0.segment(n_body_ + n_foot_, n_joints_ / 2) = joint_positions;
    x0.segment(n_body_ + n_foot_ + n_joints_ / 2, n_joints_ / 2) =
        joint_velocities;
  }
  state_null_traj.row(0) = x0.segment(x_dim_simple_, x_dim_null_);

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
      quadKD_->convertCentroidalToFullBody(
          x1.head(mynlp_->n_body_), mynlp_->foot_pos_world_.row(i + 1),
          mynlp_->foot_vel_world_.row(i + 1), u.head(mynlp_->n_body_),
          joint_positions, joint_velocities, joint_torques);
      x1.conservativeResize(mynlp_->n_complex_);
      x1.segment(n_body_, n_foot_ / 2) = mynlp_->foot_pos_world_.row(i + 1);
      x1.segment(n_body_ + n_foot_ / 2, n_foot_ / 2) =
          mynlp_->foot_vel_world_.row(i + 1);
      x1.segment(n_body_ + n_foot_, n_joints_ / 2) = joint_positions;
      x1.segment(n_body_ + n_foot_ + n_joints_ / 2, n_joints_ / 2) =
          joint_velocities;
    }

    if (u.size() < u_dim_complex_) {
      u.conservativeResize(u_dim_complex_);
      u.tail(u_dim_null_).fill(0);
    }

    state_null_traj.row(i + 1) = x1.segment(x_dim_simple_, x_dim_null_);
    control_null_traj.row(i + 1) = u.segment(u_dim_simple_, u_dim_null_);

    double dt = (i == 0) ? mynlp_->first_element_duration_ : dt_;
    params = mynlp_->foot_pos_world_.row(i + 1);

    constr_vals = mynlp_->eval_g_single_fe(COMPLEX, dt, x0, u, x1, params);
    lb_violation = mynlp_->g_min_complex_ - constr_vals;
    ub_violation = constr_vals - mynlp_->g_max_complex_;

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
              mynlp_->g_min_complex_[j] - constr_tol, constr_vals[j],
              mynlp_->g_max_complex_[j] + constr_tol);

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
