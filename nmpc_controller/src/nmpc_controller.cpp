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
<<<<<<< HEAD
    case 2:
      robot_ns_ = "new_platform";
      default_system = NEW_PLATFORM;
      break;
=======
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
    default:
      robot_ns_ = "spirit";
      default_system = SPIRIT;
      break;
  }

<<<<<<< HEAD
  // Load parameters set by local planner
  quad_utils::loadROSParam(nh_, "/local_planner/horizon_length", N_);
  quad_utils::loadROSParam(nh_, "/local_planner/timestep", dt_);

  // Load system parameters
  double mu, panic_weights, constraint_panic_weights, Q_temporal_factor,
      R_temporal_factor;
  quad_utils::loadROSParam(nh_, "/nmpc_controller/friction_coefficient", mu);
  quad_utils::loadROSParam(nh_, "/nmpc_controller/panic_weights",
                           panic_weights);
  quad_utils::loadROSParam(nh_, "/nmpc_controller/constraint_panic_weights",
                           constraint_panic_weights);
  quad_utils::loadROSParam(nh_, "/nmpc_controller/Q_temporal_factor",
                           Q_temporal_factor);
  quad_utils::loadROSParam(nh_, "/nmpc_controller/R_temporal_factor",
                           R_temporal_factor);
  Q_temporal_factor = std::pow(Q_temporal_factor, 1.0 / (N_ - 2));
  R_temporal_factor = std::pow(R_temporal_factor, 1.0 / (N_ - 2));

  // Determine whether to let horizon length vary or not
  quad_utils::loadROSParam(nh_, "/nmpc_controller/enable_variable_horizon",
                           enable_variable_horizon_);
  quad_utils::loadROSParam(nh_, "/nmpc_controller/min_horizon_length", N_min_);
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
    quad_utils::loadROSParam(nh_, "nmpc_controller/" + component + "/x_dim",
                             x_dim);
    quad_utils::loadROSParam(nh_, "nmpc_controller/" + component + "/u_dim",
                             u_dim);
    quad_utils::loadROSParam(nh_, "nmpc_controller/" + component + "/g_dim",
                             g_dim);
    quad_utils::loadROSParam(nh_, "nmpc_controller/" + component + "/x_lb",
                             x_lb);
    quad_utils::loadROSParam(nh_, "nmpc_controller/" + component + "/x_ub",
                             x_ub);
    quad_utils::loadROSParam(nh_, "nmpc_controller/" + component + "/x_lb_soft",
                             x_lb_soft);
    quad_utils::loadROSParam(nh_, "nmpc_controller/" + component + "/x_ub_soft",
                             x_ub_soft);
    quad_utils::loadROSParam(nh_, "nmpc_controller/" + component + "/u_lb",
                             u_lb);
    quad_utils::loadROSParam(nh_, "nmpc_controller/" + component + "/u_ub",
                             u_ub);
    quad_utils::loadROSParam(nh_, "nmpc_controller/" + component + "/g_lb",
                             g_lb);
    quad_utils::loadROSParam(nh_, "nmpc_controller/" + component + "/g_ub",
                             g_ub);
    quad_utils::loadROSParam(nh_, "nmpc_controller/" + component + "/x_weights",
                             x_weights);
    quad_utils::loadROSParam(nh_, "nmpc_controller/" + component + "/u_weights",
                             u_weights);

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
=======
  // Load MPC/system parameters
  ros::param::get("/nmpc_controller/" + param_ns_ + "/horizon_length", N_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_dimension", n_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_dimension", m_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/step_length", dt_);

  // Load MPC cost weighting and bounds
  std::vector<double> state_weights,
      control_weights,
      state_weights_factors,
      control_weights_factors,
      state_lower_bound,
      state_upper_bound,
      control_lower_bound,
      control_upper_bound;
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_weights", state_weights);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_weights", control_weights);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_weights_factors", state_weights_factors);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_weights_factors", control_weights_factors);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_lower_bound", state_lower_bound);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_upper_bound", state_upper_bound);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_lower_bound", control_lower_bound);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_upper_bound", control_upper_bound);
  Eigen::Map<Eigen::MatrixXd> Q(state_weights.data(), n_, 1),
      R(control_weights.data(), m_, 1),
      Q_factor(state_weights_factors.data(), N_, 1),
      R_factor(control_weights_factors.data(), N_, 1),
      x_min(state_lower_bound.data(), n_, 1),
      x_max(state_upper_bound.data(), n_, 1),
      u_min(control_lower_bound.data(), m_, 1),
      u_max(control_upper_bound.data(), m_, 1);

  mynlp_ = new quadNLP(
      type_,
      N_,
      n_,
      m_,
      dt_,
      Q,
      R,
      Q_factor,
      R_factor,
      x_min,
      x_max,
      u_min,
      u_max);
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18

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

<<<<<<< HEAD
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
=======
  // app_->Options()->SetIntegerValue("max_iter", 100);
  // app_->Options()->SetStringValue("print_timing_statistics", "yes");
  // app_->Options()->SetStringValue("linear_solver", "ma57");
  app_->Options()->SetIntegerValue("print_level", 0);
  app_->Options()->SetStringValue("mu_strategy", "adaptive");
  // app_->Options()->SetStringValue("mu_oracle", "probing");
  app_->Options()->SetStringValue("mehrotra_algorithm", "yes");
  app_->Options()->SetStringValue("bound_mult_init_method", "mu-based");
  app_->Options()->SetStringValue("expect_infeasible_problem", "yes");
  // app_->Options()->SetStringValue("start_with_resto", "yes");
  // app_->Options()->SetStringValue("adaptive_mu_globalization", "never-monotone-mode");
  // app_->Options()->SetStringValue("accept_every_trial_step", "yes");
  app_->Options()->SetStringValue("nlp_scaling_method", "none");
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18

  app_ = IpoptApplicationFactory();

  app_->Options()->SetStringValue("print_timing_statistics", "no");
  app_->Options()->SetStringValue("linear_solver", "ma27");
  app_->Options()->SetIntegerValue("print_level", 0);
  app_->Options()->SetNumericValue("ma57_pre_alloc", 1.5);
  app_->Options()->SetStringValue("fixed_variable_treatment",
                                  "make_parameter_nodual");
  app_->Options()->SetNumericValue("tol", 1e-3);
  app_->Options()->SetNumericValue("bound_relax_factor", 1e-3);
  app_->Options()->SetNumericValue("max_wall_time", 3.6 * dt_);
  app_->Options()->SetNumericValue("max_cpu_time", 3.6 * dt_);

  ApplicationReturnStatus status;
  status = app_->Initialize();
  if (status != Solve_Succeeded) {
    std::cout << std::endl
              << std::endl
              << "*** Error during NMPC initialization!" << std::endl;
    return;
  }

  require_init_ = true;
<<<<<<< HEAD

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
=======

  for (int i = 0; i < N_; ++i)
  {
    u.block(0, i, m_, 1) = mynlp_->w0_.block(i * (n_ + m_), 0, m_, 1);
    x.block(0, i, n_, 1) = mynlp_->w0_.block(i * (n_ + m_) + m_, 0, n_, 1);
  }

  app_->Options()->SetStringValue("warm_start_same_structure", "yes");

  mynlp_->w0_.setZero();
  mynlp_->z_L0_.setZero();
  mynlp_->z_U0_.setZero();
  mynlp_->lambda0_.setZero();
}

bool NMPCController::computeLegPlan(const Eigen::VectorXd &initial_state,
                                    const Eigen::MatrixXd &ref_traj,
                                    const Eigen::MatrixXd &foot_positions,
                                    const std::vector<std::vector<bool>> &contact_schedule,
                                    const Eigen::VectorXd &ref_ground_height,
                                    Eigen::MatrixXd &state_traj,
                                    Eigen::MatrixXd &control_traj)
{
  // Local planner will send a reference traj with N+1 rows
  mynlp_->shift_initial_guess();
  mynlp_->update_solver(
      initial_state,
      ref_traj.bottomRows(N_),
      foot_positions,
      contact_schedule,
      ref_ground_height.tail(N_));

  // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  // std::cout << "leg contact_sequence_: " << mynlp_->contact_sequence_.transpose().format(CleanFmt) << std::endl;

  return this->computePlan(initial_state,
                           ref_traj,
                           foot_positions,
                           contact_schedule,
                           state_traj,
                           control_traj);
}

bool NMPCController::computeCentralizedTailPlan(const Eigen::VectorXd &initial_state,
                                                const Eigen::MatrixXd &ref_traj,
                                                const Eigen::MatrixXd &foot_positions,
                                                const std::vector<std::vector<bool>> &contact_schedule,
                                                const Eigen::VectorXd &tail_initial_state,
                                                const Eigen::MatrixXd &tail_ref_traj,
                                                Eigen::MatrixXd &state_traj,
                                                Eigen::MatrixXd &control_traj,
                                                Eigen::MatrixXd &tail_state_traj,
                                                Eigen::MatrixXd &tail_control_traj)
{
  Eigen::MatrixXd ref_traj_with_tail(N_ + 1, 16),
      state_traj_with_tail(N_ + 1, 16),
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

  mynlp_->shift_initial_guess();
  mynlp_->update_solver(
      initial_state_with_tail,
      ref_traj_with_tail.bottomRows(N_),
      foot_positions,
      contact_schedule);

  bool success = this->computePlan(initial_state_with_tail,
                                   ref_traj_with_tail,
                                   foot_positions,
                                   contact_schedule,
                                   state_traj_with_tail,
                                   control_traj_with_tail);

  state_traj = Eigen::MatrixXd::Zero(N_ + 1, 12);
  control_traj = Eigen::MatrixXd::Zero(N_, 12);
  tail_state_traj = Eigen::MatrixXd::Zero(N_ + 1, 4);
  tail_control_traj = Eigen::MatrixXd::Zero(N_, 2);

  state_traj.leftCols(6) = state_traj_with_tail.leftCols(6);
  state_traj.rightCols(6) = state_traj_with_tail.block(0, 8, N_ + 1, 6);

  control_traj = control_traj_with_tail.rightCols(12);

  tail_state_traj.leftCols(2) = state_traj_with_tail.block(0, 6, N_ + 1, 2);
  tail_state_traj.rightCols(2) = state_traj_with_tail.block(0, 14, N_ + 1, 2);

  tail_control_traj = control_traj_with_tail.leftCols(2);

  return success;
}

bool NMPCController::computeDistributedTailPlan(const Eigen::VectorXd &initial_state,
                                                const Eigen::MatrixXd &ref_traj,
                                                const Eigen::MatrixXd &foot_positions,
                                                const std::vector<std::vector<bool>> &contact_schedule,
                                                const Eigen::VectorXd &tail_initial_state,
                                                const Eigen::MatrixXd &tail_ref_traj,
                                                const Eigen::MatrixXd &state_traj,
                                                const Eigen::MatrixXd &control_traj,
                                                Eigen::MatrixXd &tail_state_traj,
                                                Eigen::MatrixXd &tail_control_traj)
{
  Eigen::MatrixXd ref_traj_with_tail(N_ + 1, 16),
      state_traj_with_tail(N_, 16),
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

  mynlp_->shift_initial_guess();
  mynlp_->update_solver(
      initial_state_with_tail,
      ref_traj_with_tail.bottomRows(N_),
      foot_positions,
      contact_schedule,
      state_traj,
      control_traj);

  bool success = this->computePlan(initial_state_with_tail,
                                   ref_traj_with_tail,
                                   foot_positions,
                                   contact_schedule,
                                   state_traj_with_tail,
                                   control_traj_with_tail);

  tail_state_traj.leftCols(2) = state_traj_with_tail.block(0, 6, N_ + 1, 2);
  tail_state_traj.rightCols(2) = state_traj_with_tail.block(0, 14, N_ + 1, 2);

  tail_control_traj = control_traj_with_tail.leftCols(2);
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18

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

<<<<<<< HEAD
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
=======
  if (status == Solve_Succeeded)
  {
    state_traj = Eigen::MatrixXd::Zero(N_ + 1, n_);
    state_traj.topRows(1) = initial_state.transpose();
    control_traj = Eigen::MatrixXd::Zero(N_, m_);

    state_traj.bottomRows(N_) = x.transpose();
    control_traj = u.transpose();

    ROS_INFO_STREAM(param_ns_ << " solving success");
    return true;
  }
  else
  {
    mynlp_->w0_.setZero();
    mynlp_->z_L0_.setZero();
    mynlp_->z_U0_.setZero();
    mynlp_->lambda0_.setZero();

    ROS_INFO_STREAM(param_ns_ << " solving fail");
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
    return false;
  }
}
