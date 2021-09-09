#include "nmpc_controller/nmpc_controller.h"

NMPCController::NMPCController(int type)
{
  type_ = type;

  switch (type_)
  {
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

  // Load rosparams from parameter server
  ros::param::get("nmpc_controller/" + param_ns_ + "/update_rate", update_rate_);

  // Load MPC/system parameters
  ros::param::get("/nmpc_controller/" + param_ns_ + "/horizon_length", N_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/state_dimension", n_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/control_dimension", m_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/step_length", dt_);
  ros::param::get("/nmpc_controller/" + param_ns_ + "/terminal_scale_factor", terminal_scale_factor_);

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
  kinematics_ = std::make_shared<spirit_utils::SpiritKinematics>();

  mynlp_ = new spiritNLP(
      type_,
      N_,
      n_,
      m_,
      dt_,
      terminal_scale_factor_,
      Q,
      R,
      x_min,
      x_max,
      u_min,
      u_max);

  app_ = IpoptApplicationFactory();

  // app_->Options()->SetIntegerValue("max_iter", 100);
  // app_->Options()->SetStringValue("print_timing_statistics", "yes");
  // app_->Options()->SetStringValue("linear_solver", "ma57");
  app_->Options()->SetIntegerValue("print_level", 0);
  app_->Options()->SetStringValue("mu_strategy", "adaptive");
  // app_->Options()->SetStringValue("mu_oracle", "probing");
  app_->Options()->SetStringValue("mehrotra_algorithm", "yes");
  app_->Options()->SetStringValue("bound_mult_init_method", "mu-based");
  app_->Options()->SetStringValue("fast_step_computation", "yes");
  // app_->Options()->SetStringValue("expect_infeasible_problem", "yes");
  // app_->Options()->SetStringValue("adaptive_mu_globalization", "never-monotone-mode");
  // app_->Options()->SetStringValue("accept_every_trial_step", "yes");
  app_->Options()->SetStringValue("nlp_scaling_method", "none");

  app_->Options()->SetStringValue("warm_start_init_point", "yes");

  app_->Options()->SetNumericValue("tol", 1e-3);
  // app_->Options()->SetNumericValue("bound_relax_factor", 1e-3);
  app_->Options()->SetNumericValue("max_wall_time", 0.9 * dt_);
  app_->Options()->SetNumericValue("max_cpu_time", 0.9 * dt_);

  ApplicationReturnStatus status;
  status = app_->Initialize();
  if (status != Solve_Succeeded)
  {
    std::cout << std::endl
              << std::endl
              << "*** Error during NMPC initialization!" << std::endl;
    return;
  }

  // Optimize once for structure preparation
  status = app_->OptimizeTNLP(mynlp_);

  Eigen::MatrixXd x(n_, N_);
  Eigen::MatrixXd u(m_, N_);

  for (int i = 0; i < N_; ++i)
  {
    u.block(0, i, m_, 1) = mynlp_->w0_.block(i * (n_ + m_), 0, m_, 1);
    x.block(0, i, n_, 1) = mynlp_->w0_.block(i * (n_ + m_) + m_, 0, n_, 1);
  }

  last_state_traj_ = x.transpose();
  last_control_traj_ = u.transpose();

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
                                    Eigen::MatrixXd &state_traj,
                                    Eigen::MatrixXd &control_traj)
{
  mynlp_->shift_initial_guess();
  mynlp_->update_solver(
      initial_state,
      ref_traj,
      foot_positions,
      contact_schedule);

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
      ref_traj_with_tail,
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
                                                const double dt_first_step,
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
      ref_traj_with_tail,
      foot_positions,
      contact_schedule,
      state_traj.bottomRows(N_),
      control_traj);
  mynlp_->dt_first_step_ = dt_first_step;

  // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  // std::cout << "mynlp_->x_current_.transpose().format(CleanFmt)" << std::endl;
  // std::cout << mynlp_->x_current_.transpose().format(CleanFmt) << std::endl;
  // std::cout << "mynlp_->x_reference_.transpose().format(CleanFmt)" << std::endl;
  // std::cout << mynlp_->x_reference_.transpose().format(CleanFmt) << std::endl;
  // std::cout << "mynlp_->feet_location_.transpose().format(CleanFmt)" << std::endl;
  // std::cout << mynlp_->feet_location_.transpose().format(CleanFmt) << std::endl;
  // std::cout << "mynlp_->contact_sequence_.transpose().format(CleanFmt)" << std::endl;
  // std::cout << mynlp_->contact_sequence_.transpose().format(CleanFmt) << std::endl;
  // std::cout << "mynlp_->leg_input_.transpose().format(CleanFmt)" << std::endl;
  // std::cout << mynlp_->leg_input_.transpose().format(CleanFmt) << std::endl;

  // Eigen::Map<Eigen::MatrixXd> w0_start(mynlp_->w0_.data(), n_ + m_, N_);
  // std::cout << "w0_start.transpose().format(CleanFmt)" << std::endl;
  // std::cout << w0_start.transpose().format(CleanFmt) << std::endl;

  bool success = this->computePlan(initial_state_with_tail,
                                   ref_traj_with_tail,
                                   foot_positions,
                                   contact_schedule,
                                   state_traj_with_tail,
                                   control_traj_with_tail);

  // Eigen::Map<Eigen::MatrixXd> w0_end(mynlp_->w0_.data(), n_ + m_, N_);
  // std::cout << "w0_end.transpose().format(CleanFmt)" << std::endl;
  // std::cout << w0_end.transpose().format(CleanFmt) << std::endl;

  // Eigen::Map<Eigen::MatrixXd> z_L_matrix(mynlp_->z_L0_.data(), n_ + m_, N_);
  // Eigen::Map<Eigen::MatrixXd> z_U_matrix(mynlp_->z_U0_.data(), n_ + m_, N_);
  // Eigen::Map<Eigen::MatrixXd> lambda_matrix(mynlp_->lambda0_.data(), n_ + 16, N_);
  // std::cout << "z_L_matrix.transpose().format(CleanFmt)" << std::endl;
  // std::cout << z_L_matrix.transpose().format(CleanFmt) << std::endl;
  // std::cout << "z_U_matrix.transpose().format(CleanFmt)" << std::endl;
  // std::cout << z_U_matrix.transpose().format(CleanFmt) << std::endl;
  // std::cout << "lambda_matrix.transpose().format(CleanFmt)" << std::endl;
  // std::cout << lambda_matrix.transpose().format(CleanFmt) << std::endl;

  tail_state_traj.leftCols(2) = state_traj_with_tail.block(0, 6, N_ + 1, 2);
  tail_state_traj.rightCols(2) = state_traj_with_tail.block(0, 14, N_ + 1, 2);

  tail_control_traj = control_traj_with_tail.leftCols(2);

  // std::cout << "tail_state_traj.format(CleanFmt)" << std::endl;
  // std::cout << tail_state_traj.format(CleanFmt) << std::endl;
  // std::cout << "tail_control_traj.format(CleanFmt)" << std::endl;
  // std::cout << tail_control_traj.format(CleanFmt) << std::endl;

  return success;
}

bool NMPCController::computePlan(const Eigen::VectorXd &initial_state,
                                 const Eigen::MatrixXd &ref_traj,
                                 const Eigen::MatrixXd &foot_positions,
                                 const std::vector<std::vector<bool>> &contact_schedule,
                                 Eigen::MatrixXd &state_traj,
                                 Eigen::MatrixXd &control_traj)
{
  ApplicationReturnStatus status;
  status = app_->ReOptimizeTNLP(mynlp_);

  Eigen::MatrixXd x(n_, N_);
  Eigen::MatrixXd u(m_, N_);

  for (int i = 0; i < N_; ++i)
  {
    u.block(0, i, m_, 1) = mynlp_->w0_.block(i * (n_ + m_), 0, m_, 1);
    x.block(0, i, n_, 1) = mynlp_->w0_.block(i * (n_ + m_) + m_, 0, n_, 1);
  }

  if (status == Solve_Succeeded)
  {
    state_traj = Eigen::MatrixXd::Zero(N_ + 1, n_);
    state_traj.topRows(1) = initial_state.transpose();
    control_traj = Eigen::MatrixXd::Zero(N_, m_);

    last_state_traj_ = x.transpose();
    last_control_traj_ = u.transpose();

    state_traj.bottomRows(N_) = x.transpose();
    control_traj = u.transpose();

    ROS_INFO_STREAM(param_ns_ << " solving success");
    return true;
  }
  else
  {
    last_state_traj_.topRows(N_ - 1) = last_state_traj_.bottomRows(N_ - 1);
    last_control_traj_.topRows(N_ - 1) = last_control_traj_.bottomRows(N_ - 1);

    if (contact_schedule.rbegin()[0] != contact_schedule.rbegin()[1])
    {
      for (size_t i = 2; i < N_; i++)
      {
        // std::cout << "change contact to idx: " << i << std::endl;
        if (contact_schedule.rbegin()[0] == contact_schedule.rbegin()[i])
        {
          last_control_traj_.bottomRows(1) = last_control_traj_.row(N_ - (i + 1));
          break;
        }
      }
    }

    mynlp_->w0_.setZero();
    mynlp_->z_L0_.setZero();
    mynlp_->z_U0_.setZero();
    mynlp_->lambda0_.setZero();

    // state_traj.bottomRows(N_) = last_state_traj_;
    // control_traj = last_control_traj_;

    ROS_INFO_STREAM(param_ns_ << " solving fail");
    return false;
  }
}
