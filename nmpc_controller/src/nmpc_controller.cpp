#include "nmpc_controller/nmpc_controller.h"

NMPCController::NMPCController(bool with_tail)
{
  init(with_tail);
}

NMPCController::NMPCController(ros::NodeHandle nh)
{
  nh_ = nh;
  init(false);

  std::string robot_state_traj_topic, robot_state_topic, grf_array_topic, control_traj_topic;
  ros::param::get("topics/trajectory", robot_state_traj_topic);
  ros::param::get("topics/state/ground_truth", robot_state_topic);
  ros::param::get("topics/control/grfs", grf_array_topic);
  ros::param::get("topics/control/trajectory", control_traj_topic);

  // Setup pubs and subs
  robot_state_traj_sub_ = nh_.subscribe(robot_state_traj_topic, 1, &NMPCController::robotPlanCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic, 1, &NMPCController::robotStateCallback, this);
  grf_array_pub_ = nh_.advertise<spirit_msgs::GRFArray>(grf_array_topic, 1);
  traj_pub_ = nh_.advertise<spirit_msgs::RobotPlan>(control_traj_topic, 1);
  ROS_INFO("MPC Controller setup, waiting for callbacks");
}

void NMPCController::init(bool with_tail)
{
  // Load rosparams from parameter server
  ros::param::get("nmpc_controller/update_rate", update_rate_);

  // Load MPC/system parameters
  int N, n, m;
  double dt;
  ros::param::get("/nmpc_controller/horizon_length", N);
  ros::param::get("/nmpc_controller/state_dimension", n);
  ros::param::get("/nmpc_controller/control_dimension", m);
  ros::param::get("/nmpc_controller/step_length", dt);

  // TODO seperate with_tail or not cases
  std::vector<double> state_weights,
      control_weights,
      state_lower_bound,
      state_upper_bound,
      control_lower_bound,
      control_upper_bound;
  ros::param::get("/nmpc_controller/state_weights", state_weights);
  ros::param::get("/nmpc_controller/control_weights", control_weights);
  ros::param::get("/nmpc_controller/state_lower_bound", state_lower_bound);
  ros::param::get("/nmpc_controller/state_upper_bound", state_upper_bound);
  ros::param::get("/nmpc_controller/control_lower_bound", control_lower_bound);
  ros::param::get("/nmpc_controller/control_upper_bound", control_upper_bound);
  Eigen::Map<Eigen::MatrixXd> Q(state_weights.data(), n, 1),
      R(control_weights.data(), m, 1),
      x_min(state_lower_bound.data(), n, 1),
      x_max(state_upper_bound.data(), n, 1),
      u_min(control_lower_bound.data(), m, 1),
      u_max(control_upper_bound.data(), m, 1);

  // Convert kinematics
  kinematics_ = std::make_shared<spirit_utils::SpiritKinematics>();

  mynlp_ = new spiritNLP(
      N,
      n,
      m,
      dt,
      with_tail,
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
  // app_->Options()->SetStringValue("adaptive_mu_globalization", "never-monotone-mode");
  // app_->Options()->SetStringValue("accept_every_trial_step", "yes");
  // app_->Options()->SetStringValue("nlp_scaling_method", "none");

  app_->Options()->SetStringValue("warm_start_init_point", "yes");

  app_->Options()->SetNumericValue("tol", 1e-3);
  app_->Options()->SetNumericValue("max_wall_time", 0.9 * mynlp_->dt_);
  app_->Options()->SetNumericValue("max_cpu_time", 0.9 * mynlp_->dt_);

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

  Eigen::MatrixXd x(mynlp_->n_, mynlp_->N_);
  Eigen::MatrixXd u(mynlp_->m_, mynlp_->N_);

  for (int i = 0; i < mynlp_->N_; ++i)
  {
    u.block(0, i, mynlp_->m_, 1) = mynlp_->w0_.block(i * (mynlp_->n_ + mynlp_->m_), 0, mynlp_->m_, 1);
    x.block(0, i, mynlp_->n_, 1) = mynlp_->w0_.block(i * (mynlp_->n_ + mynlp_->m_) + mynlp_->m_, 0, mynlp_->n_, 1);
  }

  last_state_traj_ = x.transpose();
  last_control_traj_ = u.transpose();

  app_->Options()->SetStringValue("warm_start_same_structure", "yes");
}

void NMPCController::robotPlanCallback(const spirit_msgs::RobotStateTrajectory::ConstPtr &msg)
{
  last_plan_msg_ = msg;
}

void NMPCController::robotStateCallback(const spirit_msgs::RobotState::ConstPtr &msg)
{
  cur_state_ = spirit_utils::bodyStateMsgToEigen(msg->body);
}

void NMPCController::extractMPCTrajectory(int start_idx,
                                          std::vector<std::vector<bool>> &contact_sequences,
                                          Eigen::MatrixXd &foot_positions,
                                          Eigen::MatrixXd &ref_traj)
{
  int plan_length = last_plan_msg_->states.size();
  contact_sequences.resize(mynlp_->N_);
  foot_positions = Eigen::MatrixXd::Zero(mynlp_->N_, 12);
  ref_traj = Eigen::MatrixXd::Zero(mynlp_->N_, mynlp_->n_);

  spirit_msgs::RobotState robot_state;
  Eigen::Vector3d foot_pos_body;
  sensor_msgs::JointState joint_state;

  int plan_index;
  bool zero_vel = false;
  for (int i = 0; i < mynlp_->N_; ++i)
  {
    // Saturate at last state in plane and zero velocity
    if (start_idx + i < plan_length)
    {
      plan_index = start_idx + i;
    }
    else
    {
      plan_index = plan_length - 1;
      zero_vel = true;
    }

    // Collect state at correct index
    robot_state = last_plan_msg_->states.at(plan_index);

    // Load contact sequence and foot positions
    contact_sequences.at(i).resize(num_legs_);
    std::vector<double> joint_states = robot_state.joints.position;

    for (int leg_idx = 0; leg_idx < num_legs_; ++leg_idx)
    {
      contact_sequences.at(i).at(leg_idx) = robot_state.feet.feet.at(leg_idx).contact;

      Eigen::Vector3d joint_pos;
      for (int joint_idx = 0; joint_idx < num_joints_per_leg_; ++joint_idx)
      {
        joint_pos(joint_idx) = joint_states.at(leg_idx * num_joints_per_leg_ + joint_idx);
      }

      kinematics_->bodyToFootFK(leg_idx, joint_pos, foot_pos_body);

      // This should be replaced with a block operation but for now it'll do
      foot_positions(i, leg_idx * num_joints_per_leg_ + 0) = foot_pos_body(0);
      foot_positions(i, leg_idx * num_joints_per_leg_ + 1) = foot_pos_body(1);
      foot_positions(i, leg_idx * num_joints_per_leg_ + 2) = foot_pos_body(2);
    }

    // Load state into reference trajectory (w/ zero velocity if we're at end of plan)
    ref_traj.row(i) = spirit_utils::bodyStateMsgToEigen(robot_state.body);
  }
}

void NMPCController::publishGRFArray()
{
  if (last_plan_msg_ == NULL)
  {
    ROS_WARN_THROTTLE(0.5, "No robot trajectory plan in MPCController, exiting");
    return;
  }

  if (last_plan_msg_->states.size() == 0)
  {
    ROS_WARN_THROTTLE(0.5, "Received robot trajectory with no states in MPCController, exiting");
    return;
  }

  double last_plan_age_seconds = spirit_utils::getROSMessageAgeInMs(last_plan_msg_->header) / 1000.0;
  int start_idx = ceil(last_plan_age_seconds / mynlp_->dt_) + 1;

  // Containers for MPC inputs
  std::vector<std::vector<bool>> contact_sequences;
  Eigen::MatrixXd foot_positions;
  Eigen::MatrixXd ref_traj;
  this->extractMPCTrajectory(start_idx, contact_sequences, foot_positions, ref_traj);

  Eigen::MatrixXd state_traj, control_traj;
  this->computePlan(
      true,
      cur_state_,
      ref_traj,
      foot_positions,
      contact_sequences,
      state_traj,
      control_traj);

  // Copy normal forces into GRFArray
  spirit_msgs::GRFArray msg; // control traj nu x n
  msg.points.resize(num_legs_);
  msg.vectors.resize(num_legs_);
  for (int i = 0; i < num_legs_; ++i)
  {
    msg.points[i].x = foot_positions(0, 3 * i);
    msg.points[i].y = foot_positions(0, 3 * i + 1);
    msg.points[i].z = foot_positions(0, 3 * i + 2);

    msg.vectors[i].x = control_traj(0, 3 * i);
    msg.vectors[i].y = control_traj(0, 3 * i + 1);
    msg.vectors[i].z = control_traj(0, 3 * i + 2);
  }

  msg.header.stamp = ros::Time::now();
  grf_array_pub_.publish(msg);
}

bool NMPCController::computePlan(const bool &new_step,
                                 const Eigen::VectorXd &initial_state,
                                 const Eigen::MatrixXd &ref_traj,
                                 const Eigen::MatrixXd &foot_positions,
                                 const std::vector<std::vector<bool>> &contact_schedule,
                                 Eigen::MatrixXd &state_traj,
                                 Eigen::MatrixXd &control_traj)
{
  // Start a timer
  // spirit_utils::FunctionTimer timer(__FUNCTION__);

  if (new_step)
  {
    mynlp_->shift_initial_guess();
  }

  mynlp_->update_solver(
      initial_state,
      ref_traj.bottomRows(mynlp_->N_),
      foot_positions,
      contact_schedule);

  ApplicationReturnStatus status;
  status = app_->ReOptimizeTNLP(mynlp_);

  Eigen::MatrixXd x(mynlp_->n_, mynlp_->N_);
  Eigen::MatrixXd u(mynlp_->m_, mynlp_->N_);

  for (int i = 0; i < mynlp_->N_; ++i)
  {
    u.block(0, i, mynlp_->m_, 1) = mynlp_->w0_.block(i * (mynlp_->n_ + mynlp_->m_), 0, mynlp_->m_, 1);
    x.block(0, i, mynlp_->n_, 1) = mynlp_->w0_.block(i * (mynlp_->n_ + mynlp_->m_) + mynlp_->m_, 0, mynlp_->n_, 1);
  }

  if (status == Solve_Succeeded)
  {
    // Activate warm start
    // app_->Options()->SetNumericValue("warm_start_bound_push", 1e-9);
    // app_->Options()->SetNumericValue("warm_start_bound_frac", 1e-9);
    // app_->Options()->SetNumericValue("warm_start_slack_bound_push", 1e-9);
    // app_->Options()->SetNumericValue("warm_start_slack_bound_frac", 1e-9);
    // app_->Options()->SetNumericValue("warm_start_mult_bound_push", 1e-9);

    last_state_traj_ = x.transpose();
    last_control_traj_ = u.transpose();

    state_traj = x.transpose();
    control_traj = u.transpose();
  }
  else
  {
    // Disable warm start
    // app_->Options()->SetNumericValue("warm_start_bound_push", 1e-2);
    // app_->Options()->SetNumericValue("warm_start_bound_frac", 1e-2);
    // app_->Options()->SetNumericValue("warm_start_slack_bound_push", 1e-2);
    // app_->Options()->SetNumericValue("warm_start_slack_bound_frac", 1e-2);
    // app_->Options()->SetNumericValue("warm_start_mult_bound_push", 1e-2);

    last_state_traj_.topRows(mynlp_->N_ - 1) = last_state_traj_.bottomRows(mynlp_->N_ - 1);
    last_control_traj_.topRows(mynlp_->N_ - 1) = last_control_traj_.bottomRows(mynlp_->N_ - 1);

    if (contact_schedule.rbegin()[0] != contact_schedule.rbegin()[1])
    {
      for (size_t i = 2; i < mynlp_->N_; i++)
      {
        // std::cout << "change contact to idx: " << i << std::endl;
        if (contact_schedule.rbegin()[0] == contact_schedule.rbegin()[i])
        {
          last_control_traj_.bottomRows(1) = last_control_traj_.row(mynlp_->N_ - (i + 1));
          break;
        }
      }
    }

    // last_state_traj_.bottomRows(1) = x.rightCols(mynlp_->N_).transpose();
    // last_control_traj_.bottomRows(1) = u.rightCols(mynlp_->N_).transpose();

    state_traj = last_state_traj_;
    control_traj = last_control_traj_;
  }

  // std::cout << last_state_traj_ << std::endl;
  // std::cout << last_control_traj_ << std::endl;

  // double compute_time = 1000 * timer.reportSilent();
  // ROS_INFO_STREAM("IPOPT solve time: " << compute_time << " ms");

  return true;
}

void NMPCController::spin()
{
  ros::Rate r(update_rate_);
  while (ros::ok())
  {
    ROS_WARN_THROTTLE(5, "MPC node still operating");

    ros::spinOnce();

    // Publish control input data
    publishGRFArray();

    r.sleep();
  }
}