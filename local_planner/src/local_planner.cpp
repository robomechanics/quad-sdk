#include "local_planner/local_planner.h"

// namespace plt = matplotlibcpp;
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

LocalPlanner::LocalPlanner(ros::NodeHandle nh) :
  local_body_planner_convex_(), local_body_planner_nonlinear_(), local_footstep_planner_() {

	nh_ = nh;
  
    // Load rosparams from parameter server
  std::string terrain_map_topic, body_plan_topic, robot_state_topic, local_plan_topic,
    foot_plan_discrete_topic, foot_plan_continuous_topic, cmd_vel_topic;
  spirit_utils::loadROSParam(nh_, "topics/terrain_map", terrain_map_topic);
  spirit_utils::loadROSParam(nh_, "topics/global_plan", body_plan_topic);
  spirit_utils::loadROSParam(nh_, "topics/state/ground_truth",robot_state_topic);
  spirit_utils::loadROSParam(nh_, "topics/local_plan", local_plan_topic);
  spirit_utils::loadROSParam(nh_, "topics/foot_plan_discrete", foot_plan_discrete_topic);
  spirit_utils::loadROSParam(nh_, "topics/foot_plan_continuous", foot_plan_continuous_topic);
  spirit_utils::loadROSParam(nh_, "map_frame", map_frame_);

  nh_.param<std::string>("topics/cmd_vel", cmd_vel_topic, "/cmd_vel");

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(terrain_map_topic,1, &LocalPlanner::terrainMapCallback, this);
  body_plan_sub_ = nh_.subscribe(body_plan_topic,1, &LocalPlanner::robotPlanCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic,1,&LocalPlanner::robotStateCallback,this);
  local_plan_pub_ = nh_.advertise<spirit_msgs::RobotPlan>(local_plan_topic,1);
  foot_plan_discrete_pub_ = nh_.advertise<
    spirit_msgs::MultiFootPlanDiscrete>(foot_plan_discrete_topic,1);
  foot_plan_continuous_pub_ = nh_.advertise<
    spirit_msgs::MultiFootPlanContinuous>(foot_plan_continuous_topic,1);

  cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic,1,&LocalPlanner::cmdVelCallback, this);

  // Load system parameters
  spirit_utils::loadROSParam(nh_, "local_planner/update_rate", update_rate_);
  spirit_utils::loadROSParam(nh_, "local_planner/timestep",dt_);
  spirit_utils::loadROSParam(nh_, "local_planner/iterations",iterations_);

  nh_.param<double>("twist_body_planner/cmd_vel_scale", cmd_vel_scale_, 1);
  nh_.param<double>("twist_body_planner/last_cmd_vel_msg_time_max",last_cmd_vel_msg_time_max_,1.0);

  // Convert kinematics
  kinematics_ = std::make_shared<spirit_utils::SpiritKinematics>();

  // Initialize nominal footstep positions projected down from the hips
  Eigen::Vector3d nominal_joint_state;
  nominal_joint_state << 0, 0.78, 1.57; // Default stand angles
  hip_projected_foot_positions_ = Eigen::MatrixXd::Zero(N_,num_feet_*3); 

  for (int i = 0; i < N_; ++i) {
    for (int j = 0; j < num_feet_; ++j) {
      Eigen::Vector3d toe_body_pos;
      kinematics_->bodyToFootFK(j, nominal_joint_state, toe_body_pos);
      hip_projected_foot_positions_.block<1,3>(i,j*3) = toe_body_pos;
    }
  }

  ref_body_plan_ = Eigen::MatrixXd::Zero(N_+1, Nx_);
  foot_positions_world_ = Eigen::MatrixXd::Zero(N_,num_feet_*3);
  foot_positions_body_ = Eigen::MatrixXd::Zero(N_,num_feet_*3);
  current_foot_positions_body_ = Eigen::VectorXd::Zero(num_feet_*3);
  current_foot_positions_world_ = Eigen::VectorXd::Zero(num_feet_*3);
  
  nh.param<bool>("local_planner/use_nmpc", use_nmpc_, false);
  nh.param<bool>("local_planner/use_twist_input", use_twist_input_, false);

  // Initialize body and footstep planners
  initLocalBodyPlanner();
  initLocalFootstepPlanner();

  // Initialize twist input
  cmd_vel_.resize(6);
  // Zero the velocity to start
  std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0);

  initial_timestamp_ = ros::Time::now();
  first_plan_ = true;
}

void LocalPlanner::initLocalBodyPlanner() {

    // Load MPC parameters 
  double m,Ixx,Iyy,Izz,mu,normal_lo, normal_hi;
  spirit_utils::loadROSParam(nh_, "local_body_planner/body_mass",m);
  spirit_utils::loadROSParam(nh_, "local_body_planner/body_ixx",Ixx);
  spirit_utils::loadROSParam(nh_, "local_body_planner/body_iyy",Iyy);
  spirit_utils::loadROSParam(nh_, "local_body_planner/body_izz",Izz);
  spirit_utils::loadROSParam(nh_, "local_body_planner/friction_mu",mu);
  spirit_utils::loadROSParam(nh_, "local_body_planner/normal_lo",normal_lo);
  spirit_utils::loadROSParam(nh_, "local_body_planner/normal_hi",normal_hi);

  std::vector<double> state_weights, control_weights, state_lower_bound, state_upper_bound;
  double terminal_weight_scaling;
  spirit_utils::loadROSParam(nh_, "local_body_planner/state_weights",state_weights);
  spirit_utils::loadROSParam(nh_, "local_body_planner/terminal_weight_scaling",
    terminal_weight_scaling);
  spirit_utils::loadROSParam(nh_, "local_body_planner/control_weights",control_weights);
  spirit_utils::loadROSParam(nh_, "local_body_planner/state_lower_bound",state_lower_bound);
  spirit_utils::loadROSParam(nh_, "local_body_planner/state_upper_bound",state_upper_bound);

  // Load state weights and bounds
  Eigen::MatrixXd Qx = Eigen::MatrixXd::Zero(Nx_, Nx_);
  Eigen::VectorXd state_lo = Eigen::VectorXd::Zero(Nx_);
  Eigen::VectorXd state_hi = Eigen::VectorXd::Zero(Nx_);
  for (int i = 0; i < Nx_; ++i) {
    Qx(i,i) = state_weights.at(i);
    state_lo(i) = state_lower_bound.at(i);
    state_hi(i) = state_upper_bound.at(i);
  }

  // Load control weights
  Eigen::MatrixXd Ru = Eigen::MatrixXd::Zero(Nu_,Nu_);
  for (int i = 0; i < 3; ++i) { // for each dimension
    for (int j = 0; j < num_feet_; ++j) { //for each leg
      Ru(3*j + i,3*j + i) = control_weights.at(i);
    }
  }
  // Ru(Nu_-1,Nu_-1) = 1e-6; //gravity weight term

  std::vector<Eigen::MatrixXd> Q_vec(N_+1);
  std::vector<Eigen::MatrixXd> U_vec(N_);
  for (int i = 0; i < N_+1; ++i) {
    Q_vec.at(i) = Qx;
    if (i == N_) {
      Q_vec.at(i) = terminal_weight_scaling*Qx;
    }
  }
  for (int i = 0; i < N_; ++i) {
    U_vec.at(i) = Ru;
  }

  // Robot body inertia matrix
  Eigen::Matrix3d Ib = Eigen::Matrix3d::Zero();
  Ib.diagonal() << Ixx,Iyy,Izz;

  // Create mpc wrapper class
  if (use_nmpc_)
  {
    local_body_planner_nonlinear_ = std::make_shared<NMPCController>(0);
  }
  else
  {
    local_body_planner_convex_ = std::make_shared<QuadrupedMPC>();
    local_body_planner_convex_->setTimestep(dt_);
    local_body_planner_convex_->setMassProperties(m, Ib);
    local_body_planner_convex_->update_weights(Q_vec, U_vec);
    local_body_planner_convex_->update_state_bounds(state_lo, state_hi);
    local_body_planner_convex_->update_control_bounds(normal_lo, normal_hi);
    local_body_planner_convex_->update_friction(mu);
  }
}

void LocalPlanner::initLocalFootstepPlanner() {

  // Load parameters from server
  double grf_weight, ground_clearance, standing_error_threshold, period_d;
  int period;
  spirit_utils::loadROSParam(nh_, "local_footstep_planner/grf_weight", grf_weight);
  spirit_utils::loadROSParam(nh_, "local_footstep_planner/ground_clearance", ground_clearance);
  spirit_utils::loadROSParam(nh_, "local_footstep_planner/standing_error_threshold",
    standing_error_threshold);
  spirit_utils::loadROSParam(nh_, "local_footstep_planner/period", period_d);

  period = period_d/dt_;

  // Confirm grf weight is valid
  if (grf_weight>1 || grf_weight<0) {
    grf_weight = std::min(std::max(grf_weight,0.0),1.0);
    ROS_WARN("Invalid grf weight, clamping to %4.2f", grf_weight);
  }

  // Create footstep class, make sure we use the same dt as the local planner
  local_footstep_planner_ = std::make_shared<LocalFootstepPlanner>();
  local_footstep_planner_->setTemporalParams(dt_, period, N_);
  local_footstep_planner_->setSpatialParams(ground_clearance, standing_error_threshold,
    grf_weight, kinematics_);

  past_footholds_msg_.feet.resize(num_feet_);
}

void LocalPlanner::terrainMapCallback(
  const grid_map_msgs::GridMap::ConstPtr& msg) {
  // Get the map in its native form
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  // Convert to FastTerrainMap structure for faster querying
  terrain_.loadDataFromGridMap(map);
  local_footstep_planner_->updateMap(terrain_);
}

void LocalPlanner::robotPlanCallback(const spirit_msgs::RobotPlan::ConstPtr& msg) {
  // If this is the first plan, initialize the message of past footholds with current foot positions
  if (body_plan_msg_ == NULL && robot_state_msg_ != NULL) {
    past_footholds_msg_.header = msg->header;
    for (int i = 0; i < num_feet_; i++) {
      past_footholds_msg_.feet[i].footholds.clear();
      past_footholds_msg_.feet[i].footholds.push_back(robot_state_msg_->feet.feet[i]);
      past_footholds_msg_.feet[i].footholds.front().header = past_footholds_msg_.header;
      past_footholds_msg_.feet[i].footholds.front().traj_index = 0;
    }
  }

  body_plan_msg_ = msg;
}

void LocalPlanner::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {

  // Make sure the data is actually populated
  if (msg->feet.feet.empty() || msg->joints.position.empty())
    return;

  robot_state_msg_ = msg;
}

void LocalPlanner::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {

  if ((cmd_vel_[0] != msg->linear.x) || (cmd_vel_[1] != msg->linear.y) || 
    (cmd_vel_[5] != msg->angular.z)) {
    
    //initial_timestamp_ = ros::Time::now();
  }
  // Ignore non-planar components of desired twist
  cmd_vel_[0] = cmd_vel_scale_*msg->linear.x;
  cmd_vel_[1] = cmd_vel_scale_*msg->linear.y;
  cmd_vel_[2] = 0;
  cmd_vel_[3] = 0;
  cmd_vel_[4] = 0;
  cmd_vel_[5] = cmd_vel_scale_*msg->angular.z;

  // Record when this was last reached for safety
  last_cmd_vel_msg_time_ = ros::Time::now();

}

void LocalPlanner::getStateAndReferencePlan() {

  // Make sure body plan and robot state data is populated
  if (body_plan_msg_ == NULL || robot_state_msg_ == NULL)
    return;

  // Get index within the global plan
  current_plan_index_ = spirit_utils::getPlanIndex(body_plan_msg_->global_plan_timestamp,dt_);

  // Get the current body and foot positions into Eigen
  current_state_ = spirit_utils::odomMsgToEigen(robot_state_msg_->body);
  spirit_utils::multiFootStateMsgToEigen(robot_state_msg_->feet, current_foot_positions_world_);
  local_footstep_planner_->getFootPositionsBodyFrame(current_state_, current_foot_positions_world_,
      current_foot_positions_body_);

  // Grab the appropriate states from the body plan and convert to an Eigen matrix
  ref_body_plan_.setZero();
  for (int i = 0; i < N_+1; i++) {

    // If the horizon extends past the reference trajectory, just hold the last state
    if (i+current_plan_index_ > body_plan_msg_->plan_indices.back()) {
      ref_body_plan_.row(i) = spirit_utils::odomMsgToEigen(body_plan_msg_->states.back().body);
    } else {
      ref_body_plan_.row(i) = spirit_utils::odomMsgToEigen(body_plan_msg_->states[i+current_plan_index_].body);
    }
  }

  // Update the body plan to use for linearization
  if (body_plan_.rows() < N_+1) {
    // Cold start with reference  plan
    body_plan_ = ref_body_plan_;

    // foot_positions_body_ = hip_projected_foot_positions_;
    // foot_positions_world_.setZero();

    // Initialize with the current foot positions
    for (int i = 0; i < N_; i++) {
      foot_positions_body_.row(i) = current_foot_positions_body_;
      foot_positions_world_.row(i) = current_foot_positions_world_;
    }
  } else {
    // Warm start with old solution indexed by one
    body_plan_.topRows(N_) = body_plan_.bottomRows(N_);
    body_plan_.row(N_+1) = ref_body_plan_.row(N_+1);
    // body_plan_.bottomRows(1) = ref_body_plan_.bottomRows(1);

    // No reference for feet so last two elements will be the same
    foot_positions_body_.topRows(N_-1) = foot_positions_body_.bottomRows(N_-1);
    foot_positions_world_.topRows(N_-1) = foot_positions_world_.bottomRows(N_-1);
  }

  // Initialize with current foot and body positions
  body_plan_.row(0) = current_state_;
  // foot_positions_body_.row(0) = current_foot_positions_body_;
  // foot_positions_world_.row(0) = current_foot_positions_world_;
}

void LocalPlanner::getStateAndTwistInput() {
  //*
  if (robot_state_msg_ == NULL)
    return;

  // Get index
  current_plan_index_ = spirit_utils::getPlanIndex(initial_timestamp_,dt_);

  if (first_plan_) {
    first_plan_ = false;
    past_footholds_msg_.header = robot_state_msg_->header;
    for (int i = 0; i < num_feet_; i++) {
      past_footholds_msg_.feet[i].footholds.clear();
      past_footholds_msg_.feet[i].footholds.push_back(robot_state_msg_->feet.feet[i]);
      past_footholds_msg_.feet[i].footholds.front().header = past_footholds_msg_.header;
      past_footholds_msg_.feet[i].footholds.front().traj_index = 0;
    }
  }

  // Get the current body and foot positions into Eigen
  current_state_ = spirit_utils::odomMsgToEigen(robot_state_msg_->body);
  spirit_utils::multiFootStateMsgToEigen(robot_state_msg_->feet, current_foot_positions_world_);
  local_footstep_planner_->getFootPositionsBodyFrame(current_state_, current_foot_positions_world_,
      current_foot_positions_body_);

  ref_body_plan_.setZero();

  // Check that we have recent twist data, otherwise set cmd_vel to zero
  ros::Duration time_elapsed_since_msg = ros::Time::now() - last_cmd_vel_msg_time_;
  if (time_elapsed_since_msg.toSec() > last_cmd_vel_msg_time_max_) {
    std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0);
    ROS_WARN_THROTTLE(1.0, "No cmd_vel data, setting twist cmd_vel to zero");
  }

  // Integrate to get full body plan
  ref_body_plan_(0,0) = current_state_[0];
  ref_body_plan_(0,1) = current_state_[1];
  ref_body_plan_(0,2) = z_des_;
  ref_body_plan_(0,3) = 0;
  ref_body_plan_(0,4) = 0;
  ref_body_plan_(0,5) = current_state_[5];
  ref_body_plan_(0,6) = cmd_vel_[0];
  ref_body_plan_(0,7) = cmd_vel_[1];
  ref_body_plan_(0,8) = cmd_vel_[2];
  ref_body_plan_(0,9) = cmd_vel_[3];
  ref_body_plan_(0,10) = cmd_vel_[4];
  ref_body_plan_(0,11) = cmd_vel_[5];
  for (int i = 1; i < N_+1; i++) {
    Twist current_cmd_vel = cmd_vel_;

    double yaw = ref_body_plan_(i-1,5);
    current_cmd_vel[0] = cmd_vel_[0]*cos(yaw) - cmd_vel_[1]*sin(yaw);
    current_cmd_vel[1] = cmd_vel_[0]*sin(yaw) + cmd_vel_[1]*cos(yaw);

    for (int j = 0; j < 6; j ++) {
      ref_body_plan_(i,j) = ref_body_plan_(i-1,j) + current_cmd_vel[j]*dt_;
      ref_body_plan_(i,j+6) = (current_cmd_vel[j]);
    }
  }

  // Update the body plan to use for linearization
  if (body_plan_.rows() < N_+1) {
    // Cold start with reference  plan
    body_plan_ = ref_body_plan_;

    // foot_positions_body_ = hip_projected_foot_positions_;
    // foot_positions_world_.setZero();

    // Initialize with the current foot positions
    for (int i = 0; i < N_; i++) {
      foot_positions_body_.row(i) = current_foot_positions_body_;
      foot_positions_world_.row(i) = current_foot_positions_world_;
    }
  } else {
    // Warm start with old solution indexed by one
    body_plan_.topRows(N_) = body_plan_.bottomRows(N_);
    body_plan_.row(N_+1) = ref_body_plan_.row(N_+1);
    // body_plan_.bottomRows(1) = ref_body_plan_.bottomRows(1);

    // No reference for feet so last two elements will be the same
    foot_positions_body_.topRows(N_-1) = foot_positions_body_.bottomRows(N_-1);
    foot_positions_world_.topRows(N_-1) = foot_positions_world_.bottomRows(N_-1);
  }

  // Initialize with current foot and body positions
  body_plan_.row(0) = current_state_;
  foot_positions_body_.row(0) = current_foot_positions_body_;
  foot_positions_world_.row(0) = current_foot_positions_world_;
  //*/
}

bool LocalPlanner::computeLocalPlan() {

  if (terrain_.isEmpty() || body_plan_msg_ == NULL && !use_twist_input_ || robot_state_msg_ == NULL)
    return false;  

  // If desired, start the timer
  spirit_utils::FunctionTimer timer(__FUNCTION__);

  // Compute the contact schedule
  local_footstep_planner_->computeContactSchedule(current_plan_index_, current_state_,
    ref_body_plan_,contact_schedule_);

  // Iteratively generate body and footstep plans (non-parallelizable)
  for (int i = 0; i < iterations_; i++) {

    if (grf_plan_.rows() == N_) {
      // Compute the new footholds
      local_footstep_planner_->computeFootPositions(body_plan_, grf_plan_,
        contact_schedule_, ref_body_plan_, foot_positions_world_);

      // Transform the new foot positions into the body frame for body planning
      local_footstep_planner_->getFootPositionsBodyFrame(body_plan_, foot_positions_world_,
        foot_positions_body_);
    }

    // Compute body plan with MPC, return if solve fails
    if (use_nmpc_)
    {
      if (!local_body_planner_nonlinear_->computeLegPlan(current_state_, ref_body_plan_, foot_positions_body_,
                                            contact_schedule_, body_plan_, grf_plan_))
        return false;
    }
    else
    {
      if (!local_body_planner_convex_->computePlan(current_state_, ref_body_plan_, foot_positions_body_,
                                            contact_schedule_, body_plan_, grf_plan_))
        return false;
    }
  }

  // Record computation time and update exponential filter
  compute_time_ = 1000.0*timer.reportSilent();
  mean_compute_time_ = (filter_smoothing_constant_)*mean_compute_time_ +
    (1-filter_smoothing_constant_)*compute_time_;

  if (compute_time_ >= 1000.0/update_rate_) {
    ROS_WARN_THROTTLE(0.1, "LocalPlanner took %5.3fms, exceeding %5.3fms allowed",
      compute_time_, 1000.0/update_rate_);
  } else {
    ROS_INFO("LocalPlanner took %5.3f ms", compute_time_);
  };
  
  return true;
}

void LocalPlanner::publishLocalPlan() {

  // if (current_plan_index_ >= 50) {
  //   std::cout << "current_state_\n" << current_state_ << std::endl;
  //   std::cout << "ref_body_plan_\n" << ref_body_plan_ << std::endl;
  //   std::cout << "body_plan_\n" << body_plan_ << std::endl;
  //   std::cout << "grf_plan_\n" << grf_plan_ << std::endl;
  //   std::cout << "foot_positions_world_\n" << foot_positions_world_ << std::endl;
  //   std::cout << "foot_positions_body_\n" << foot_positions_body_ << std::endl;
  //   throw std::runtime_error("Stop");
  // }
  
  // Create messages to publish
  spirit_msgs::RobotPlan local_plan_msg;
  spirit_msgs::MultiFootPlanDiscrete future_footholds_msg;
  spirit_msgs::MultiFootPlanContinuous foot_plan_msg;

  // Update the headers of all messages
  ros::Time timestamp = ros::Time::now();
  local_plan_msg.header.stamp = timestamp;
  local_plan_msg.header.frame_id = map_frame_;
  if (!use_twist_input_) {
    local_plan_msg.global_plan_timestamp = body_plan_msg_->global_plan_timestamp;
  } else {
    local_plan_msg.global_plan_timestamp = initial_timestamp_;
  }
  local_plan_msg.compute_time = compute_time_;
  future_footholds_msg.header = local_plan_msg.header;
  foot_plan_msg.header = local_plan_msg.header;

  // Compute the discrete and continuous foot plan messages
  local_footstep_planner_->computeFootPlanMsgs(contact_schedule_, foot_positions_world_,
    current_plan_index_, past_footholds_msg_, future_footholds_msg, foot_plan_msg);

  // Add body, foot, joint, and grf data to the local plan message
  for (int i = 0; i < N_; i++) {

    // Add the state information
    spirit_msgs::RobotState robot_state_msg;
    robot_state_msg.body = spirit_utils::eigenToOdomMsg(body_plan_.row(i));
    robot_state_msg.feet = foot_plan_msg.states[i];
    spirit_utils::ikRobotState(*kinematics_, robot_state_msg);

    // Add the GRF information
    spirit_msgs::GRFArray grf_array_msg;
    spirit_utils::eigenToGRFArrayMsg(grf_plan_.row(i), foot_plan_msg.states[i], grf_array_msg);
    grf_array_msg.contact_states.resize(num_feet_);
    for (int j = 0; j < num_feet_; j++) {
      grf_array_msg.contact_states[j] = contact_schedule_[i][j];
    }
    
    // Update the headers and plan indices of the messages
    ros::Time state_timestamp = local_plan_msg.header.stamp + ros::Duration(i*dt_);
    spirit_utils::updateStateHeaders(robot_state_msg, state_timestamp, map_frame_, current_plan_index_+i);
    grf_array_msg.header = robot_state_msg.header;
    grf_array_msg.traj_index = robot_state_msg.traj_index;

    local_plan_msg.states.push_back(robot_state_msg);
    local_plan_msg.grfs.push_back(grf_array_msg);
    local_plan_msg.plan_indices.push_back(current_plan_index_ + i);
    local_plan_msg.primitive_ids.push_back(2);
  }

  // Publish
  local_plan_pub_.publish(local_plan_msg);
  foot_plan_discrete_pub_.publish(future_footholds_msg);
  foot_plan_continuous_pub_.publish(foot_plan_msg);

  // std::cout << foot_plan_msg << std::endl;
}

void LocalPlanner::spin() {

  ros::Rate r(update_rate_);

  while (ros::ok()) {

    ros::spinOnce();

    // Wait until all required data has been received
    if (terrain_.isEmpty() || body_plan_msg_ == NULL && !use_twist_input_ || robot_state_msg_ == NULL)
      continue;

    
    if (use_twist_input_) {
      // Get twist commands
      getStateAndTwistInput();
    } else {
      // Get the reference plan and robot state into the desired data structures
      getStateAndReferencePlan();
    }

    // Compute the local plan and publish if it solved successfully, otherwise just sleep
    if (computeLocalPlan())
      publishLocalPlan();
    
    r.sleep();
  }
}