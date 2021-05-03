#include "local_planner/local_planner.h"

// namespace plt = matplotlibcpp;
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

LocalPlanner::LocalPlanner(ros::NodeHandle nh) :
  local_body_planner_(), local_footstep_planner_() {

	nh_ = nh;
  
  printf("In constructor\n");

    // Load rosparams from parameter server
  std::string terrain_map_topic, body_plan_topic, robot_state_topic, local_plan_topic,
    foot_plan_discrete_topic, foot_plan_continuous_topic;
  spirit_utils::loadROSParam(nh_, "topics/terrain_map", terrain_map_topic);
  spirit_utils::loadROSParam(nh_, "topics/body_plan", body_plan_topic);
  spirit_utils::loadROSParam(nh_, "topics/state/ground_truth",robot_state_topic);
  spirit_utils::loadROSParam(nh_, "topics/local_plan", local_plan_topic);
  spirit_utils::loadROSParam(nh_, "topics/foot_plan_discrete", foot_plan_discrete_topic);
  spirit_utils::loadROSParam(nh_, "topics/foot_plan_continuous", foot_plan_continuous_topic);
  spirit_utils::loadROSParam(nh_, "map_frame", map_frame_);

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(terrain_map_topic,1, &LocalPlanner::terrainMapCallback, this);
  body_plan_sub_ = nh_.subscribe(body_plan_topic,1, &LocalPlanner::bodyPlanCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic,1,&LocalPlanner::robotStateCallback,this);
  local_plan_pub_ = nh_.advertise<spirit_msgs::LocalPlan>(local_plan_topic,1);
  foot_plan_discrete_pub_ = nh_.advertise<
    spirit_msgs::MultiFootPlanDiscrete>(foot_plan_discrete_topic,1);
  foot_plan_continuous_pub_ = nh_.advertise<
    spirit_msgs::MultiFootPlanContinuous>(foot_plan_continuous_topic,1);


  // Load system parameters
  spirit_utils::loadROSParam(nh_, "local_planner/update_rate", update_rate_);
  spirit_utils::loadROSParam(nh_, "local_planner/timestep",dt_);
  spirit_utils::loadROSParam(nh_, "local_planner/iterations",iterations_);

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

  ref_body_plan_ = Eigen::MatrixXd::Zero(N_, Nx_);
  foot_positions_world_ = Eigen::MatrixXd::Zero(N_,num_feet_*3);
  foot_positions_body_ = Eigen::MatrixXd::Zero(N_,num_feet_*3);
  current_foot_positions_body_ = Eigen::VectorXd::Zero(num_feet_*3);
  current_foot_positions_world_ = Eigen::VectorXd::Zero(num_feet_*3);
  
  // Initialize body and footstep planners
  initLocalBodyPlanner();
  initLocalFootstepPlanner();

  ROS_INFO("LocalPlanner setup complete, waiting for callbacks");

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
  spirit_utils::loadROSParam(nh_, "local_body_planner/state_weights",state_weights);
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
    for (int j = 0; j < 4; ++j) { //for each leg
      Ru(4*i + j,4*i+j) = control_weights.at(i);
    }
  }
  Ru(Nu_-1,Nu_-1) = 1e-6; //gravity weight term

  std::vector<Eigen::MatrixXd> Q_vec(N_+1);
  std::vector<Eigen::MatrixXd> U_vec(N_);
  for (int i = 0; i < N_+1; ++i) {
    Q_vec.at(i) = Qx;
  }

  for (int i = 0; i < N_; ++i) {
    U_vec.at(i) = Ru;
  }

  // Robot body inertia matrix
  Eigen::Matrix3d Ib = Eigen::Matrix3d::Zero();
  Ib.diagonal() << Ixx,Iyy,Izz;

  // Create convex mpc wrapper class
  local_body_planner_ = std::make_shared<QuadrupedMPC>();
  local_body_planner_->setTimestep(dt_);
  local_body_planner_->setMassProperties(m,Ib);
  local_body_planner_->update_weights(Q_vec,U_vec);
  local_body_planner_->update_state_bounds(state_lo, state_hi);
  local_body_planner_->update_control_bounds(normal_lo, normal_hi);
  local_body_planner_->update_friction(mu);

}

void LocalPlanner::initLocalFootstepPlanner() {

  // Load parameters from server
  double grf_weight, ground_clearance, period_d;
  int period;
  spirit_utils::loadROSParam(nh_, "local_footstep_planner/grf_weight", grf_weight);
  spirit_utils::loadROSParam(nh_, "local_footstep_planner/ground_clearance", ground_clearance);
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
  local_footstep_planner_->setSpatialParams(ground_clearance, grf_weight, kinematics_);

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

void LocalPlanner::bodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg) {
  printf("Entering body plan callback\n");
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

  printf("Leaving body plan callback\n");
}

void LocalPlanner::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {

  // Make sure the data is actually populated
  if (msg->feet.feet.empty() || msg->joints.position.empty())
    return;

  robot_state_msg_ = msg;
}

void LocalPlanner::preProcessPlanAndState() {

  // Make sure body plan and robot state data is populated
  if (body_plan_msg_ == NULL || robot_state_msg_ == NULL)
    return;

  // Get index within the global plan
  current_plan_index_ = spirit_utils::getPlanIndex(body_plan_msg_->header.stamp,dt_);

  // Grab the appropriate states from the body plan and convert to an Eigen matrix
  ref_body_plan_.setZero();
  for (int i = 0; i < N_; i++) {
    ref_body_plan_.row(i) = spirit_utils::odomMsgToEigen(body_plan_msg_->states[i+current_plan_index_]);
  }

  // Get the current body and foot positions into Eigen
  current_state_ = spirit_utils::odomMsgToEigen(robot_state_msg_->body);
  spirit_utils::multiFootStateMsgToEigen(robot_state_msg_->feet, current_foot_positions_world_);
  local_footstep_planner_->getFootPositionsBodyFrame(current_state_, current_foot_positions_world_,
      current_foot_positions_body_);

  // Initialize foot positions and contact schedule
  foot_positions_body_ = hip_projected_foot_positions_;
  foot_positions_world_.setZero();
  foot_positions_body_.row(0) = current_foot_positions_body_;
  foot_positions_world_.row(0) = current_foot_positions_world_;
}

void LocalPlanner::computeLocalPlan() {

  if (terrain_.isEmpty() || body_plan_msg_ == NULL || robot_state_msg_ == NULL)
    return;  

  // If desired, start the timer
  spirit_utils::FunctionTimer timer(__FUNCTION__);

  // Determine the contact schedule
  local_footstep_planner_->computeContactSchedule(current_plan_index_, contact_schedule_);

  // Iteratively generate body and footstep plans (non-parallelizable)
  for (int i = 0; i < iterations_; i++) {

        // Compute body plan with MPC
    local_body_planner_->computePlan(current_state_, ref_body_plan_, foot_positions_body_,
      contact_schedule_, body_plan_, grf_plan_);

    // Compute the new footholds to match that body plan
    local_footstep_planner_->computeFootPositions(body_plan_, grf_plan_,
      contact_schedule_, foot_positions_world_);

    local_footstep_planner_->getFootPositionsBodyFrame(body_plan_, foot_positions_world_,
      foot_positions_body_);
  }

  std::cout << ref_body_plan_ << std::endl;
  std::cout << body_plan_ << std::endl;
  std::cout << foot_positions_world_ << std::endl;
  throw std::runtime_error("STOP");

  // If desired, report the function time
  // timer.report();

}

void LocalPlanner::publishLocalPlan() {

  // Create messages to publish
  spirit_msgs::LocalPlan local_plan_msg;
  spirit_msgs::MultiFootPlanDiscrete future_footholds_msg;
  spirit_msgs::MultiFootPlanContinuous foot_plan_msg;

  // Update the headers of all messages
  ros::Time timestamp = ros::Time::now();
  local_plan_msg.header.stamp = timestamp;
  local_plan_msg.header.frame_id = map_frame_;
  local_plan_msg.global_plan_timestamp = body_plan_msg_->header.stamp;
  future_footholds_msg.header = local_plan_msg.header;
  foot_plan_msg.header = local_plan_msg.header;

  // Compute the discrete and continuous foot plan messages
  local_footstep_planner_->computeFootPlanMsgs(contact_schedule_, foot_positions_world_,
    current_plan_index_, past_footholds_msg_, future_footholds_msg, foot_plan_msg);

  // Add body, foot, joint, and grf data to the local plan message
  for (int i = 0; i < N_; i++) {

    ros::Time timestamp = local_plan_msg.header.stamp + ros::Duration(i*dt_);

    spirit_msgs::RobotState robot_state_msg;
    robot_state_msg.body = spirit_utils::eigenToOdomMsg(body_plan_.row(i));
    robot_state_msg.feet = foot_plan_msg.states[i];
    spirit_utils::ikRobotState(*kinematics_, robot_state_msg);

    spirit_msgs::GRFArray grf_array_msg;
    grf_array_msg.header.stamp = timestamp;
    grf_array_msg.header.frame_id = map_frame_;
    grf_array_msg = spirit_utils::eigenToGRFArrayMsg(grf_plan_.row(i), foot_plan_msg.states[i]);
    for (int j = 0; j < num_feet_; j++) {
      grf_array_msg.contact_states.push_back(contact_schedule_[i][j]);
    }
    
    // Update the headers and plan indices of the messages
    spirit_utils::updateStateHeaders(robot_state_msg, timestamp, map_frame_, current_plan_index_+i);
    grf_array_msg.header = robot_state_msg.header;
    grf_array_msg.traj_index = robot_state_msg.traj_index;

    local_plan_msg.states.push_back(robot_state_msg);
    local_plan_msg.grfs.push_back(grf_array_msg);
    local_plan_msg.plan_indices.push_back(current_plan_index_ + i);
  }

  // Publish
  local_plan_pub_.publish(local_plan_msg);
  foot_plan_discrete_pub_.publish(future_footholds_msg);
  foot_plan_continuous_pub_.publish(foot_plan_msg);
}

void LocalPlanner::spin() {

  ros::Rate r(update_rate_);

  while (ros::ok()) {

    ros::spinOnce();

    if (terrain_.isEmpty() || body_plan_msg_ == NULL || robot_state_msg_ == NULL)
      continue;
    
    // Publish local plan data (state reference traj and desired GRF array)
    preProcessPlanAndState();
    computeLocalPlan();
    publishLocalPlan();
    
    r.sleep();
  }
}