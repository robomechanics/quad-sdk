#include "local_planner/local_planner.h"

// namespace plt = matplotlibcpp;
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

LocalPlanner::LocalPlanner(ros::NodeHandle nh) : 
  local_body_planner_convex_(), local_body_planner_nonlinear_(), local_footstep_planner_(), filterChain_("grid_map::GridMap")
{

  nh_ = nh;
  
    // Load rosparams from parameter server
  std::string terrain_map_topic, body_plan_topic, robot_state_topic, local_plan_topic,
    foot_plan_discrete_topic, foot_plan_continuous_topic, cmd_vel_topic, grf_topic, tail_plan_topic;
  quad_utils::loadROSParam(nh_, "topics/terrain_map", terrain_map_topic);
  quad_utils::loadROSParam(nh_, "topics/global_plan", body_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/state/ground_truth",robot_state_topic);
  quad_utils::loadROSParam(nh_, "topics/local_plan", local_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/foot_plan_discrete", foot_plan_discrete_topic);
  quad_utils::loadROSParam(nh_, "topics/foot_plan_continuous", foot_plan_continuous_topic);
  quad_utils::loadROSParam(nh_, "topics/cmd_vel", cmd_vel_topic);
  quad_utils::loadROSParam(nh_, "map_frame", map_frame_);
  quad_utils::loadROSParam(nh_, "/topics/state/grfs", grf_topic);
  quad_utils::loadROSParam(nh_, "/topics/control/tail_plan", tail_plan_topic);

  // Setup pubs and subs
  // terrain_map_sub_ = nh_.subscribe(terrain_map_topic,1, &LocalPlanner::terrainMapCallback, this);
  body_plan_sub_ = nh_.subscribe(body_plan_topic,1, &LocalPlanner::robotPlanCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic,1,&LocalPlanner::robotStateCallback,this);
  cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic,1,&LocalPlanner::cmdVelCallback, this);
  grf_sub_ = nh_.subscribe(grf_topic,1,&LocalPlanner::grfCallback,this);

  local_plan_pub_ = nh_.advertise<quad_msgs::RobotPlan>(local_plan_topic,1);
  foot_plan_discrete_pub_ = nh_.advertise<
    quad_msgs::MultiFootPlanDiscrete>(foot_plan_discrete_topic,1);
  foot_plan_continuous_pub_ = nh_.advertise<
    quad_msgs::MultiFootPlanContinuous>(foot_plan_continuous_topic,1);  
  tail_plan_pub_ = nh_.advertise<quad_msgs::LegCommandArray>(tail_plan_topic, 1);

  // Load system parameters from parameter server
  quad_utils::loadROSParam(nh_, "local_planner/update_rate", update_rate_);
  quad_utils::loadROSParam(nh_, "local_planner/timestep",dt_);
  quad_utils::loadROSParam(nh_, "local_planner/iterations",iterations_);
  quad_utils::loadROSParam(nh_, "twist_body_planner/cmd_vel_scale", cmd_vel_scale_);
  quad_utils::loadROSParam(nh_, "twist_body_planner/last_cmd_vel_msg_time_max",
    last_cmd_vel_msg_time_max_);

  // Load system parameters from launch file (not in config file)
  nh.param<bool>("local_planner/use_nmpc", use_nmpc_, false);
  nh.param<bool>("local_planner/use_twist_input", use_twist_input_, false);
  nh.param<int>("/tail_controller/tail_type", tail_type_, 0);

  // Convert kinematics
quadKD_ = std::make_shared<quad_utils::QuadKD>();

  // Initialize nominal footstep positions projected down from the hips
  Eigen::Vector3d nominal_joint_state;
  nominal_joint_state << 0, 0.78, 1.57; // Default stand angles
  hip_projected_foot_positions_ = Eigen::MatrixXd::Zero(N_,num_feet_*3); 

  for (int i = 0; i < N_; ++i) {
    for (int j = 0; j < num_feet_; ++j) {
      Eigen::Vector3d toe_body_pos;
    quadKD_->bodyToFootFKBodyFrame(j, nominal_joint_state, toe_body_pos);
      hip_projected_foot_positions_.block<1,3>(i,j*3) = toe_body_pos;
    }
  }

  // Initialize body and foot position arrays
  ref_body_plan_ = Eigen::MatrixXd::Zero(N_+1, Nx_);
  foot_positions_world_ = Eigen::MatrixXd::Zero(N_,num_feet_*3);
  foot_positions_body_ = Eigen::MatrixXd::Zero(N_,num_feet_*3);
  current_foot_positions_body_ = Eigen::VectorXd::Zero(num_feet_*3);
  current_foot_positions_world_ = Eigen::VectorXd::Zero(num_feet_*3);
  ref_ground_height_ = Eigen::VectorXd::Zero(N_+1);

  // Initialize body and footstep planners
  initLocalBodyPlanner();
  initLocalFootstepPlanner();

  // Initialize twist input variables
  cmd_vel_.resize(6);
  std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0);
  initial_timestamp_ = ros::Time::now();
  first_plan_ = true;

  // Initialize stand pose
  stand_pose_.fill(std::numeric_limits<double>::max());

  // Initialize footstep history publisher
  foot_step_hist_pub_ = nh_.advertise<grid_map_msgs::GridMap>("/foot_step_hist", 1, true);

  // Initialize footstep history map
  foot_step_hist_ = grid_map::GridMap({"z"});
  foot_step_hist_.setFrameId("map");
  foot_step_hist_.setGeometry(grid_map::Length(10.0, 8.0), 0.1, grid_map::Position(3.0, 0.0));
  foot_step_hist_.clear("z");

  // Initialize footstep history filter
  filterChainParametersName_ = std::string("/local_planner/grid_map_filters");
  if (!filterChain_.configure(filterChainParametersName_, nh))
  {
    ROS_ERROR("Could not configure the filter chain!");
    return;
  }

  // Initialize leg contact miss states vector
  miss_contact_leg_.resize(4);

  // Initialize solve success state boolean
  first_solve_success = false;
}

void LocalPlanner::initLocalBodyPlanner() {

    // Load MPC parameters 
  double m,Ixx,Iyy,Izz,mu,normal_lo, normal_hi;
  quad_utils::loadROSParam(nh_, "local_body_planner/body_mass",m);
  quad_utils::loadROSParam(nh_, "local_body_planner/body_ixx",Ixx);
  quad_utils::loadROSParam(nh_, "local_body_planner/body_iyy",Iyy);
  quad_utils::loadROSParam(nh_, "local_body_planner/body_izz",Izz);
  quad_utils::loadROSParam(nh_, "local_body_planner/friction_mu",mu);
  quad_utils::loadROSParam(nh_, "local_body_planner/normal_lo",normal_lo);
  quad_utils::loadROSParam(nh_, "local_body_planner/normal_hi",normal_hi);

  std::vector<double> state_weights, control_weights, state_lower_bound, state_upper_bound;
  double terminal_weight_scaling;
  quad_utils::loadROSParam(nh_, "local_body_planner/state_weights",state_weights);
  quad_utils::loadROSParam(nh_, "local_body_planner/terminal_weight_scaling",
    terminal_weight_scaling);
  quad_utils::loadROSParam(nh_, "local_body_planner/control_weights",control_weights);
  quad_utils::loadROSParam(nh_, "local_body_planner/state_lower_bound",state_lower_bound);
  quad_utils::loadROSParam(nh_, "local_body_planner/state_upper_bound",state_upper_bound);

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
  if (tail_type_ == CENTRALIZED)
  {
    local_body_planner_nonlinear_ = std::make_shared<NMPCController>(tail_type_);
  }
  else if (use_nmpc_)
  {
    local_body_planner_nonlinear_ = std::make_shared<NMPCController>(NONE);
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
  double grf_weight, ground_clearance, hip_clearance, standing_error_threshold, period_d;
  int period;
  quad_utils::loadROSParam(nh_, "local_footstep_planner/grf_weight", grf_weight);
  quad_utils::loadROSParam(nh_, "local_footstep_planner/ground_clearance", ground_clearance);
  quad_utils::loadROSParam(nh_, "local_footstep_planner/hip_clearance", hip_clearance);
  quad_utils::loadROSParam(nh_, "local_footstep_planner/standing_error_threshold",
    standing_error_threshold);
  quad_utils::loadROSParam(nh_, "local_footstep_planner/period", period_d);

  period = period_d/dt_;

  // Confirm grf weight is valid
  if (grf_weight>1 || grf_weight<0) {
    grf_weight = std::min(std::max(grf_weight,0.0),1.0);
    ROS_WARN("Invalid grf weight, clamping to %4.2f", grf_weight);
  }

  // Create footstep class, make sure we use the same dt as the local planner
  local_footstep_planner_ = std::make_shared<LocalFootstepPlanner>();
  local_footstep_planner_->setTemporalParams(dt_, period, N_);
  local_footstep_planner_->setSpatialParams(ground_clearance, hip_clearance, standing_error_threshold,
    grf_weight,quadKD_);

  past_footholds_msg_.feet.resize(num_feet_);
}

void LocalPlanner::terrainMapCallback(
  const grid_map_msgs::GridMap::ConstPtr& msg) {
  // grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, terrain_grid_);

  // Convert to FastTerrainMap structure for faster querying
  terrain_.loadDataFromGridMap(terrain_grid_);
  local_footstep_planner_->updateMap(terrain_);
  local_footstep_planner_->updateMap(terrain_grid_);
}

void LocalPlanner::robotPlanCallback(const quad_msgs::RobotPlan::ConstPtr& msg) {
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

void LocalPlanner::robotStateCallback(const quad_msgs::RobotState::ConstPtr& msg) {

  // Make sure the data is actually populated
  if (msg->feet.feet.empty() || msg->joints.position.empty())
    return;

  robot_state_msg_ = msg;
}

void LocalPlanner::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {

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

void LocalPlanner::grfCallback(const quad_msgs::GRFArray::ConstPtr &msg)
{
  grf_msg_ = msg;
}

void LocalPlanner::getStateAndReferencePlan() {

  // Make sure body plan and robot state data is populated
  if (body_plan_msg_ == NULL || robot_state_msg_ == NULL)
    return;

  // Get index within the global plan
  current_plan_index_ = quad_utils::getPlanIndex(body_plan_msg_->global_plan_timestamp,dt_);

  // Get the current body and foot positions into Eigen
  current_state_ = quad_utils::bodyStateMsgToEigen(robot_state_msg_->body);
  current_state_timestamp_ = robot_state_msg_->header.stamp;
  quad_utils::multiFootStateMsgToEigen(robot_state_msg_->feet, current_foot_positions_world_);
  local_footstep_planner_->getFootPositionsBodyFrame(current_state_, current_foot_positions_world_,
      current_foot_positions_body_);

  // Grab the appropriate states from the body plan and convert to an Eigen matrix
  ref_body_plan_.setZero();
  for (int i = 0; i < N_+1; i++) {

    // If the horizon extends past the reference trajectory, just hold the last state
    if (i+current_plan_index_ > body_plan_msg_->plan_indices.back()) {
      ref_body_plan_.row(i) = quad_utils::bodyStateMsgToEigen(body_plan_msg_->states.back().body);
    } else {
      ref_body_plan_.row(i) = quad_utils::bodyStateMsgToEigen(body_plan_msg_->states[i+current_plan_index_].body);
    }

    ref_ground_height_(i) = local_footstep_planner_->getTerrainHeight(ref_body_plan_(i, 0), ref_body_plan_(i, 1));
  }

  ref_ground_height_(0) = local_footstep_planner_->getTerrainHeight(current_state_(0), current_state_(1));

  if (tail_type_ == CENTRALIZED)
  {
    tail_current_state_ = quad_utils::odomMsgToEigenForTail(*robot_state_msg_);
    ref_tail_plan_ = Eigen::MatrixXd::Zero(N_ + 1, 4);
    ref_tail_plan_.row(0) = tail_current_state_.transpose();
  }

  // Update the body plan to use for linearization
  if (body_plan_.rows() < N_+1) {
    // Cold start with reference  plan
    body_plan_ = ref_body_plan_;

    // Initialize with the current foot positions
    for (int i = 0; i < N_; i++) {
      foot_positions_body_.row(i) = current_foot_positions_body_;
      foot_positions_world_.row(i) = current_foot_positions_world_;
    }
  } else {
    // Warm start with old solution indexed by one
    body_plan_.topRows(N_) = body_plan_.bottomRows(N_);
    body_plan_.row(N_+1) = ref_body_plan_.row(N_+1);

    // No reference for feet so last two elements will be the same
    foot_positions_body_.topRows(N_-1) = foot_positions_body_.bottomRows(N_-1);
    foot_positions_world_.topRows(N_-1) = foot_positions_world_.bottomRows(N_-1);
  }

  // Initialize with current foot and body positions
  body_plan_.row(0) = current_state_;
  foot_positions_body_.row(0) = current_foot_positions_body_;
  foot_positions_world_.row(0) = current_foot_positions_world_;
}

void LocalPlanner::publishFootStepHist()
{
  if (robot_state_msg_ == NULL || grf_msg_ == NULL)
    return;

  // Get the current body and foot positions into Eigen
  current_state_ = quad_utils::bodyStateMsgToEigen(robot_state_msg_->body);
  current_state_timestamp_ = robot_state_msg_->header.stamp;
  quad_utils::multiFootStateMsgToEigen(robot_state_msg_->feet,
                                         current_foot_positions_world_);
  local_footstep_planner_->getFootPositionsBodyFrame(current_state_,
                                                     current_foot_positions_world_,
                                                     current_foot_positions_body_);

  for (size_t i = 0; i < 4; i++)
  {
    // Check if it's on the ground
    if (grf_msg_->contact_states.at(i) && sqrt(pow(grf_msg_->vectors.at(i).x, 2) +
                                               pow(grf_msg_->vectors.at(i).y, 2) +
                                               pow(grf_msg_->vectors.at(i).z, 2)) > 1)
    {
      // Add the foot position to the history
      foot_step_hist_.atPosition("z", grid_map::Position(current_foot_positions_world_(3 * i + 0),
                                                         current_foot_positions_world_(3 * i + 1))) = current_foot_positions_world_(3 * i + 2);
    }
  }

  // Apply filter
  if (!filterChain_.update(foot_step_hist_, terrain_grid_))
  {
    ROS_ERROR("Could not update the grid map filter chain!");
    return;
  }

  // Update footstep planner
  local_footstep_planner_->updateMap(terrain_grid_);

  // Publish footstep history
  grid_map_msgs::GridMap foot_step_hist_message;
  grid_map::GridMapRosConverter::toMessage(terrain_grid_, foot_step_hist_message);
  foot_step_hist_pub_.publish(foot_step_hist_message);
}

void LocalPlanner::getStateAndTwistInput() {

  if (!terrain_grid_.exists("z_smooth") || robot_state_msg_ == NULL || grf_msg_ == NULL)
    return;

  // The first couple solving might fail, we want to aligh all the gait  
  if (!first_solve_success)
  {
    initial_timestamp_ = ros::Time::now();
  }

  // Get index
  current_plan_index_ = quad_utils::getPlanIndex(initial_timestamp_,dt_);

  // Initializing foot positions if not data has arrived
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
  current_state_ = quad_utils::bodyStateMsgToEigen(robot_state_msg_->body);
  current_state_timestamp_ = robot_state_msg_->header.stamp;
  quad_utils::multiFootStateMsgToEigen(robot_state_msg_->feet, current_foot_positions_world_);
  local_footstep_planner_->getFootPositionsBodyFrame(current_state_, current_foot_positions_world_,
      current_foot_positions_body_);

  // Clear any old reference plans
  ref_body_plan_.setZero();

  // Assign a sidewalk velocity
  cmd_vel_.at(1) = 0.5;

  // Set initial ground height
  ref_ground_height_(0) = local_footstep_planner_->getTerrainHeight(current_state_(0), current_state_(1));

  // If it's not initialized, set to current positions
  if (stand_pose_(0) == std::numeric_limits<double>::max() && stand_pose_(1) == std::numeric_limits<double>::max() && stand_pose_(2) == std::numeric_limits<double>::max())
  {
    stand_pose_ << current_state_[0], current_state_[1], current_state_[5];
  }

  // If it's going to walk, use latest states
  if (cmd_vel_[0] != 0 || cmd_vel_[1] != 0 || cmd_vel_[5] != 0)
  {
    stand_pose_ << current_state_[0], current_state_[1], current_state_[5];
  }
  // If it's standing, try to stablized the waggling
  else
  {
    Eigen::Vector3d current_stand_pose;
    current_stand_pose << current_state_[0], current_state_[1], current_state_[5];
    stand_pose_ = stand_pose_ * (1 - 1 / update_rate_) + current_stand_pose * 1 / update_rate_;
  }

  // Set initial condition for forward integration
  ref_body_plan_(0,0) = current_state_[0];
  ref_body_plan_(0,1) = current_state_[1];
  ref_body_plan_(0,2) = z_des_ + ref_ground_height_(0);
  ref_body_plan_(0,3) = 0;
  ref_body_plan_(0,4) = 0;
  ref_body_plan_(0,5) = current_state_[5];
  ref_body_plan_(0,6) = cmd_vel_[0]*cos(current_state_[5]) - cmd_vel_[1]*sin(current_state_[5]);
  ref_body_plan_(0,7) = cmd_vel_[0]*sin(current_state_[5]) + cmd_vel_[1]*cos(current_state_[5]);
  ref_body_plan_(0,8) = cmd_vel_[2];
  ref_body_plan_(0,9) = cmd_vel_[3];
  ref_body_plan_(0,10) = cmd_vel_[4];
  ref_body_plan_(0,11) = cmd_vel_[5];

  // Only adaptive pitch
  // ref_body_plan_(0, 4) = local_footstep_planner_->getTerrainSlope(current_state_(0), current_state_(1), current_state_(6), current_state_(7));
  
  // Adaptive roll and pitch
  local_footstep_planner_->getTerrainSlope(ref_body_plan_(0, 0),
                                           ref_body_plan_(0, 1),
                                           ref_body_plan_(0, 5),
                                           ref_body_plan_(0, 3),
                                           ref_body_plan_(0, 4));

  // Integrate to get full body plan (Forward Euler)
  for (int i = 1; i < N_+1; i++) {
    Twist current_cmd_vel = cmd_vel_;

    double yaw = ref_body_plan_(i-1,5);
    current_cmd_vel[0] = cmd_vel_[0]*cos(yaw) - cmd_vel_[1]*sin(yaw);
    current_cmd_vel[1] = cmd_vel_[0]*sin(yaw) + cmd_vel_[1]*cos(yaw);

    for (int j = 0; j < 6; j ++) {
      ref_body_plan_(i,j) = ref_body_plan_(i-1,j) + current_cmd_vel[j]*dt_;
      ref_body_plan_(i,j+6) = (current_cmd_vel[j]);
    }

    ref_ground_height_(i) = local_footstep_planner_->getTerrainHeight(ref_body_plan_(i, 0), ref_body_plan_(i, 1));
    ref_body_plan_(i, 2) = z_des_ + ref_ground_height_(i);

    // Only adaptive pitch
    // ref_body_plan_(i, 4) = local_footstep_planner_->getTerrainSlope(ref_body_plan_(i, 0), ref_body_plan_(i, 1), ref_body_plan_(i, 6), ref_body_plan_(i, 7));
    
    // Adaptive roll and pitch
    local_footstep_planner_->getTerrainSlope(ref_body_plan_(i, 0),
                                             ref_body_plan_(i, 1),
                                             ref_body_plan_(i, 5),
                                             ref_body_plan_(i, 3),
                                             ref_body_plan_(i, 4));
  }

  if (tail_type_ == CENTRALIZED)
  {
    tail_current_state_ = quad_utils::odomMsgToEigenForTail(*robot_state_msg_);
    ref_tail_plan_ = Eigen::MatrixXd::Zero(N_ + 1, 4);
  }

  // Update the body plan to use for linearization
  if (body_plan_.rows() < N_+1) {
    // Cold start with reference  plan
    body_plan_ = ref_body_plan_;

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
}

bool LocalPlanner::computeLocalPlan() {

  if (!terrain_grid_.exists("z_smooth") || body_plan_msg_ == NULL && !use_twist_input_ || robot_state_msg_ == NULL || grf_msg_ == NULL)
    return false;  

  // Start the timer
  quad_utils::FunctionTimer timer(__FUNCTION__);

  // Compute the contact schedule
  local_footstep_planner_->computeContactSchedule(current_plan_index_, current_state_,
    ref_body_plan_,contact_schedule_);

  // Compute the new footholds if we have a valid existing plan (i.e. if grf_plan is filled)
  if (grf_plan_.rows() == N_) {
    
    local_footstep_planner_->computeFootPositions(body_plan_, grf_plan_,
      contact_schedule_, ref_body_plan_, foot_positions_world_);

    // Transform the new foot positions into the body frame for body planning
    local_footstep_planner_->getFootPositionsBodyFrame(body_plan_, foot_positions_world_,
      foot_positions_body_);
  }

  // TODO: this should be move to local footstep planner since we'll need the hip position and hip height for reference
  // Contact sensing
  adpative_contact_schedule_ = contact_schedule_;

  // Late contact 
  for (size_t i = 0; i < 4; i++)
  {
    // Check foot distance
    Eigen::VectorXd foot_position_vec = current_foot_positions_body_.segment(3 * i, 3);

    // ROS_WARN_STREAM("dis: " << foot_position_vec.norm());
    // ROS_WARN_STREAM("height: " << foot_position_vec(2));

    // If it stretches too long, we assume miss contact
    // if (foot_position_vec.norm() > 0.45)
    if (foot_position_vec(2) < -0.35)
    {
      ROS_WARN_STREAM("miss!");
      miss_contact_leg_.at(i) = true;
    }

    // It will recover only when contact sensor report it hits ground
    if (contact_schedule_.at(0).at(i) && bool(grf_msg_->contact_states.at(i)) && grf_msg_->vectors.at(i).z > 14 && miss_contact_leg_.at(i))
    {
      miss_contact_leg_.at(i) = false;
    }

    if (miss_contact_leg_.at(i))
    {
      Eigen::Vector3d hip_position, foot_shift, nominal_pos;
      // kinematics_->nominalHipFK(i, body_plan_.block(0, 0, 1, 3).transpose(), body_plan_.block(0, 3, 1, 3).transpose(), hip_position);
      // foot_shift = foot_positions_world_.block(0, i * 3, 1, 3).transpose() - hip_position;
      Eigen::Matrix3d rotation_matrix, body_rotation_matrix;
      // double rx = -foot_shift(1) / sqrt(foot_shift(0) * foot_shift(0) + foot_shift(1) * foot_shift(1));
      // double ry = foot_shift(0) / sqrt(foot_shift(0) * foot_shift(0) + foot_shift(1) * foot_shift(1));
      double theta = 0.707;
      double yaw = body_plan_(0, 5);
      // rotation_matrix << (1 - cos(theta)) * rx * rx + cos(theta),
      //     -rx * ry * (cos(theta) - 1),
      //     ry * sin(theta),
      //     -rx * ry * (cos(theta) - 1),
      //     (1 - cos(theta)) * ry * ry + cos(theta),
      //     -rx * sin(theta),
      //     -ry * sin(theta),
      //     rx * sin(theta), cos(theta);
      body_rotation_matrix << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
      body_rotation_matrix = body_rotation_matrix.transpose();
      // rotation_matrix << cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0, cos(theta);
      rotation_matrix << 1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta);
      rotation_matrix = rotation_matrix.transpose();
      nominal_pos << 0, 0, -0.3;

      // We assume it will not touch the ground at this gait peroid
      for (size_t j = 0; j < N_; j++)
      {
        if (contact_schedule_.at(j).at(i))
        {
          ROS_WARN_STREAM("shift leg: " << i);
          adpative_contact_schedule_.at(j).at(i) = false;
          if (j > 0)
          {
            // We try to move the leg more to balance the body
            // foot_positions_world_(j, i * 3 + 0) = foot_positions_world_(j, i * 3 + 0) - 0.1;
            // foot_positions_body_(j, i * 3 + 0) = foot_positions_body_(j, i * 3 + 0) - 0.1;

            Eigen::Vector3d hip_position_plan;
            quadKD_->worldToNominalHipFKWorldFrame(i, body_plan_.block(j, 0, 1, 3).transpose(), body_plan_.block(j, 3, 1, 3).transpose(), hip_position_plan);
            foot_positions_world_.block(j, i * 3, 1, 3) = (body_rotation_matrix * rotation_matrix * nominal_pos + hip_position_plan).transpose();
            foot_positions_body_.block(j, i * 3, 1, 3) = (body_rotation_matrix * rotation_matrix * nominal_pos + hip_position_plan).transpose() - body_plan_.block(j, 0, 1, 3);
          }
        }
        else
        {
          if (j > 0)
          {
            Eigen::Vector3d hip_position_plan;
            quadKD_->worldToNominalHipFKWorldFrame(i, body_plan_.block(j, 0, 1, 3).transpose(), body_plan_.block(j, 3, 1, 3).transpose(), hip_position_plan);
            foot_positions_world_.block(j, i * 3, 1, 3) = (body_rotation_matrix * rotation_matrix * nominal_pos + hip_position_plan).transpose();
            foot_positions_body_.block(j, i * 3, 1, 3) = (body_rotation_matrix * rotation_matrix * nominal_pos + hip_position_plan).transpose() - body_plan_.block(j, 0, 1, 3);
          }

          if (contact_schedule_.at(j + 1).at(i))
          {
            break;
          }
        }
      }
    }

    // // Later contact
    // // if (contact_schedule_.at(0).at(i) && abs(current_foot_positions_world_(2 + i * 3) - current_state_(2)) > 0.325)
    // if (contact_schedule_.at(0).at(i) && !grf_msg_->contact_states.at(i))
    // {
    //   sense_miss_contact = true;
    //   // If not, we assume it will not touch the ground at this gait peroid
    //   for (size_t j = 0; j < N_; j++)
    //   {
    //     if (contact_schedule_.at(j).at(i))
    //     {
    //       adpative_contact_schedule_.at(j).at(i) = false;
    //     }
    //     else
    //     {
    //       break;
    //     }
    //   }
    // }

    // // Early contact
    // // if (contact_schedule_.at(0).at(i) && abs(current_foot_positions_world_(2 + i * 3) - current_state_(2)) > 0.325)
    // if (!contact_schedule_.at(0).at(i) && contact_schedule_.at(5).at(i) && grf_msg_->contact_states.at(i))
    // {
    //   sense_miss_contact = true;
    //   // If so, we assume it will not touch the ground at this gait peroid
    //   for (size_t j = 0; j < N_; j++)
    //   {
    //     if (!contact_schedule_.at(j).at(i))
    //     {
    //       adpative_contact_schedule_.at(j).at(i) = true;
    //     }
    //     else
    //     {
    //       break;
    //     }
    //   }
    // }
  }

  // Compute body plan with MPC, return if solve fails
  if (tail_type_ == CENTRALIZED)
  {
    // TODO: we've not yet implement ground height verison for centralized tail
    if (!local_body_planner_nonlinear_->computeCentralizedTailPlan(current_state_,
                                                                   ref_body_plan_,
                                                                   foot_positions_body_,
                                                                   adpative_contact_schedule_,
                                                                   tail_current_state_,
                                                                   ref_tail_plan_,
                                                                   body_plan_,
                                                                   grf_plan_,
                                                                   tail_plan_,
                                                                   tail_torque_plan_))
      return false;
  }
  else if (use_nmpc_)
  {
    if (!local_body_planner_nonlinear_->computeLegPlan(current_state_,
                                                       ref_body_plan_,
                                                       foot_positions_body_,
                                                       adpative_contact_schedule_,
                                                       ref_ground_height_,
                                                       body_plan_,
                                                       grf_plan_))
      return false;
  }
  else
  {
    if (!local_body_planner_convex_->computePlan(current_state_,
                                                 ref_body_plan_,
                                                 foot_positions_body_,
                                                 contact_schedule_,
                                                 body_plan_,
                                                 grf_plan_))
      return false;
  }

  // We are here, meaning we get the solving success
  first_solve_success = true;

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
  
  // Return true if made it this far
  return true;
}

void LocalPlanner::publishLocalPlan() {

  // Create messages to publish
  quad_msgs::RobotPlan local_plan_msg;
  quad_msgs::MultiFootPlanDiscrete future_footholds_msg;
  quad_msgs::MultiFootPlanContinuous foot_plan_msg;
  quad_msgs::LegCommandArray tail_plan_msg;

  // Update the headers of all messages
  ros::Time timestamp = robot_state_msg_->header.stamp;
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

  if (tail_type_ == CENTRALIZED)
  {
    tail_plan_msg.header = local_plan_msg.header;
  }

  // Compute the discrete and continuous foot plan messages
  local_footstep_planner_->computeFootPlanMsgs(contact_schedule_, foot_positions_world_,
    current_plan_index_, body_plan_, past_footholds_msg_, future_footholds_msg, foot_plan_msg);

  // Add body, foot, joint, and grf data to the local plan message
  for (int i = 0; i < N_; i++) {

    // Add the state information
    quad_msgs::RobotState robot_state_msg;
    robot_state_msg.body = quad_utils::eigenToBodyStateMsg(body_plan_.row(i));
    robot_state_msg.feet = foot_plan_msg.states[i];
    quad_utils::ikRobotState(*quadKD_, robot_state_msg);

    // Add the GRF information
    quad_msgs::GRFArray grf_array_msg;
    quad_utils::eigenToGRFArrayMsg(grf_plan_.row(i), foot_plan_msg.states[i], grf_array_msg);
    grf_array_msg.contact_states.resize(num_feet_);
    for (int j = 0; j < num_feet_; j++) {
      grf_array_msg.contact_states[j] = contact_schedule_[i][j];
    }

    // Update the headers and plan indices of the messages
    ros::Time state_timestamp = local_plan_msg.header.stamp + ros::Duration(i*dt_);
    quad_utils::updateStateHeaders(robot_state_msg, state_timestamp, map_frame_, current_plan_index_+i);
    grf_array_msg.header = robot_state_msg.header;
    grf_array_msg.traj_index = robot_state_msg.traj_index;

    local_plan_msg.states.push_back(robot_state_msg);
    local_plan_msg.grfs.push_back(grf_array_msg);
    local_plan_msg.plan_indices.push_back(current_plan_index_ + i);
    local_plan_msg.primitive_ids.push_back(2);

    if (tail_type_ == CENTRALIZED)
    {
      quad_msgs::LegCommand tail_msg;
      tail_msg.motor_commands.resize(2);

      tail_msg.motor_commands.at(0).pos_setpoint = tail_plan_(i, 0);
      tail_msg.motor_commands.at(0).vel_setpoint = tail_plan_(i, 2);
      tail_msg.motor_commands.at(0).torque_ff = tail_torque_plan_(i, 0);

      tail_msg.motor_commands.at(1).pos_setpoint = tail_plan_(i, 1);
      tail_msg.motor_commands.at(1).vel_setpoint = tail_plan_(i, 3);
      tail_msg.motor_commands.at(1).torque_ff = tail_torque_plan_(i, 1);

      tail_plan_msg.leg_commands.push_back(tail_msg);
    }
  }

  // Publish
  local_plan_msg.state_timestamp = current_state_timestamp_;
  local_plan_pub_.publish(local_plan_msg);
  foot_plan_discrete_pub_.publish(future_footholds_msg);
  foot_plan_continuous_pub_.publish(foot_plan_msg);

  if (tail_type_ == CENTRALIZED)
  {
    tail_plan_pub_.publish(tail_plan_msg);
  }
}

void LocalPlanner::spin() {

  ros::Rate r(update_rate_);

  while (ros::ok()) {

    ros::spinOnce();

    publishFootStepHist();

    // Wait until all required data has been received
    if (!(!terrain_grid_.exists("z_smooth") || body_plan_msg_ == NULL && !use_twist_input_ || robot_state_msg_ == NULL))
    {
      if (use_twist_input_)
      {
        // Get twist commands
        getStateAndTwistInput();
      }
      else
      {
        // Get the reference plan and robot state into the desired data structures
        getStateAndReferencePlan();
      }

      // Compute the local plan and publish if it solved successfully, otherwise just sleep
      if (computeLocalPlan())
        publishLocalPlan();
    }

    r.sleep();
  }
}