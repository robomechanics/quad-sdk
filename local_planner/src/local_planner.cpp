#include "local_planner/local_planner.h"

// namespace plt = matplotlibcpp;
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

LocalPlanner::LocalPlanner(ros::NodeHandle nh) : 
  local_body_planner_convex_(), local_body_planner_nonlinear_(), local_footstep_planner_(), filterChain_("grid_map::GridMap")
{

  nh_ = nh;
  
    // Load rosparams from parameter server
  std::string terrain_map_topic, body_plan_topic, robot_state_topic, local_plan_topic,
    foot_plan_discrete_topic, foot_plan_continuous_topic, cmd_vel_topic, grf_topic, 
    tail_plan_topic, contact_sensing_topic;
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
  quad_utils::loadROSParam(nh_,"topics/control/contact_sensing",contact_sensing_topic);

  // Setup pubs and subs
  // terrain_map_sub_ = nh_.subscribe(terrain_map_topic,1, &LocalPlanner::terrainMapCallback, this, ros::TransportHints().tcpNoDelay(true));
  body_plan_sub_ = nh_.subscribe(body_plan_topic, 1, &LocalPlanner::robotPlanCallback, this, ros::TransportHints().tcpNoDelay(true));
  robot_state_sub_ = nh_.subscribe(robot_state_topic, 1, &LocalPlanner::robotStateCallback, this, ros::TransportHints().tcpNoDelay(true));
  cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic, 1, &LocalPlanner::cmdVelCallback, this, ros::TransportHints().tcpNoDelay(true));
  grf_sub_ = nh_.subscribe(grf_topic, 1, &LocalPlanner::grfCallback, this, ros::TransportHints().tcpNoDelay(true));
  contact_sensing_sub_ = nh_.subscribe(contact_sensing_topic, 1, &LocalPlanner::contactSensingCallback, this, ros::TransportHints().tcpNoDelay(true));

  local_plan_pub_ = nh_.advertise<quad_msgs::RobotPlan>(local_plan_topic,1);
  foot_plan_discrete_pub_ = nh_.advertise<
    quad_msgs::MultiFootPlanDiscrete>(foot_plan_discrete_topic,1);
  foot_plan_continuous_pub_ = nh_.advertise<
    quad_msgs::MultiFootPlanContinuous>(foot_plan_continuous_topic,1);  
  tail_plan_pub_ = nh_.advertise<quad_msgs::LegCommandArray>(tail_plan_topic, 1);

  // Load system parameters from parameter server
  quad_utils::loadROSParam(nh_, "local_planner/update_rate", update_rate_);
  quad_utils::loadROSParam(nh_, "local_planner/timestep",dt_);
  quad_utils::loadROSParam(nh_, "local_planner/horizon_length",N_);
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
  current_foot_velocities_world_ = Eigen::VectorXd::Zero(num_feet_*3);
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
  contact_sensing_.assign(4, false);

  // Initialize the temporary foothold index vector
  tmp_foot_hist_idx_.resize(4);

  // Initialize foor position record for missing foot
  foot_pos_body_miss_contact_ = Eigen::VectorXd::Zero(12);

  // Initialize solve success state boolean
  first_solve_success = false;
  
  // Initialize the time duration to the next plan index
  first_element_duration_ = dt_;

  // Initialize the plan index boolean
  same_plan_index_ = true;

  // Initialize estimated ground height
  ground_height_ = std::numeric_limits<double>::max();
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
  std::vector<double> duty_cycles, phase_offsets;
  quad_utils::loadROSParam(nh_, "local_footstep_planner/grf_weight", grf_weight);
  quad_utils::loadROSParam(nh_, "local_footstep_planner/ground_clearance", ground_clearance);
  quad_utils::loadROSParam(nh_, "local_footstep_planner/hip_clearance", hip_clearance);
  quad_utils::loadROSParam(nh_, "local_footstep_planner/standing_error_threshold",
    standing_error_threshold);
  quad_utils::loadROSParam(nh_, "local_footstep_planner/period", period_d);
  quad_utils::loadROSParam(nh_, "local_footstep_planner/duty_cycles", duty_cycles);
  quad_utils::loadROSParam(nh_, "local_footstep_planner/phase_offsets", phase_offsets);

  period = period_d/dt_;

  // Confirm grf weight is valid
  if (grf_weight>1 || grf_weight<0) {
    grf_weight = std::min(std::max(grf_weight,0.0),1.0);
    ROS_WARN("Invalid grf weight, clamping to %4.2f", grf_weight);
  }

  // Create footstep class, make sure we use the same dt as the local planner
  local_footstep_planner_ = std::make_shared<LocalFootstepPlanner>();
  local_footstep_planner_->setTemporalParams(dt_, period, N_, duty_cycles, phase_offsets);
  local_footstep_planner_->setSpatialParams(ground_clearance, hip_clearance, standing_error_threshold,
    grf_weight,quadKD_);

  past_footholds_msg_.feet.resize(num_feet_);

  future_nominal_footholds_msg_.feet.resize(num_feet_);
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
  cmd_vel_[0] = 0.95*cmd_vel_[0]+0.05*cmd_vel_scale_*msg->linear.x;
  cmd_vel_[1] = 0.95*cmd_vel_[1]+0.05*cmd_vel_scale_*msg->linear.y;
  cmd_vel_[2] = 0;
  cmd_vel_[3] = 0;
  cmd_vel_[4] = 0;
  cmd_vel_[5] = 0.95*cmd_vel_[5]+0.05*cmd_vel_scale_*msg->angular.z;

  // Record when this was last reached for safety
  last_cmd_vel_msg_time_ = ros::Time::now();

}

void LocalPlanner::grfCallback(const quad_msgs::GRFArray::ConstPtr &msg)
{
  grf_msg_ = msg;
}

void LocalPlanner::contactSensingCallback(const std_msgs::ByteMultiArray::ConstPtr &msg)
{
  contact_sensing_msg_ = msg;
}

void LocalPlanner::getStateAndReferencePlan() {

  // Make sure body plan and robot state data is populated
  if (body_plan_msg_ == NULL || robot_state_msg_ == NULL)
    return;

  // Get index within the global plan, compare with the previous one to check if this is a duplicated solve
  int previous_plan_index = current_plan_index_;
  quad_utils::getPlanIndex(body_plan_msg_->global_plan_timestamp, dt_, current_plan_index_, first_element_duration_);
  same_plan_index_ = previous_plan_index == current_plan_index_;

  // Get the current body and foot positions into Eigen
  current_state_ = quad_utils::bodyStateMsgToEigen(robot_state_msg_->body);
  current_state_timestamp_ = robot_state_msg_->header.stamp;
  quad_utils::multiFootStateMsgToEigen(robot_state_msg_->feet, current_foot_positions_world_, current_foot_velocities_world_);
  local_footstep_planner_->getFootPositionsBodyFrame(current_state_, current_foot_positions_world_,
      current_foot_positions_body_);

  // TODO: I don't think this is compatible with the adpative first step now. We might need to interplate it.
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
    // Only shift the foot position if it's a solve for a new plan index
    if (!same_plan_index_)
    {
      body_plan_.topRows(N_) = body_plan_.bottomRows(N_);
      grf_plan_.topRows(N_-1) = grf_plan_.bottomRows(N_-1);

      foot_positions_body_.topRows(N_-1) = foot_positions_body_.bottomRows(N_-1);
      foot_positions_world_.topRows(N_-1) = foot_positions_world_.bottomRows(N_-1);
    }
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
    if ((grf_msg_->contact_states.at(i) && abs(grf_msg_->vectors.at(i).z) > 5))
    {
      if (!tmp_foot_hist_idx_.at(i).empty())
      {
        // If there's temporary index, replace them all with the actual contact value
        for (size_t j = 0; j < tmp_foot_hist_idx_.at(i).size(); j++)
        {
          foot_step_hist_.at("z", tmp_foot_hist_idx_.at(i).at(j)) = current_foot_positions_world_(3 * i + 2) - toe_radius;
        }

        // Clear the temporary index vector
        tmp_foot_hist_idx_.at(i).clear();
      }

      // Add the foot position to the history
      foot_step_hist_.atPosition("z", grid_map::Position(current_foot_positions_world_(3 * i + 0),
                                                         current_foot_positions_world_(3 * i + 1))) =
          current_foot_positions_world_(3 * i + 2) - toe_radius;
    }
    else if (terrain_grid_.exists("z_inpainted"))
    {
      // Check if the current foot is lower than the records
      if (current_foot_positions_world_(3 * i + 2) - toe_radius <
          terrain_grid_.atPosition("z_inpainted", grid_map::Position(current_foot_positions_world_(3 * i + 0), current_foot_positions_world_(3 * i + 1)),
                                   grid_map::InterpolationMethods::INTER_LINEAR))
      {
        // Temporary record the position
        grid_map::Index tmp_idx;
        foot_step_hist_.getIndex(grid_map::Position(current_foot_positions_world_(3 * i + 0),
                                                    current_foot_positions_world_(3 * i + 1)),
                                 tmp_idx);
        tmp_foot_hist_idx_.at(i).push_back(tmp_idx);

        // Replace all the previous record with the latest position
        for (size_t j = 0; j < tmp_foot_hist_idx_.at(i).size(); j++)
        {
          foot_step_hist_.at("z", tmp_foot_hist_idx_.at(i).at(j)) = current_foot_positions_world_(3 * i + 2) - toe_radius;
        }
      }
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
    initial_timestamp_ = ros::Time::now()-ros::Duration(1e-6);
    enter_time_ = ros::Time::now();
  }

  // Get plan index, compare with the previous one to check if this is a duplicated solve
  int previous_plan_index = current_plan_index_;
  quad_utils::getPlanIndex(initial_timestamp_, dt_, current_plan_index_, first_element_duration_);
  same_plan_index_ = previous_plan_index == current_plan_index_;

  // Initializing foot positions if not data has arrived
  if (first_plan_) {
    first_plan_ = false;
    past_footholds_msg_.header = robot_state_msg_->header;
    for (int i = 0; i < num_feet_; i++) {
      past_footholds_msg_.feet[i].footholds.clear();
      past_footholds_msg_.feet[i].footholds.push_back(robot_state_msg_->feet.feet[i]);
      past_footholds_msg_.feet[i].footholds.front().header = past_footholds_msg_.header;
      past_footholds_msg_.feet[i].footholds.front().traj_index = 0;

      future_nominal_footholds_msg_.feet[i] = robot_state_msg_->feet.feet[i];
    }
  }

  // Get the current body and foot positions into Eigen
  current_state_ = quad_utils::bodyStateMsgToEigen(robot_state_msg_->body);
  current_state_timestamp_ = robot_state_msg_->header.stamp;
  quad_utils::multiFootStateMsgToEigen(robot_state_msg_->feet, current_foot_positions_world_, current_foot_velocities_world_);
  local_footstep_planner_->getFootPositionsBodyFrame(current_state_, current_foot_positions_world_,
      current_foot_positions_body_);

  // Clear any old reference plans
  ref_body_plan_.setZero();

  // Assign a sidewalk velocity
  cmd_vel_.at(1) = 0.75;

  // Adaptive body height, use the lowest foot and exponential filter
  std::vector<double> foot_height;
  for (size_t i = 0; i < 4; i++)
  {
    foot_height.push_back(current_foot_positions_world_(3 * i + 2));
  }

  // Estimate the ground height using flat assumption
  double time_diff = (ros::Time::now() - enter_time_).toSec();
  local_footstep_planner_->time_diff_ = time_diff;
  enter_time_ = ros::Time::now();

  if (ground_height_ == std::numeric_limits<double>::max())
  {
    ground_height_ = std::max(*std::min_element(foot_height.begin(), foot_height.end()), 0.0);
  }
  else
  {
    ground_height_ = (1 - time_diff / 0.04) * std::max(*std::min_element(foot_height.begin(), foot_height.end()), 0.0) +
                     time_diff / 0.04 * ground_height_;
  }

  // Set initial ground height
  // Use estimated terrain
  ref_ground_height_(0) = local_footstep_planner_->getTerrainHeight(current_state_(0), current_state_(1));
  // Use average of the estimated terrain
  // ref_ground_height_(0) = local_footstep_planner_->getTerrainHeight(current_state_.segment(0, 3), current_state_.segment(3, 3));
  // Use flat terrain assumption
  // ref_ground_height_(0) = ground_height_;

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
  double x_mean = 0;
  double y_mean = 0;
  for (int i = 0; i < num_feet_; i++) {
    x_mean += robot_state_msg_->feet.feet[i].position.x/(num_feet_);
    y_mean += robot_state_msg_->feet.feet[i].position.y/(num_feet_);
  }

  ref_body_plan_(0,0) = current_state_[0];//x_mean;
  ref_body_plan_(0,1) = current_state_[1];//y_mean;
  ref_body_plan_(0,2) = z_des_ + ref_ground_height_(0);
  ref_body_plan_(0,3) = 0;
  ref_body_plan_(0,4) = 0;
  ref_body_plan_(0,5) = 1.57;
  ref_body_plan_(0,6) = cmd_vel_[0]*cos(current_state_[5]) - cmd_vel_[1]*sin(current_state_[5]);
  ref_body_plan_(0,7) = cmd_vel_[0]*sin(current_state_[5]) + cmd_vel_[1]*cos(current_state_[5]);
  ref_body_plan_(0,8) = cmd_vel_[2];
  ref_body_plan_(0,9) = cmd_vel_[3];
  ref_body_plan_(0,10) = cmd_vel_[4];
  ref_body_plan_(0,11) = cmd_vel_[5];

  // Only adaptive pitch
  // ref_body_plan_(0, 4) = local_footstep_planner_->getTerrainSlope(current_state_(0), current_state_(1), current_state_(6), current_state_(7));

  // Adaptive roll and pitch
  // local_footstep_planner_->getTerrainSlope(ref_body_plan_(0, 0),
  //                                          ref_body_plan_(0, 1),
  //                                          ref_body_plan_(0, 5),
  //                                          ref_body_plan_(0, 3),
  //                                          ref_body_plan_(0, 4));

  // ref_body_plan_(0, 3) = std::min(std::max(ref_body_plan_(0, 3), -0.2), 0.2);
  // ref_body_plan_(0, 4) = std::min(std::max(ref_body_plan_(0, 4), -0.2), 0.2);

  // Integrate to get full body plan (Forward Euler)
  for (int i = 1; i < N_+1; i++) {
    Twist current_cmd_vel = cmd_vel_;

    double yaw = current_state_[5];
    current_cmd_vel[0] = cmd_vel_[0]*cos(yaw) - cmd_vel_[1]*sin(yaw);
    current_cmd_vel[1] = cmd_vel_[0]*sin(yaw) + cmd_vel_[1]*cos(yaw);

    for (int j = 0; j < 6; j ++) {
      if (i == 1)
      {
        ref_body_plan_(i,j) = ref_body_plan_(i-1,j) + current_cmd_vel[j]*first_element_duration_;
      }
      else
      {
        ref_body_plan_(i,j) = ref_body_plan_(i-1,j) + current_cmd_vel[j]*dt_;
      }
      ref_body_plan_(i,j+6) = (current_cmd_vel[j]);
    }

    // ref_ground_height_(i) = local_footstep_planner_->getTerrainHeight(ref_body_plan_(i, 0), ref_body_plan_(i, 1));
    // ref_body_plan_(i, 2) = z_des_ + ref_ground_height_(i);

    // Only adaptive pitch
    // ref_body_plan_(i, 4) = local_footstep_planner_->getTerrainSlope(ref_body_plan_(i, 0), ref_body_plan_(i, 1), ref_body_plan_(i, 6), ref_body_plan_(i, 7));
    
    // Adaptive roll and pitch
    // local_footstep_planner_->getTerrainSlope(ref_body_plan_(i, 0),
    //                                          ref_body_plan_(i, 1),
    //                                          ref_body_plan_(i, 5),
    //                                          ref_body_plan_(i, 3),
    //                                          ref_body_plan_(i, 4));

    // ref_body_plan_(i, 3) = std::min(std::max(ref_body_plan_(i, 3), -0.2), 0.2);
    // ref_body_plan_(i, 4) = std::min(std::max(ref_body_plan_(i, 4), -0.2), 0.2);

    // Use flat terrain assumption
    // ref_ground_height_(i) = ground_height_;
    // Use average of the estimated terrain
    // ref_ground_height_(i) = local_footstep_planner_->getTerrainHeight(ref_body_plan_.row(i).segment(0, 3), ref_body_plan_.row(i).segment(3, 3));
    // Use estimated terrain
    ref_ground_height_(i) = local_footstep_planner_->getTerrainHeight(ref_body_plan_(i, 0), ref_body_plan_(i, 1));
    ref_body_plan_(i, 2) = z_des_ + ref_ground_height_(i);                          
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
    // Only shift the foot position if it's a solve for a new plan index
    if (!same_plan_index_)
    {
      body_plan_.topRows(N_) = body_plan_.bottomRows(N_);
      grf_plan_.topRows(N_-1) = grf_plan_.bottomRows(N_-1);

      foot_positions_body_.topRows(N_-1) = foot_positions_body_.bottomRows(N_-1);
      foot_positions_world_.topRows(N_-1) = foot_positions_world_.bottomRows(N_-1);
    }
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
  if (grf_plan_.rows() == N_)
  {
    // Swap the last grf if contact schedule changed
    std::vector<bool> contact_schedule_back_prev = contact_schedule_.back();
    local_footstep_planner_->computeContactSchedule(current_plan_index_, current_state_,
                                                    ref_body_plan_, contact_schedule_);
    if (contact_schedule_back_prev != contact_schedule_.back())
    {
      Eigen::MatrixXd trans = Eigen::MatrixXd::Zero(12, 12);
      trans.block(0, 3, 3, 3).diagonal() << 1, 1, 1;
      trans.block(3, 0, 3, 3).diagonal() << 1, 1, 1;
      trans.block(6, 9, 3, 3).diagonal() << 1, 1, 1;
      trans.block(9, 6, 3, 3).diagonal() << 1, 1, 1;

      grf_plan_.row(N_ - 1) = (trans * grf_plan_.row(N_ - 7).transpose()).transpose();
    }
  }
  else
  {
    local_footstep_planner_->computeContactSchedule(current_plan_index_, current_state_,
                                                    ref_body_plan_, contact_schedule_);
  }

  // Start from nominal contact schedule
  adaptive_contact_schedule_ = contact_schedule_;

  // Compute the new footholds if we have a valid existing plan (i.e. if grf_plan is filled)
  if (grf_plan_.rows() == N_) {
    
    local_footstep_planner_->computeFootPositions(body_plan_, grf_plan_,
      contact_schedule_, ref_body_plan_, foot_positions_world_);

    // // For standing test we know the foot position will be constant
    // for (int i = 0; i < N_; i++) {
    //   foot_positions_world_.row(i) = current_foot_positions_world_;
    // }

    // Transform the new foot positions into the body frame for body planning
    local_footstep_planner_->getFootPositionsBodyFrame(body_plan_, foot_positions_world_,
      foot_positions_body_);

    // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    // ROS_INFO_STREAM("same_plan_index_");
    // ROS_INFO_STREAM(same_plan_index_);
    // ROS_INFO_STREAM("first_element_duration_");
    // ROS_INFO_STREAM(first_element_duration_);
    // ROS_INFO_STREAM("foot_positions_world_");
    // ROS_INFO_STREAM(foot_positions_world_.format(CleanFmt));
    // ROS_INFO_STREAM("ref_body_plan_");
    // ROS_INFO_STREAM(ref_body_plan_.format(CleanFmt));
    // ROS_INFO_STREAM("foot_positions_body_");
    // ROS_INFO_STREAM(foot_positions_body_.format(CleanFmt));

    // Contact sensing
    // Foot position refine
    for (size_t i = 0; i < 4; i++)
    {
      if (contact_sensing_.at(i))
      {
        for (size_t j = 0; j < N_; j++)
        {
          if (contact_schedule_.at(j).at(i))
          {
            Eigen::Vector3d hip_pos_plan, nominal_foot_shift;
            quadKD_->worldToNominalHipFKWorldFrame(i, body_plan_.row(j).segment(0, 3), body_plan_.row(j).segment(3, 3), hip_pos_plan);
            nominal_foot_shift << 0, 0, -0.27;

            Eigen::Matrix3d rotation_matrix, body_rotation_matrix;
            double theta = 0.707;
            double yaw = body_plan_(j, 5);
            body_rotation_matrix << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
            body_rotation_matrix = body_rotation_matrix.transpose();
            rotation_matrix << 1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta);
            rotation_matrix = rotation_matrix.transpose();

            foot_positions_world_.row(j).segment(3 * i, 3) = (body_rotation_matrix * rotation_matrix * nominal_foot_shift + hip_pos_plan);

            // foot_positions_body_.block(j, i * 3, 1, 3) = foot_pos_body_miss_contact_.segment(3 * i, 3).transpose();
            adaptive_contact_schedule_.at(j).at(i) = false;
          }
          else
          {
            foot_positions_world_.row(j).segment(3 * i, 3) = foot_positions_world_.row(j - 1).segment(3 * i, 3);
          }

          if (!contact_schedule_.at(j).at(i) && contact_schedule_.at(j + 1).at(i))
          {
            break;
          }
        }
      }
    }

    // local_footstep_planner_->getFootPositionsWorldFrame(body_plan_, foot_positions_body_, foot_positions_world_);
    local_footstep_planner_->getFootPositionsBodyFrame(body_plan_, foot_positions_world_,
                                                       foot_positions_body_);

    // ROS_INFO_STREAM("foot_positions_world_");
    // ROS_INFO_STREAM(foot_positions_world_.format(CleanFmt));
    // ROS_INFO_STREAM("foot_positions_body_");
    // ROS_INFO_STREAM(foot_positions_body_.format(CleanFmt));

    // ROS_INFO_STREAM("contact_schedule_");
    // for (auto i = contact_schedule_.begin(); i != contact_schedule_.end(); ++i)
    // {
    //   for (auto j = i->begin(); j != i->end(); ++j)
    //   {
    //     std::cout << *j << ' ';
    //   }
    //   std::cout << std::endl;
    // }
    // ROS_INFO_STREAM("adaptive_contact_schedule_");
    // for (auto i = adaptive_contact_schedule_.begin(); i != adaptive_contact_schedule_.end(); ++i)
    // {
    //   for (auto j = i->begin(); j != i->end(); ++j)
    //   {
    //     std::cout << *j << ' ';
    //   }
    //   std::cout << std::endl;
    // }
  }

  // Compute grf position considering the toe radius
  Eigen::MatrixXd grf_positions_body = foot_positions_body_;
  for (size_t i = 0; i < 4; i++)
  {
    grf_positions_body.col(3 * i + 2) = foot_positions_body_.col(3 * i + 2).array() - toe_radius;
  }

  // Compute body plan with MPC, return if solve fails
  if (tail_type_ == CENTRALIZED)
  {
    if (!local_body_planner_nonlinear_->computeCentralizedTailPlan(current_state_,
                                                                   ref_body_plan_,
                                                                   grf_positions_body,
                                                                   adaptive_contact_schedule_,
                                                                   tail_current_state_,
                                                                   ref_tail_plan_,
                                                                   ref_ground_height_,
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
                                                       grf_positions_body,
                                                       adaptive_contact_schedule_,
                                                       ref_ground_height_,
                                                       first_element_duration_,
                                                       same_plan_index_,
                                                       body_plan_,
                                                       grf_plan_))
      return false;
  }
  else
  {
    if (!local_body_planner_convex_->computePlan(current_state_,
                                                 ref_body_plan_,
                                                 grf_positions_body,
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

  // if (compute_time_ >= 1000.0/update_rate_) {
  //   ROS_WARN("LocalPlanner took %5.3fms, exceeding %5.3fms allowed",
  //     compute_time_, 1000.0/update_rate_);
  // } else {
    // ROS_INFO("LocalPlanner took %5.3f ms", compute_time_);
  // };

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
  local_footstep_planner_->computeFootPlanMsgs(contact_schedule_, foot_positions_world_, current_foot_positions_world_, current_foot_velocities_world_,
    current_plan_index_, body_plan_, first_element_duration_, past_footholds_msg_, future_footholds_msg, future_nominal_footholds_msg_, foot_plan_msg);

  // Add body, foot, joint, and grf data to the local plan message
  for (int i = 0; i < N_; i++) {

    // Add the state information
    quad_msgs::RobotState robot_state_msg;
    robot_state_msg.body = quad_utils::eigenToBodyStateMsg(body_plan_.row(i));
    robot_state_msg.feet = foot_plan_msg.states[i];
    quad_utils::ikRobotState(*quadKD_, robot_state_msg);

    // Compute nominal foot state
    quad_msgs::RobotState nominal_robot_state_msg;
    nominal_robot_state_msg.body = quad_utils::eigenToBodyStateMsg(body_plan_.row(i));
    nominal_robot_state_msg.feet = future_nominal_footholds_msg_;
    quad_utils::ikRobotState(*quadKD_, nominal_robot_state_msg);
    robot_state_msg.joints_nominal = nominal_robot_state_msg.joints;
    robot_state_msg.feet_nominal = future_nominal_footholds_msg_;

    // Add refernce trajectory information
    quad_msgs::RobotState robot_ref_state_msg;
    robot_ref_state_msg.body = quad_utils::eigenToBodyStateMsg(ref_body_plan_.row(i));

    // Add the GRF information
    quad_msgs::GRFArray grf_array_msg;
    quad_utils::eigenToGRFArrayMsg(grf_plan_.row(i), foot_plan_msg.states[i], grf_array_msg);
    grf_array_msg.contact_states.resize(num_feet_);
    for (int j = 0; j < num_feet_; j++) {
      grf_array_msg.contact_states[j] = contact_schedule_[i][j];
    }

    // Update the headers and plan indices of the messages
    ros::Time state_timestamp;
    // The first duration may vary
    if (i == 0)
    {
      state_timestamp = local_plan_msg.header.stamp;
    }
    else if (i == 1)
    {
      state_timestamp = local_plan_msg.header.stamp + ros::Duration(first_element_duration_);
    }
    else
    {
      state_timestamp = local_plan_msg.header.stamp + ros::Duration(first_element_duration_) + ros::Duration((i - 1) * dt_);
    }
    quad_utils::updateStateHeaders(robot_state_msg, state_timestamp, map_frame_, current_plan_index_+i);
    grf_array_msg.header = robot_state_msg.header;
    grf_array_msg.traj_index = robot_state_msg.traj_index;

    quad_msgs::MultiFootState foot_state_msg_body;
    for (size_t j = 0; j < 4; j++)
    {
      quad_msgs::FootState foot_state_msg;
      quad_utils::eigenToFootStateMsg(foot_positions_body_.block(i, 3*j, 1, 3).transpose(), Eigen::Vector3d::Zero(3), Eigen::Vector3d::Zero(3), foot_state_msg);
      foot_state_msg_body.feet.push_back(foot_state_msg);
    }
    robot_state_msg.feet_body = foot_state_msg_body;

    local_plan_msg.states.push_back(robot_state_msg);
    local_plan_msg.ref_states.push_back(robot_ref_state_msg);
    local_plan_msg.grfs.push_back(grf_array_msg);
    local_plan_msg.plan_indices.push_back(current_plan_index_ + i);
    local_plan_msg.primitive_ids.push_back(2);
    local_plan_msg.ground_height.push_back(ref_ground_height_(i));

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

void LocalPlanner::contactSensing()
{
  if (contact_sensing_msg_ == NULL || !first_solve_success)
  {
    return;
  }

  // Contact sensing
  for (size_t i = 0; i < 4; i++)
  {
    // Swing or contact to miss
    if (contact_schedule_.at(0).at(i) &&
        contact_sensing_msg_->data.at(i) &&
        !contact_sensing_.at(i))
    {
      ROS_WARN_STREAM("Local planner: swing or contact to miss leg: " << i);

      contact_sensing_.at(i) = true;

      // Record foot position in body frame when missing
      Eigen::VectorXd tmp_foot_pos(12);
      quad_utils::multiFootStateMsgToEigen(future_nominal_footholds_msg_, tmp_foot_pos);
      foot_pos_body_miss_contact_.segment(3 * i, 3) = tmp_foot_pos.segment(3 * i, 3) - body_plan_.block(0, 0, 1, 3).transpose();
    }

    // Miss to contact or swing
    if (!contact_sensing_msg_->data.at(i) &&
        contact_sensing_.at(i))
    {
      ROS_WARN_STREAM("Local planner: miss to contact or swing leg: " << i);
      contact_sensing_.at(i) = false;
    }

    // Keep miss
    // Otherwise is keep contact, keep swing, swing to contact, or contact to swing
  }
}

void LocalPlanner::spin() {

  ros::Rate r(update_rate_);

  while (ros::ok()) {

    ros::spinOnce();

    if (!terrain_grid_.exists("z_smooth"))
    {
      publishFootStepHist();
    }

    // Wait until all required data has been received
    if (!(!terrain_grid_.exists("z_smooth") || body_plan_msg_ == NULL && !use_twist_input_ || robot_state_msg_ == NULL))
    {
      contactSensing();

      publishFootStepHist();

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