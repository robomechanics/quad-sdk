#include "local_planner/local_planner.h"

// namespace plt = matplotlibcpp;
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

LocalPlanner::LocalPlanner(ros::NodeHandle nh) :
  local_body_planner_convex_(), local_body_planner_nonlinear_(), local_footstep_planner_() {

	nh_ = nh;
  
    // Load rosparams from parameter server
  std::string terrain_map_topic, body_plan_topic, robot_state_topic, local_plan_topic,
    foot_plan_discrete_topic, foot_plan_continuous_topic, cmd_vel_topic, tail_plan_topic, grf_topic;
  spirit_utils::loadROSParam(nh_, "topics/terrain_map", terrain_map_topic);
  spirit_utils::loadROSParam(nh_, "topics/global_plan", body_plan_topic);
  spirit_utils::loadROSParam(nh_, "topics/state/ground_truth",robot_state_topic);
  spirit_utils::loadROSParam(nh_, "topics/local_plan", local_plan_topic);
  spirit_utils::loadROSParam(nh_, "topics/foot_plan_discrete", foot_plan_discrete_topic);
  spirit_utils::loadROSParam(nh_, "topics/foot_plan_continuous", foot_plan_continuous_topic);
  spirit_utils::loadROSParam(nh_, "topics/cmd_vel", cmd_vel_topic);
  spirit_utils::loadROSParam(nh_, "/topics/control/tail_plan", tail_plan_topic);
  spirit_utils::loadROSParam(nh_, "/topics/state/grfs", grf_topic);
  spirit_utils::loadROSParam(nh_, "map_frame", map_frame_);

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(terrain_map_topic,1, &LocalPlanner::terrainMapCallback, this);
  body_plan_sub_ = nh_.subscribe(body_plan_topic,1, &LocalPlanner::robotPlanCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic,1,&LocalPlanner::robotStateCallback,this);
  cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic,1,&LocalPlanner::cmdVelCallback, this);
  grf_sub_ = nh_.subscribe(grf_topic,1,&LocalPlanner::grfCallback,this);
  local_plan_pub_ = nh_.advertise<spirit_msgs::RobotPlan>(local_plan_topic,1);
  foot_plan_discrete_pub_ = nh_.advertise<
    spirit_msgs::MultiFootPlanDiscrete>(foot_plan_discrete_topic,1);
  foot_plan_continuous_pub_ = nh_.advertise<
    spirit_msgs::MultiFootPlanContinuous>(foot_plan_continuous_topic,1);  
  tail_plan_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>(tail_plan_topic, 1);

  // Load system parameters from parameter server
  spirit_utils::loadROSParam(nh_, "local_planner/update_rate", update_rate_);
  spirit_utils::loadROSParam(nh_, "local_planner/timestep",dt_);
  spirit_utils::loadROSParam(nh_, "local_planner/iterations",iterations_);
  spirit_utils::loadROSParam(nh_, "twist_body_planner/cmd_vel_scale", cmd_vel_scale_);
  spirit_utils::loadROSParam(nh_, "twist_body_planner/last_cmd_vel_msg_time_max",
    last_cmd_vel_msg_time_max_);

  // Load system parameters from launch file (not in config file)
  nh.param<bool>("local_planner/use_nmpc", use_nmpc_, false);
  nh.param<bool>("local_planner/use_twist_input", use_twist_input_, false);
  nh.param<int>("/tail_controller/tail_type", tail_type_, 0);

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

  // Initialize body and foot position arrays
  ref_body_plan_ = Eigen::MatrixXd::Zero(N_+1, Nx_);
  foot_positions_world_ = Eigen::MatrixXd::Zero(N_,num_feet_*3);
  foot_positions_body_ = Eigen::MatrixXd::Zero(N_,num_feet_*3);
  current_foot_positions_body_ = Eigen::VectorXd::Zero(num_feet_*3);
  current_foot_positions_world_ = Eigen::VectorXd::Zero(num_feet_*3);

  tail_plan_ = Eigen::MatrixXd::Zero(N_, 4);
  tail_torque_plan_ = Eigen::MatrixXd::Zero(N_, 2);

  // Initialize body and footstep planners
  initLocalBodyPlanner();
  initLocalFootstepPlanner();

  // Initialize twist input variables
  cmd_vel_.resize(6);
  std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0);
  initial_timestamp_ = ros::Time::now();
  first_plan_ = true;

  // Assume we know the step height
  z_des_ = std::numeric_limits<double>::max();

  miss_contact_leg_.resize(4);

  first_solve_success = false;
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
  double grf_weight, min_ground_clearance, max_ground_clearance, standing_error_threshold, period_d;
  int period;
  spirit_utils::loadROSParam(nh_, "local_footstep_planner/grf_weight", grf_weight);
  spirit_utils::loadROSParam(nh_, "local_footstep_planner/min_ground_clearance", min_ground_clearance);
  spirit_utils::loadROSParam(nh_, "local_footstep_planner/max_ground_clearance", max_ground_clearance);
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
  local_footstep_planner_->setSpatialParams(min_ground_clearance, max_ground_clearance, standing_error_threshold,
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

void LocalPlanner::grfCallback(const spirit_msgs::GRFArray::ConstPtr &msg)
{
  grf_msg_ = msg;
}

void LocalPlanner::getStateAndReferencePlan() {

  // Make sure body plan and robot state data is populated
  if (body_plan_msg_ == NULL || robot_state_msg_ == NULL)
    return;

  // Get index within the global plan
  current_plan_index_ = spirit_utils::getPlanIndex(body_plan_msg_->global_plan_timestamp,dt_);

  // Get the current body and foot positions into Eigen
  current_state_ = spirit_utils::bodyStateMsgToEigen(robot_state_msg_->body);
  current_state_timestamp_ = robot_state_msg_->header.stamp;
  spirit_utils::multiFootStateMsgToEigen(robot_state_msg_->feet, current_foot_positions_world_);
  local_footstep_planner_->getFootPositionsBodyFrame(current_state_, current_foot_positions_world_,
      current_foot_positions_body_);

  // Grab the appropriate states from the body plan and convert to an Eigen matrix
  ref_body_plan_.setZero();
  for (int i = 0; i < N_+1; i++) {

    // If the horizon extends past the reference trajectory, just hold the last state
    if (i+current_plan_index_ > body_plan_msg_->plan_indices.back()) {
      ref_body_plan_.row(i) = spirit_utils::bodyStateMsgToEigen(body_plan_msg_->states.back().body);
    } else {
      ref_body_plan_.row(i) = spirit_utils::bodyStateMsgToEigen(body_plan_msg_->states[i+current_plan_index_].body);
    }
  }

  if (tail_type_ == CENTRALIZED)
  {
    tail_current_state_ = spirit_utils::odomMsgToEigenForTail(*robot_state_msg_);
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

void LocalPlanner::getStateAndTwistInput() {

  if (robot_state_msg_ == NULL || grf_msg_ == NULL)
    return;

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

  if (!first_solve_success)
  {
    initial_timestamp_ = ros::Time::now();
    // ros::Duration tmp(0.03*6);
    // initial_timestamp_ = initial_timestamp_ - tmp;
  }
  
  // Get index
  current_plan_index_ = spirit_utils::getPlanIndex(initial_timestamp_,dt_);

  // Get the current body and foot positions into Eigen
  current_state_ = spirit_utils::bodyStateMsgToEigen(robot_state_msg_->body);
  current_state_timestamp_ = robot_state_msg_->header.stamp;
  spirit_utils::multiFootStateMsgToEigen(robot_state_msg_->feet, current_foot_positions_world_);
  local_footstep_planner_->getFootPositionsBodyFrame(current_state_, current_foot_positions_world_,
      current_foot_positions_body_);

  // Clear any old reference plans
  ref_body_plan_.setZero();

  // Check that we have recent twist data, otherwise set cmd_vel to zero
  // ros::Duration time_elapsed_since_msg = ros::Time::now() - last_cmd_vel_msg_time_;
  // if (time_elapsed_since_msg.toSec() > last_cmd_vel_msg_time_max_) {
  //   std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0);
  //   ROS_WARN_THROTTLE(1.0, "No cmd_vel data, setting twist cmd_vel to zero");
  // }
  std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0);
  cmd_vel_.at(1) = 0.5;

  // Adaptive body height, assume we know step height
  // if (abs(current_foot_positions_world_(2) - current_state_(2)) >= 0.35 ||
  // abs(current_foot_positions_world_(5) - current_state_(2)) >= 0.35 || 
  // abs(current_foot_positions_world_(8) - current_state_(2)) >= 0.35 || 
  // abs(current_foot_positions_world_(11) - current_state_(2)) >= 0.35)
  // {
  //   z_des_ = 0.3;
  // }

  // Adaptive body height, use the lowest contact foot and exponential filter
  std::vector<double> contact_foot_height;
  for (size_t i = 0; i < 4; i++)
  {
    // if (grf_msg_->contact_states.at(i) && grf_msg_->vectors.at(i).z > 10)
    // {
      contact_foot_height.push_back(current_foot_positions_world_(3 * i + 2));
    // }
  }
  if (!contact_foot_height.empty())
  {
    if (z_des_ == std::numeric_limits<double>::max())
    {
      z_des_ = std::max(*std::min_element(contact_foot_height.begin(), contact_foot_height.end()) + 0.3, 0.0);
    }
    else
    {
      z_des_ = 0.75 * std::max(*std::min_element(contact_foot_height.begin(), contact_foot_height.end()) + 0.3, 0.0) + 0.25 * z_des_;
    }
  }

  // Set initial condition for forward integration
  ref_body_plan_(0,0) = current_state_[0];
  ref_body_plan_(0,1) = current_state_[1];
  ref_body_plan_(0,2) = z_des_;
  ref_body_plan_(0,3) = 0;
  ref_body_plan_(0,4) = 0;
  ref_body_plan_(0,5) = current_state_[5];
  ref_body_plan_(0,6) = cmd_vel_[0]*cos(current_state_[5]) - cmd_vel_[1]*sin(current_state_[5]);
  ref_body_plan_(0,7) = cmd_vel_[0]*sin(current_state_[5]) + cmd_vel_[1]*cos(current_state_[5]);
  ref_body_plan_(0,8) = cmd_vel_[2];
  ref_body_plan_(0,9) = cmd_vel_[3];
  ref_body_plan_(0,10) = cmd_vel_[4];
  ref_body_plan_(0,11) = cmd_vel_[5];

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
  }

  if (tail_type_ == CENTRALIZED)
  {
    tail_current_state_ = spirit_utils::odomMsgToEigenForTail(*robot_state_msg_);
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

  if (terrain_.isEmpty() || body_plan_msg_ == NULL && !use_twist_input_ || robot_state_msg_ == NULL || grf_msg_ == NULL)
    return false;  

  // Start the timer
  spirit_utils::FunctionTimer timer(__FUNCTION__);

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

  // Contact sensing
  adpative_contact_schedule_ = contact_schedule_;

  for (size_t i = 0; i < 4; i++)
  {
    // Check foot distance
    Eigen::VectorXd foot_position_vec = current_foot_positions_body_.segment(3 * i, 3);

    ROS_WARN_STREAM("dis: " << foot_position_vec.norm());

    if (foot_position_vec.norm() > 0.45)
    {
      ROS_WARN_STREAM("miss!");
      miss_contact_leg_.at(i) = true;
    }

    if (contact_schedule_.at(0).at(i) && bool(grf_msg_->contact_states.at(i)) && miss_contact_leg_.at(i))
    {
      miss_contact_leg_.at(i) = false;
    }

    if (miss_contact_leg_.at(i))
    {
      // We assume it will not touch the ground at this gait peroid
      for (size_t j = 0; j < N_; j++)
      {
        if (contact_schedule_.at(j).at(i))
        {
          adpative_contact_schedule_.at(j).at(i) = false;
          if (j > 0)
          {
            foot_positions_world_(j, i * 3 + 0) = foot_positions_world_(j, i * 3 + 0) - 0.05;
            foot_positions_body_(j, i * 3 + 0) = foot_positions_body_(j, i * 3 + 0) - 0.05;
            ROS_WARN_STREAM("shift: " << i << "leg");
          }
        }
        else
        {
          break;
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
      if (!local_body_planner_nonlinear_->computeLegPlan(current_state_, ref_body_plan_, foot_positions_body_,
                                                         adpative_contact_schedule_, body_plan_, grf_plan_))
        return false;
    }
    else
    {
      if (!local_body_planner_convex_->computePlan(current_state_, ref_body_plan_, foot_positions_body_,
                                                   adpative_contact_schedule_, body_plan_, grf_plan_))
        return false;
    }

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

  // if (current_plan_index_ >= 50) {
  // std::cout << "current_state_\n" << current_state_ << std::endl;
  // std::cout << "ref_body_plan_\n" << ref_body_plan_ << std::endl;
  // std::cout << "body_plan_\n" << body_plan_ << std::endl;
  // std::cout << "grf_plan_\n" << grf_plan_ << std::endl;
  // std::cout << "foot_positions_world_\n" << foot_positions_world_ << std::endl;
  // std::cout << "foot_positions_body_\n" << foot_positions_body_ << std::endl;
  // throw std::runtime_error("Stop");
  // }

  // Create messages to publish
  spirit_msgs::RobotPlan local_plan_msg;
  spirit_msgs::MultiFootPlanDiscrete future_footholds_msg;
  spirit_msgs::MultiFootPlanContinuous foot_plan_msg;
  spirit_msgs::LegCommandArray tail_plan_msg;

  // Update the headers of all messages
  ros::Time timestamp = entrance_time_;
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
      tail_plan_msg.dt_first_step = dt_;
  }

  // Compute the discrete and continuous foot plan messages
  local_footstep_planner_->computeFootPlanMsgs(contact_schedule_, foot_positions_world_,
    current_plan_index_, past_footholds_msg_, future_footholds_msg, foot_plan_msg);

  // for (size_t i = 0; i < 4; i++)
  // {
  //   if (miss_contact_leg_.at(i))
  //   {
  //     // We assume it will not touch the ground at this gait peroid
  //     for (size_t j = 0; j < N_; j++)
  //     {
  //       if (contact_schedule_.at(j).at(i))
  //       {
  //         grf_plan_(j, i * 3 + 0) = 0;
  //         grf_plan_(j, i * 3 + 1) = 0;
  //         grf_plan_(j, i * 3 + 2) = 0;
  //       }
  //       else
  //       {
  //         break;
  //       }
  //     }
  //   }
  // }

  // Add body, foot, joint, and grf data to the local plan message
  for (int i = 0; i < N_; i++) {

    // for (int j = 0; j < num_feet_; j++)
    // {
    //   // Potential leg control for later contact
    //   if (contact_schedule_[i][j] && !adpative_contact_schedule_[i][j])
    //   {
    //     Eigen::VectorXd body_foot_positions = foot_positions_body_.block(i, 3 * j, 1, 3).transpose();

    //     foot_plan_msg.states[i].feet[j].position.x += 0.1*body_plan_(0, 6);
    //     foot_plan_msg.states[i].feet[j].position.y += 0.1*body_plan_(0, 7);
    //     foot_plan_msg.states[i].feet[j].position.z = body_plan_(i, 2) - 0.35;
    //   }

    //   // Potential leg control for early contact
    //   if (!contact_schedule_[i][j] && adpative_contact_schedule_[i][j])
    //   {
    //     foot_plan_msg.states[i].feet[j].contact = true;
    //   }
    // }

    // Add the state information
    spirit_msgs::RobotState robot_state_msg;
    robot_state_msg.body = spirit_utils::eigenToBodyStateMsg(body_plan_.row(i));
    robot_state_msg.feet = foot_plan_msg.states[i];
    spirit_utils::ikRobotState(*kinematics_, robot_state_msg);

    // Add the GRF information
    spirit_msgs::GRFArray grf_array_msg;
    grf_array_msg.contact_states.resize(num_feet_);
    for (int j = 0; j < num_feet_; j++)
    {
      grf_array_msg.contact_states[j] = contact_schedule_[i][j];

      // // Potential leg control for later contact
      // if (contact_schedule_[i][j] && !adpative_contact_schedule_[i][j])
      // {
      //   Eigen::VectorXd hip_foot_positions = (hip_projected_foot_positions_.block(i, 3 * j, 1, 3) - foot_positions_body_.block(i, 3 * j, 1, 3)).transpose();

      //   grf_plan_(i, 3 * j + 0) = hip_foot_positions(0) / 0.3 * 58;
      //   grf_plan_(i, 3 * j + 1) = hip_foot_positions(1) / 0.3 * 58;
      //   grf_plan_(i, 3 * j + 2) = 58;

      //   grf_plan_(i, 3 * j + 0) = 0;
      //   grf_plan_(i, 3 * j + 1) = 0;
      //   grf_plan_(i, 3 * j + 2) = 0;
      // }
    }
    spirit_utils::eigenToGRFArrayMsg(grf_plan_.row(i), foot_plan_msg.states[i], grf_array_msg);

    // Update the headers and plan indices of the messages
    ros::Time state_timestamp = local_plan_msg.header.stamp + ros::Duration(i*dt_);
    spirit_utils::updateStateHeaders(robot_state_msg, state_timestamp, map_frame_, current_plan_index_+i);
    grf_array_msg.header = robot_state_msg.header;
    grf_array_msg.traj_index = robot_state_msg.traj_index;

    local_plan_msg.states.push_back(robot_state_msg);
    local_plan_msg.grfs.push_back(grf_array_msg);
    local_plan_msg.plan_indices.push_back(current_plan_index_ + i);
    local_plan_msg.primitive_ids.push_back(2);

    if (tail_type_ == CENTRALIZED)
    {
      spirit_msgs::LegCommand tail_msg;
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

  // std::cout << foot_plan_msg << std::endl;
}

void LocalPlanner::spin() {

  ros::Rate r(update_rate_);

  while (ros::ok()) {

    ros::spinOnce();

    entrance_time_ = ros::Time::now();

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