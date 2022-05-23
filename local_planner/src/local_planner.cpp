#include "local_planner/local_planner.h"

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

LocalPlanner::LocalPlanner(ros::NodeHandle nh)
    : local_body_planner_nonlinear_(), local_footstep_planner_() {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string terrain_map_topic, body_plan_topic, robot_state_topic,
      local_plan_topic, foot_plan_discrete_topic, foot_plan_continuous_topic,
      cmd_vel_topic, control_mode_topic;

  // Load system parameters from launch file (not in config file)
  quad_utils::loadROSParamDefault(nh_, "robot_type", robot_name_,
                                  std::string("spirit"));
  quad_utils::loadROSParam(nh_, "/topics/terrain_map", terrain_map_topic);
  quad_utils::loadROSParam(nh_, "topics/global_plan", body_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/state/ground_truth", robot_state_topic);
  quad_utils::loadROSParam(nh_, "topics/local_plan", local_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/foot_plan_discrete",
                           foot_plan_discrete_topic);
  quad_utils::loadROSParam(nh_, "topics/foot_plan_continuous",
                           foot_plan_continuous_topic);
  quad_utils::loadROSParam(nh_, "topics/cmd_vel", cmd_vel_topic);
  quad_utils::loadROSParam(nh_, "/map_frame", map_frame_);
  quad_utils::loadROSParam(nh_, "topics/control/mode", control_mode_topic);

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(terrain_map_topic, 1,
                                   &LocalPlanner::terrainMapCallback, this);
  body_plan_sub_ =
      nh_.subscribe(body_plan_topic, 1, &LocalPlanner::robotPlanCallback, this);
  robot_state_sub_ =
      nh_.subscribe(robot_state_topic, 1, &LocalPlanner::robotStateCallback,
                    this, ros::TransportHints().tcpNoDelay(true));
  cmd_vel_sub_ =
      nh_.subscribe(cmd_vel_topic, 1, &LocalPlanner::cmdVelCallback, this);

  local_plan_pub_ = nh_.advertise<quad_msgs::RobotPlan>(local_plan_topic, 1);
  foot_plan_discrete_pub_ = nh_.advertise<quad_msgs::MultiFootPlanDiscrete>(
      foot_plan_discrete_topic, 1);
  foot_plan_continuous_pub_ = nh_.advertise<quad_msgs::MultiFootPlanContinuous>(
      foot_plan_continuous_topic, 1);

  // Load system parameters from parameter server
  quad_utils::loadROSParam(nh_, "/local_planner/update_rate", update_rate_);
  quad_utils::loadROSParam(nh_, "/local_planner/timestep", dt_);
  quad_utils::loadROSParam(nh_, "/local_planner/horizon_length", N_);
  quad_utils::loadROSParam(nh_, "/local_planner/desired_height", z_des_);
  quad_utils::loadROSParam(nh_, "/local_planner/toe_radius", toe_radius_);
  quad_utils::loadROSParam(nh_, "/local_planner/cmd_vel_scale", cmd_vel_scale_);
  quad_utils::loadROSParam(nh_, "/local_planner/last_cmd_vel_msg_time_max",
                           last_cmd_vel_msg_time_max_);
  quad_utils::loadROSParam(nh_, "/local_planner/cmd_vel_filter_const",
                           cmd_vel_filter_const_);
  quad_utils::loadROSParam(nh_, "/local_planner/stand_vel_threshold",
                           stand_vel_threshold_);
  quad_utils::loadROSParam(nh_, "/local_planner/stand_cmd_vel_threshold",
                           stand_cmd_vel_threshold_);
  quad_utils::loadROSParam(nh_, "/local_planner/stand_pos_error_threshold",
                           stand_pos_error_threshold_);

  // Load system parameters from launch file (not in config file)
  nh.param<bool>("local_planner/use_twist_input", use_twist_input_, false);

  // Convert kinematics
  quadKD_ = std::make_shared<quad_utils::QuadKD>();

  // Initialize body and foot position arrays (grf_plan horizon is one index
  // shorter since control after last state is not in the horizon)
  ref_body_plan_ = Eigen::MatrixXd::Zero(N_, Nx_);
  foot_positions_world_ = Eigen::MatrixXd::Zero(N_, num_feet_ * 3);
  foot_velocities_world_ = Eigen::MatrixXd::Zero(N_, num_feet_ * 3);
  foot_accelerations_world_ = Eigen::MatrixXd::Zero(N_, num_feet_ * 3);
  foot_positions_body_ = Eigen::MatrixXd::Zero(N_, num_feet_ * 3);
  current_foot_positions_body_ = Eigen::VectorXd::Zero(num_feet_ * 3);
  current_foot_positions_world_ = Eigen::VectorXd::Zero(num_feet_ * 3);
  current_foot_velocities_world_ = Eigen::VectorXd::Zero(num_feet_ * 3);
  ref_primitive_plan_ = Eigen::VectorXi::Zero(N_);
  ref_ground_height_ = Eigen::VectorXd::Zero(N_);
  grf_plan_ = Eigen::MatrixXd::Zero(N_ - 1, 12);
  for (int i = 0; i < num_feet_; i++) {
    grf_plan_.col(3 * i + 2).fill(13.3 * 9.81 / num_feet_);
  }

  // Initialize body and footstep planners
  initLocalBodyPlanner();
  initLocalFootstepPlanner();

  // Initialize twist input variables
  cmd_vel_.resize(6);
  cmd_vel_.setZero();
  initial_timestamp_ = ros::Time::now();
  first_plan_ = true;

  // Initialize stand pose
  stand_pose_.fill(std::numeric_limits<double>::max());
  control_mode_ = STAND;

  // Initialize the time duration to the next plan index
  first_element_duration_ = dt_;

  // Initialize the plan index boolean
  same_plan_index_ = true;

  // Initialize the plan index
  current_plan_index_ = 0;
}

void LocalPlanner::initLocalBodyPlanner() {
  // Create nmpc wrapper class
  SystemID type;
  if (robot_name_ == "spirit") {
    type = SPIRIT;
  } else if (robot_name_ == "a1") {
    type = A1;
  } else {
    ROS_WARN("WRONG ROBOT TYPE");
  }
  local_body_planner_nonlinear_ = std::make_shared<NMPCController>(nh_, type);
}

void LocalPlanner::initLocalFootstepPlanner() {
  // Load parameters from server
  double grf_weight, ground_clearance, hip_clearance, standing_error_threshold,
      period_d, foothold_search_radius, foothold_obj_threshold;
  std::string obj_fun_layer;
  int period;
  std::vector<double> duty_cycles, phase_offsets;
  quad_utils::loadROSParam(nh_, "/local_footstep_planner/grf_weight",
                           grf_weight);
  quad_utils::loadROSParam(nh_, "/local_footstep_planner/ground_clearance",
                           ground_clearance);
  quad_utils::loadROSParam(nh_, "local_footstep_planner/hip_clearance",
                           hip_clearance);
  quad_utils::loadROSParam(nh_,
                           "/local_footstep_planner/standing_error_threshold",
                           standing_error_threshold);
  quad_utils::loadROSParam(nh_, "local_footstep_planner/foothold_search_radius",
                           foothold_search_radius);
  quad_utils::loadROSParam(nh_,
                           "/local_footstep_planner/foothold_obj_threshold",
                           foothold_obj_threshold);
  quad_utils::loadROSParam(nh_, "/local_footstep_planner/obj_fun_layer",
                           obj_fun_layer);
  quad_utils::loadROSParam(nh_, "/local_footstep_planner/period", period_d);
  quad_utils::loadROSParam(nh_, "/local_footstep_planner/duty_cycles",
                           duty_cycles);
  quad_utils::loadROSParam(nh_, "/local_footstep_planner/phase_offsets",
                           phase_offsets);

  period = period_d / dt_;

  // Confirm grf weight is valid
  if (grf_weight > 1 || grf_weight < 0) {
    grf_weight = std::min(std::max(grf_weight, 0.0), 1.0);
    ROS_WARN("Invalid grf weight, clamping to %4.2f", grf_weight);
  }

  // Create footstep class, make sure we use the same dt as the local planner
  local_footstep_planner_ = std::make_shared<LocalFootstepPlanner>();
  local_footstep_planner_->setTemporalParams(dt_, period, N_, duty_cycles,
                                             phase_offsets);
  local_footstep_planner_->setSpatialParams(
      ground_clearance, hip_clearance, standing_error_threshold, grf_weight,
      quadKD_, foothold_search_radius, foothold_obj_threshold, obj_fun_layer,
      toe_radius_);

  past_footholds_msg_.feet.resize(num_feet_);
}

void LocalPlanner::terrainMapCallback(
    const grid_map_msgs::GridMap::ConstPtr &msg) {
  grid_map::GridMapRosConverter::fromMessage(*msg, terrain_grid_);

  // Convert to FastTerrainMap structure for faster querying
  terrain_.loadDataFromGridMap(terrain_grid_);
  local_footstep_planner_->updateMap(terrain_);
  local_footstep_planner_->updateMap(terrain_grid_);
}

void LocalPlanner::robotPlanCallback(
    const quad_msgs::RobotPlan::ConstPtr &msg) {
  body_plan_msg_ = msg;
}

void LocalPlanner::robotStateCallback(
    const quad_msgs::RobotState::ConstPtr &msg) {
  // Make sure the data is actually populated
  if (msg->feet.feet.empty() || msg->joints.position.empty()) return;

  robot_state_msg_ = msg;
}

void LocalPlanner::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  // Ignore non-planar components of desired twist
  cmd_vel_[0] = (1 - cmd_vel_filter_const_) * cmd_vel_[0] +
                cmd_vel_filter_const_ * cmd_vel_scale_ * msg->linear.x;
  cmd_vel_[1] = (1 - cmd_vel_filter_const_) * cmd_vel_[1] +
                cmd_vel_filter_const_ * cmd_vel_scale_ * msg->linear.y;
  cmd_vel_[2] = 0;
  cmd_vel_[3] = 0;
  cmd_vel_[4] = 0;
  cmd_vel_[5] = (1 - cmd_vel_filter_const_) * cmd_vel_[5] +
                cmd_vel_filter_const_ * cmd_vel_scale_ * msg->angular.z;

  // Record when this was last reached for safety
  last_cmd_vel_msg_time_ = ros::Time::now();
}

void LocalPlanner::getReference() {
  if (first_plan_) {
    first_plan_ = false;
    past_footholds_msg_ = robot_state_msg_->feet;
    past_footholds_msg_.traj_index = current_plan_index_;
    for (int i = 0; i < num_feet_; i++) {
      past_footholds_msg_.feet[i].header = past_footholds_msg_.header;
      past_footholds_msg_.feet[i].traj_index = past_footholds_msg_.traj_index;
    }

    // We want to start from a full period when using twist input
    if (use_twist_input_) {
      initial_timestamp_ = ros::Time::now() - ros::Duration(1e-6);
    }
  }

  // Make sure we use the most recent global plan timestamp for reference
  if (!use_twist_input_) {
    initial_timestamp_ = body_plan_msg_->global_plan_timestamp;
  }

  // Tracking trajectory so enter run mode
  control_mode_ = STEP;

  // Get plan index, compare with the previous one to check if this is a
  // duplicated solve
  int previous_plan_index = current_plan_index_;
  quad_utils::getPlanIndex(initial_timestamp_, dt_, current_plan_index_,
                           first_element_duration_);
  same_plan_index_ = previous_plan_index == current_plan_index_;

  // Get the current body and foot positions into Eigen
  current_state_ = quad_utils::bodyStateMsgToEigen(robot_state_msg_->body);
  current_state_timestamp_ = robot_state_msg_->header.stamp;
  quad_utils::multiFootStateMsgToEigen(robot_state_msg_->feet,
                                       current_foot_positions_world_,
                                       current_foot_velocities_world_);
  local_footstep_planner_->getFootPositionsBodyFrame(
      current_state_, current_foot_positions_world_,
      current_foot_positions_body_);

  // Grab the appropriate states from the body plan and convert to an Eigen
  // matrix
  ref_body_plan_.setZero();
  ref_primitive_plan_.setZero();

  if (use_twist_input_) {
    // Use twist planner
    // Check that we have recent twist data, otherwise set cmd_vel to zero
    ros::Duration time_elapsed_since_msg =
        ros::Time::now() - last_cmd_vel_msg_time_;
    if (time_elapsed_since_msg.toSec() > last_cmd_vel_msg_time_max_) {
      cmd_vel_.setZero();
      ROS_WARN_THROTTLE(1.0, "No cmd_vel data, setting twist cmd_vel to zero");
    }

    // Set initial ground height
    ref_ground_height_(0) = local_footstep_planner_->getTerrainHeight(
        current_state_(0), current_state_(1));

    // If it's not initialized, set to current positions
    if (stand_pose_(0) == std::numeric_limits<double>::max() &&
        stand_pose_(1) == std::numeric_limits<double>::max() &&
        stand_pose_(2) == std::numeric_limits<double>::max()) {
      stand_pose_ << current_state_[0], current_state_[1], current_state_[5];
    }

    // Set initial condition for forward integration
    Eigen::Vector2d support_center;
    support_center.setZero();
    for (int i = 0; i < num_feet_; i++) {
      support_center.x() +=
          robot_state_msg_->feet.feet[i].position.x / ((double)num_feet_);
      support_center.y() +=
          robot_state_msg_->feet.feet[i].position.y / ((double)num_feet_);
    }

    // Step if velocity commanded, current velocity exceeds threshold, or too
    // far from center of support
    bool is_stepping =
        (cmd_vel_.norm() > stand_cmd_vel_threshold_ ||
         current_state_.segment(6, 2).norm() > stand_vel_threshold_ ||
         (support_center - current_state_.segment(0, 2)).norm() >
             stand_pos_error_threshold_);

    if (is_stepping) {
      control_mode_ = STEP;
      stand_pose_ << current_state_[0], current_state_[1], current_state_[5];
    } else {
      // If it's standing, try to stablized the waggling
      control_mode_ = STAND;
      Eigen::Vector3d current_stand_pose;
      current_stand_pose << support_center[0], support_center[1],
          current_state_[5];
      stand_pose_ = stand_pose_ * (1 - 1 / update_rate_) +
                    current_stand_pose * 1 / update_rate_;
    }

    ref_body_plan_(0, 0) = stand_pose_[0];  // support_center.x();
    ref_body_plan_(0, 1) = stand_pose_[1];  // support_center.x();
    ref_body_plan_(0, 2) = z_des_ + ref_ground_height_(0);
    ref_body_plan_(0, 3) = 0;
    ref_body_plan_(0, 4) = 0;
    ref_body_plan_(0, 5) = stand_pose_[2];
    ref_body_plan_(0, 6) = cmd_vel_[0] * cos(current_state_[5]) -
                           cmd_vel_[1] * sin(current_state_[5]);
    ref_body_plan_(0, 7) = cmd_vel_[0] * sin(current_state_[5]) +
                           cmd_vel_[1] * cos(current_state_[5]);
    ref_body_plan_(0, 8) = cmd_vel_[2];
    ref_body_plan_(0, 9) = cmd_vel_[3];
    ref_body_plan_(0, 10) = cmd_vel_[4];
    ref_body_plan_(0, 11) = cmd_vel_[5];

    // Alternatively only adaptive pitch
    // ref_body_plan_(0, 4) = local_footstep_planner_->getTerrainSlope(
    //     current_state_(0), current_state_(1), current_state_(6),
    //     current_state_(7));

    // Adaptive roll and pitch
    local_footstep_planner_->getTerrainSlope(
        ref_body_plan_(0, 0), ref_body_plan_(0, 1), ref_body_plan_(0, 5),
        ref_body_plan_(0, 3), ref_body_plan_(0, 4));

    // Integrate to get full body plan (Forward Euler)
    for (int i = 1; i < N_; i++) {
      Eigen::VectorXd current_cmd_vel = cmd_vel_;

      double yaw = ref_body_plan_(i - 1, 5);
      current_cmd_vel[0] = cmd_vel_[0] * cos(yaw) - cmd_vel_[1] * sin(yaw);
      current_cmd_vel[1] = cmd_vel_[0] * sin(yaw) + cmd_vel_[1] * cos(yaw);

      for (int j = 0; j < 6; j++) {
        if (i == 1) {
          ref_body_plan_(i, j) = ref_body_plan_(i - 1, j) +
                                 current_cmd_vel[j] * first_element_duration_;
        } else {
          ref_body_plan_(i, j) =
              ref_body_plan_(i - 1, j) + current_cmd_vel[j] * dt_;
        }
        ref_body_plan_(i, j + 6) = (current_cmd_vel[j]);
      }

      ref_ground_height_(i) = local_footstep_planner_->getTerrainHeight(
          ref_body_plan_(i, 0), ref_body_plan_(i, 1));
      ref_body_plan_(i, 2) = z_des_ + ref_ground_height_(i);

      // Alternatively only adaptive pitch
      // ref_body_plan_(i, 4) = local_footstep_planner_->getTerrainSlope(
      //     ref_body_plan_(i, 0), ref_body_plan_(i, 1), ref_body_plan_(i, 6),
      //     ref_body_plan_(i, 7));

      // Adaptive roll and pitch
      local_footstep_planner_->getTerrainSlope(
          ref_body_plan_(i, 0), ref_body_plan_(i, 1), ref_body_plan_(i, 5),
          ref_body_plan_(i, 3), ref_body_plan_(i, 4));
    }
  } else {
    // Use global plan
    for (int i = 0; i < N_; i++) {
      // If the horizon extends past the reference trajectory, just hold the
      // last state
      if (i + current_plan_index_ > body_plan_msg_->plan_indices.back()) {
        ref_body_plan_.row(i) =
            quad_utils::bodyStateMsgToEigen(body_plan_msg_->states.back().body);
        if (i < N_) {
          ref_primitive_plan_(i) = body_plan_msg_->primitive_ids.back();
        }
      } else {
        ref_body_plan_.row(i) = quad_utils::bodyStateMsgToEigen(
            body_plan_msg_->states[i + current_plan_index_].body);
        if (i < N_) {
          ref_primitive_plan_(i) =
              body_plan_msg_->primitive_ids[i + current_plan_index_];
        }
      }
      ref_ground_height_(i) = local_footstep_planner_->getTerrainHeight(
          ref_body_plan_(i, 0), ref_body_plan_(i, 1));
    }
    ref_ground_height_(0) = local_footstep_planner_->getTerrainHeight(
        current_state_(0), current_state_(1));

    // Stand if the plan has been tracked
    if ((current_state_ - ref_body_plan_.bottomRows(1).transpose()).norm() <=
        stand_pos_error_threshold_) {
      control_mode_ = STAND;
    }
  }

  // Update the body plan to use for foot planning
  int N_current_plan = body_plan_.rows();
  if (N_current_plan < N_) {
    // Cold start with reference plan
    body_plan_.conservativeResize(N_, 12);

    // Initialize with the current foot positions
    for (int i = N_current_plan; i < N_; i++) {
      body_plan_.row(i) = ref_body_plan_.row(i);
      foot_positions_body_.row(i) = current_foot_positions_body_;
      foot_positions_world_.row(i) = current_foot_positions_world_;
    }
  } else {
    // Only shift the foot position if it's a solve for a new plan index
    if (!same_plan_index_) {
      body_plan_.topRows(N_ - 1) = body_plan_.bottomRows(N_ - 1);
      grf_plan_.topRows(N_ - 2) = grf_plan_.bottomRows(N_ - 2);

      foot_positions_body_.topRows(N_ - 1) =
          foot_positions_body_.bottomRows(N_ - 1);
      foot_positions_world_.topRows(N_ - 1) =
          foot_positions_world_.bottomRows(N_ - 1);
    }
  }

  // Initialize with current foot and body positions
  body_plan_.row(0) = current_state_;
  foot_positions_body_.row(0) = current_foot_positions_body_;
  foot_positions_world_.row(0) = current_foot_positions_world_;
}

bool LocalPlanner::computeLocalPlan() {
  if (terrain_.isEmpty() || body_plan_msg_ == NULL && !use_twist_input_ ||
      robot_state_msg_ == NULL) {
    ROS_WARN_STREAM(
        "ComputeLocalPlan function did not recieve the expected inputs");
    return false;
  }

  // Start the timer
  quad_utils::FunctionTimer timer(__FUNCTION__);

  // Compute the contact schedule
  local_footstep_planner_->computeContactSchedule(
      current_plan_index_, ref_primitive_plan_, control_mode_,
      contact_schedule_);

  // Compute the new footholds if we have a valid existing plan (i.e. if
  // grf_plan is filled)
  local_footstep_planner_->computeFootPlan(
      current_plan_index_, contact_schedule_, body_plan_, grf_plan_,
      ref_body_plan_, current_foot_positions_world_,
      current_foot_velocities_world_, first_element_duration_,
      past_footholds_msg_, foot_positions_world_, foot_velocities_world_,
      foot_accelerations_world_);

  // Transform the new foot positions into the body frame for body planning
  local_footstep_planner_->getFootPositionsBodyFrame(
      body_plan_, foot_positions_world_, foot_positions_body_);

  // Compute grf position considering the toe radius
  Eigen::MatrixXd grf_positions_body = foot_positions_body_;
  Eigen::MatrixXd grf_positions_world = foot_positions_world_;
  for (size_t i = 0; i < 4; i++) {
    grf_positions_body.col(3 * i + 2) =
        foot_positions_body_.col(3 * i + 2).array() - toe_radius_;
    grf_positions_world.col(3 * i + 2) =
        foot_positions_world_.col(3 * i + 2).array() - toe_radius_;
  }

  Eigen::VectorXd current_full_state(36), joint_pos(12), joint_vel(12);
  current_full_state.segment(0, 12) = current_state_;
  quad_utils::vectorToEigen(robot_state_msg_->joints.position, joint_pos);
  quad_utils::vectorToEigen(robot_state_msg_->joints.velocity, joint_vel);
  current_full_state.segment(12, 12) = joint_pos;
  current_full_state.segment(24, 12) = joint_vel;

  // Compute leg plan with MPC, return if solve fails
  if (!local_body_planner_nonlinear_->computeLegPlan(
          current_full_state, ref_body_plan_, grf_positions_body,
          grf_positions_world, foot_velocities_world_, contact_schedule_,
          ref_ground_height_, first_element_duration_, same_plan_index_,
          terrain_grid_, body_plan_, grf_plan_))
    return false;

  N_current_ = body_plan_.rows();
  foot_positions_world_ = grf_positions_world;
  for (size_t i = 0; i < 4; i++) {
    foot_positions_world_.col(3 * i + 2) =
        foot_positions_world_.col(3 * i + 2).array() + toe_radius_;
  }

  // Record computation time and update exponential filter
  compute_time_ = 1000.0 * timer.reportSilent();
  mean_compute_time_ = (filter_smoothing_constant_)*mean_compute_time_ +
                       (1 - filter_smoothing_constant_) * compute_time_;
  ROS_INFO_THROTTLE(0.1, "LocalPlanner took %5.3f ms", compute_time_);

  // Return true if made it this far
  return true;
}

void LocalPlanner::publishLocalPlan() {
  // Create messages to publish
  quad_msgs::RobotPlan local_plan_msg;
  quad_msgs::MultiFootPlanDiscrete future_footholds_msg;
  quad_msgs::MultiFootPlanContinuous foot_plan_msg;

  // Update the headers of all messages
  local_plan_msg.header.stamp = current_state_timestamp_;
  local_plan_msg.header.frame_id = map_frame_;
  local_plan_msg.global_plan_timestamp = initial_timestamp_;
  local_plan_msg.compute_time = compute_time_;
  future_footholds_msg.header = local_plan_msg.header;
  foot_plan_msg.header = local_plan_msg.header;

  // Add NLP diagnostic information
  local_body_planner_nonlinear_->getNLPDiagnostics().loadDiagnosticsMsg(
      local_plan_msg.diagnostics);

  // Compute the discrete and continuous foot plan messages
  local_footstep_planner_->loadFootPlanMsgs(
      contact_schedule_, current_plan_index_, first_element_duration_,
      foot_positions_world_, foot_velocities_world_, foot_accelerations_world_,
      future_footholds_msg, foot_plan_msg);

  // Add body, foot, joint, and grf data to the local plan message
  for (int i = 0; i < N_current_ - 1; i++) {
    // Add the state information
    quad_msgs::RobotState robot_state_msg;
    robot_state_msg.body = quad_utils::eigenToBodyStateMsg(body_plan_.row(i));
    robot_state_msg.feet = foot_plan_msg.states[i];
    quad_utils::ikRobotState(*quadKD_, robot_state_msg);

    // Add the GRF information
    quad_msgs::GRFArray grf_array_msg;
    quad_utils::eigenToGRFArrayMsg(grf_plan_.row(i), foot_plan_msg.states[i],
                                   grf_array_msg);
    grf_array_msg.contact_states.resize(num_feet_);
    for (int j = 0; j < num_feet_; j++) {
      grf_array_msg.contact_states[j] = contact_schedule_[i][j];
    }

    // Update the headers and plan indices of the messages
    ros::Time state_timestamp;

    // The first duration will vary
    state_timestamp = (i == 0) ? current_state_timestamp_
                               : current_state_timestamp_ +
                                     ros::Duration(first_element_duration_) +
                                     ros::Duration((i - 1) * dt_);

    quad_utils::updateStateHeaders(robot_state_msg, state_timestamp, map_frame_,
                                   current_plan_index_ + i);
    grf_array_msg.header = robot_state_msg.header;
    grf_array_msg.traj_index = robot_state_msg.traj_index;

    local_plan_msg.states.push_back(robot_state_msg);
    local_plan_msg.grfs.push_back(grf_array_msg);
    local_plan_msg.plan_indices.push_back(current_plan_index_ + i);
    local_plan_msg.primitive_ids.push_back(ref_primitive_plan_(i));
  }

  // Update timestamps to reflect when these messages were published
  local_plan_msg.state_timestamp = current_state_timestamp_;
  auto t_publish = ros::Time::now();
  local_plan_msg.header.stamp = t_publish;
  future_footholds_msg.header.stamp = t_publish;
  foot_plan_msg.header.stamp = t_publish;

  // Publish
  local_plan_pub_.publish(local_plan_msg);
  foot_plan_discrete_pub_.publish(future_footholds_msg);
  foot_plan_continuous_pub_.publish(foot_plan_msg);
}

void LocalPlanner::spin() {
  ros::Rate r(update_rate_);

  while (ros::ok()) {
    ros::spinOnce();

    // Wait until all required data has been received
    if (terrain_.isEmpty() || (body_plan_msg_ == NULL && !use_twist_input_) ||
        robot_state_msg_ == NULL)
      continue;

    // Get the reference plan and robot state into the desired data structures
    getReference();

    // Compute the local plan and publish if it solved successfully, otherwise
    // just sleep
    if (computeLocalPlan()) publishLocalPlan();

    r.sleep();
  }
}
