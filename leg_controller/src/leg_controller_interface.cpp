#include "leg_controller/leg_controller_interface.h"

LegControllerInterface::LegControllerInterface(ros::NodeHandle nh, int argc, char** argv) {
	nh_ = nh;

    // Load rosparams from parameter server
  std::string grf_topic, trajectory_state_topic, robot_state_topic, local_plan_topic,
    leg_command_array_topic, control_mode_topic, leg_override_topic,
    remote_heartbeat_topic, robot_heartbeat_topic, single_joint_cmd_topic, mocap_topic;
  quad_utils::loadROSParam(nh_,"topics/local_plan",local_plan_topic);
  quad_utils::loadROSParam(nh_,"topics/state/ground_truth",robot_state_topic);
  quad_utils::loadROSParam(nh_,"topics/state/trajectory",trajectory_state_topic);
  quad_utils::loadROSParam(nh_,"topics/heartbeat/remote",remote_heartbeat_topic);
  quad_utils::loadROSParam(nh_,"topics/heartbeat/robot",robot_heartbeat_topic);
  quad_utils::loadROSParam(nh_,"topics/control/grfs",grf_topic);
  quad_utils::loadROSParam(nh_,"topics/control/joint_command",leg_command_array_topic);
  quad_utils::loadROSParam(nh_,"topics/control/leg_override",leg_override_topic);
  quad_utils::loadROSParam(nh_,"topics/control/mode",control_mode_topic);
  quad_utils::loadROSParam(nh_,"topics/control/single_joint_command",single_joint_cmd_topic);
  quad_utils::loadROSParam(nh_,"topics/mocap",mocap_topic);

  quad_utils::loadROSParam(nh_,"leg_controller/controller", controller_id_);
  quad_utils::loadROSParam(nh_,"leg_controller/update_rate", update_rate_);
  quad_utils::loadROSParam(nh_,"leg_controller/publish_rate", publish_rate_);
  quad_utils::loadROSParam(nh_,"leg_controller/mocap_rate", mocap_rate_);
  quad_utils::loadROSParam(nh_,"leg_controller/mocap_dropout_threshold", mocap_dropout_threshold_);
  quad_utils::loadROSParam(nh_,"leg_controller/filter_time_constant", filter_time_constant_);
  quad_utils::loadROSParam(nh_,"leg_controller/publish_rate", publish_rate_);
  quad_utils::loadROSParam(nh_,"leg_controller/input_timeout", input_timeout_);
  quad_utils::loadROSParam(nh_,"leg_controller/state_timeout", state_timeout_);
  quad_utils::loadROSParam(nh_,"leg_controller/heartbeat_timeout", heartbeat_timeout_);
  quad_utils::loadROSParam(nh_, "leg_controller/sit_kp", sit_kp_);
  quad_utils::loadROSParam(nh_, "leg_controller/sit_kd", sit_kd_);
  quad_utils::loadROSParam(nh_, "leg_controller/stand_kp", stand_kp_);
  quad_utils::loadROSParam(nh_, "leg_controller/stand_kd", stand_kd_);
  quad_utils::loadROSParam(nh_, "leg_controller/stance_kp", stance_kp_);
  quad_utils::loadROSParam(nh_, "leg_controller/stance_kd", stance_kd_);
  quad_utils::loadROSParam(nh_, "leg_controller/swing_kp", swing_kp_);
  quad_utils::loadROSParam(nh_, "leg_controller/swing_kd", swing_kd_);
  quad_utils::loadROSParam(nh_, "leg_controller/safety_kp", safety_kp_);
  quad_utils::loadROSParam(nh_, "leg_controller/safety_kd", safety_kd_);
  quad_utils::loadROSParam(nh_, "leg_controller/stand_joint_angles", stand_joint_angles_);
  quad_utils::loadROSParam(nh_, "leg_controller/sit_joint_angles", sit_joint_angles_);
  quad_utils::loadROSParam(nh_, "leg_controller/remote_latency_threshold_warn",
    remote_latency_threshold_warn_);
  quad_utils::loadROSParam(nh_, "leg_controller/remote_latency_threshold_error",
    remote_latency_threshold_error_);

  quad_utils::loadROSParam(nh_,"local_planner/timestep", dt_);

  // Setup pubs and subs
  local_plan_sub_ = nh_.subscribe(local_plan_topic,1,&LegControllerInterface::localPlanCallback, this, ros::TransportHints().tcpNoDelay(true));
  trajectory_state_sub_ = nh_.subscribe(
    trajectory_state_topic,1,&LegControllerInterface::trajectoryStateCallback, this);
  control_mode_sub_ = nh_.subscribe(
    control_mode_topic,1,&LegControllerInterface::controlModeCallback, this);
  single_joint_cmd_sub_ = nh_.subscribe(
    single_joint_cmd_topic,1,&LegControllerInterface::singleJointCommandCallback, this);
  leg_override_sub_ = nh_.subscribe(
    leg_override_topic,1,&LegControllerInterface::legOverrideCallback, this);
  remote_heartbeat_sub_ = nh_.subscribe(
    remote_heartbeat_topic,1,&LegControllerInterface::remoteHeartbeatCallback, this);
  mocap_sub_ = nh_.subscribe(mocap_topic,1000,&LegControllerInterface::mocapCallback, this, ros::TransportHints().tcpNoDelay(true));
  grf_pub_ = nh_.advertise<quad_msgs::GRFArray>(grf_topic,1);
  leg_command_array_pub_ = nh_.advertise<quad_msgs::LegCommandArray>(leg_command_array_topic,1);
  robot_state_pub_= nh_.advertise<quad_msgs::RobotState>(robot_state_topic,1);
  robot_heartbeat_pub_ = nh_.advertise<std_msgs::Header>(robot_heartbeat_topic,1);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/state/imu",1);

  // Start sitting
  control_mode_ = SIT;
  remote_heartbeat_received_time_ = std::numeric_limits<double>::max();
  last_state_time_ = std::numeric_limits<double>::max();

  // Initialize kinematics object
  quadKD_ = std::make_shared<quad_utils::QuadKD>();

  // Initialize mblink converter
  mblink_converter_ = std::make_shared<MBLinkConverter>(nh_, argc, argv);

  // Initialize leg controller object
  if (controller_id_ == "inverse_dynamics") {
    leg_controller_ = std::make_shared<InverseDynamicsController>();
  } else if (controller_id_ == "grf_pid") {
    leg_controller_ = std::make_shared<GrfPidController>();
  } else if (controller_id_ == "joint") {
    leg_controller_ = std::make_shared<JointController>();
  } else {
    ROS_ERROR_STREAM("Invalid controller id " << controller_id_ << ", returning nullptr");
    leg_controller_ = nullptr;
  }
  leg_controller_->setGains(stance_kp_, stance_kd_, swing_kp_, swing_kd_);

  grf_array_msg_.vectors.resize(num_feet_);
  grf_array_msg_.points.resize(num_feet_);
  grf_array_msg_.contact_states.resize(num_feet_);
  grf_array_msg_.header.frame_id = "map";
  grf_array_msg_.header.stamp = ros::Time::now();
  for (int i = 0; i < num_feet_; i++) {
    geometry_msgs::Vector3 vec;
    geometry_msgs::Point point;
    bool contact_state = true;
    grf_array_msg_.vectors[i] = vec;
    grf_array_msg_.points[i] = point;
    grf_array_msg_.contact_states[i] = contact_state;
  }

  // Set joint torque limits
  torque_limits_ << 21, 21, 32;

  // Not allow to trot at first
  double gait_period;
  quad_utils::loadROSParam(nh_, "/local_footstep_planner/period", gait_period);
  trotting_duration_ = gait_period * 2 * update_rate_;
  trotting_count_ = trotting_duration_++;

  // Initialize timing
  last_mainboard_time_ = 0;
  last_robot_state_msg_.header.stamp = ros::Time::now();

  // Assume zero initial velocity
  mocap_vel_estimate_.setZero();
  imu_vel_estimate_.setZero();
}

void LegControllerInterface::controlModeCallback(const std_msgs::UInt8::ConstPtr& msg) {
  
  // Wait if transitioning
  if ((control_mode_ == SIT_TO_READY) || (control_mode_ == READY_TO_SIT))
    return;

  if ((msg->data == READY) && (control_mode_ == SIT)) { // Stand if previously sitting

    control_mode_ = SIT_TO_READY;
    transition_timestamp_ = ros::Time::now();

  } else if ((msg->data == SIT) && (control_mode_ == READY)) { // Sit if previously standing

    control_mode_ = READY_TO_SIT;
    transition_timestamp_ = ros::Time::now();

  } else if (msg->data == SIT || (msg->data == SAFETY)) { // Allow sit or safety modes
    
    control_mode_ = msg->data;
  }
}

void LegControllerInterface::singleJointCommandCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
  if (JointController* c = dynamic_cast<JointController*>(leg_controller_.get())) {
    c->updateSingleJointCommand(msg);
  }
}

void LegControllerInterface::localPlanCallback(const quad_msgs::RobotPlan::ConstPtr& msg) {
  last_local_plan_msg_ = msg;

  ros::Time t_now = ros::Time::now();
  double round_trip_time_diff = (t_now - last_local_plan_msg_->state_timestamp).toSec();
  ROS_INFO_STREAM_THROTTLE(0.5,"round trip time difference: " << round_trip_time_diff);

  double local_plan_time_diff = (t_now - last_local_plan_msg_->header.stamp).toSec();
  ROS_INFO_STREAM_THROTTLE(0.5,"local plan time difference: " << local_plan_time_diff);

  leg_controller_->updateLocalPlanMsg(last_local_plan_msg_, t_now);
}

void LegControllerInterface::mocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

  if (last_mocap_msg_ != NULL)
  {
    // Collect change in position for velocity update
    Eigen::Vector3d pos_new, pos_old;
    quad_utils::pointMsgToEigen(msg->pose.position, pos_new);
    quad_utils::pointMsgToEigen(last_mocap_msg_->pose.position, pos_old);

    // Record time diff between messages
    double t_diff_mocap_msg = (msg->header.stamp - last_mocap_msg_->header.stamp).toSec();
    // std::cout << "t_diff_mocap_msg = " << t_diff_mocap_msg << std::endl;
    ros::Time t_now = ros::Time::now();
    // std::cout << "t_diff_ros = " << (t_now - last_mocap_time_).toSec() << std::endl;
    last_mocap_time_ = t_now;
    double t_mocap_ros_latency = (ros::Time::now() - msg->header.stamp).toSec();
    // std::cout << "t_mocap_ros_latency = " << t_mocap_ros_latency << std::endl;

    // Use new measurement
    if (abs(t_diff_mocap_msg - 1.0/mocap_rate_) < mocap_dropout_threshold_) {
      
      // Declare vectors for vel measurement and estimate
      Eigen::Vector3d vel_new_measured, vel_new_est;
      vel_new_measured = (pos_new - pos_old)*mocap_rate_;

      // Filtered velocity estimate assuming motion capture frame rate is constant at mocap_rate_
      // in order to avoid variable network and ROS latency that appears in the message time stamp
      mocap_vel_estimate_ = (1-1/mocap_rate_/filter_time_constant_)*mocap_vel_estimate_ + (1/mocap_rate_/filter_time_constant_)*vel_new_measured;
    } else {
      ROS_WARN_THROTTLE(0.1,"Mocap time diff exceeds max dropout threshold, hold the last value");
      // mocap_vel_estimate_ = (pos_new - pos_old)/t_diff_mocap_msg;
    }
  }

  // Update our cached mocap position
  last_mocap_msg_ = msg;
  
}
// void LegControllerInterface::robotStateCallback(const quad_msgs::RobotState::ConstPtr& msg) {
  
//   if (last_robot_state_msg_ != NULL) {
//     first_robot_state_msg_ = msg;
//   }

//   last_robot_state_msg_ = msg;
// }

void LegControllerInterface::trajectoryStateCallback(const quad_msgs::RobotState::ConstPtr& msg) {
  last_trajectory_state_msg_ = msg;
}

void LegControllerInterface::legOverrideCallback(const quad_msgs::LegOverride::ConstPtr& msg) {
  last_leg_override_msg_ = *msg;
}

void LegControllerInterface::remoteHeartbeatCallback(const std_msgs::Header::ConstPtr& msg) {

  // Get the current time and compare to the message time
  double remote_heartbeat_sent_time = msg->stamp.toSec();
  remote_heartbeat_received_time_ = ros::Time::now().toSec();
  double t_latency = remote_heartbeat_received_time_ - remote_heartbeat_sent_time;

  ROS_INFO_THROTTLE(1.0,"Remote latency = %6.4fs", t_latency);

  if (abs(t_latency) >= remote_latency_threshold_warn_) {
    ROS_WARN_THROTTLE(1.0,"Remote latency = %6.4fs which exceeds the warning threshold of %6.4fs\n",
      t_latency, remote_latency_threshold_warn_);
  }

  if (abs(t_latency) >= remote_latency_threshold_error_) {
    ROS_WARN_THROTTLE(1.0,"Remote latency = %6.4fs which exceeds the maximum threshold of %6.4fs, "
      "entering safety mode\n", t_latency, remote_latency_threshold_error_);
    // control_mode_ = SAFETY;
  }
}

void LegControllerInterface::checkMessages() {

  // Do nothing if already in safety mode
  if (control_mode_ == SAFETY)
    return;

  // Check the remote heartbeat for timeout
  // (this adds extra safety if no heartbeat messages are arriving)
  if (abs(ros::Time::now().toSec() - remote_heartbeat_received_time_) >= heartbeat_timeout_ && 
    remote_heartbeat_received_time_ != std::numeric_limits<double>::max())
  {
    control_mode_ = SAFETY;
    ROS_WARN_THROTTLE(1,"Remote heartbeat lost or late to ID node, entering safety mode");
  }

  // // Check the state message latency
  // if (abs(ros::Time::now().toSec() - last_state_time_) >= state_timeout_ && 
  //   last_state_time_ != std::numeric_limits<double>::max())
  // {
  //   control_mode_ = SAFETY;
  //   transition_timestamp_ = ros::Time::now();
  //   ROS_WARN_THROTTLE(1,"State messages lost in ID node, entering safety mode");
  // }

}

void LegControllerInterface::executeCustomController() {

  quad_msgs::RobotState::ConstPtr last_robot_state_msg_ptr(new quad_msgs::RobotState(last_robot_state_msg_));
  if (leg_controller_->computeLegCommandArray(last_robot_state_msg_ptr,
      leg_command_array_msg_, grf_array_msg_) == false || trotting_count_ >= trotting_duration_) {

    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
          stand_joint_angles_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kp = stand_kp_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kd = stand_kd_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
      }
    }
  }
  else
  {
    // trotting_count_++;
  }
  
}

bool LegControllerInterface::computeLegCommandArray() {

  // Check if state machine should be skipped
  bool valid_cmd = true;
  if (leg_controller_->overrideStateMachine()) {
    quad_msgs::RobotState::ConstPtr last_robot_state_msg_ptr(new quad_msgs::RobotState(last_robot_state_msg_));

    valid_cmd = leg_controller_->computeLegCommandArray(last_robot_state_msg_ptr,
      leg_command_array_msg_, grf_array_msg_);
    return valid_cmd;
}

  // Check incoming messages to determine if we should enter safety mode
  checkMessages();

  // Define vectors for joint positions and velocities
  Eigen::VectorXd joint_positions(3*num_feet_), joint_velocities(3*num_feet_), body_state(12);
  quad_utils::vectorToEigen(last_robot_state_msg_.joints.position, joint_positions);
  quad_utils::vectorToEigen(last_robot_state_msg_.joints.velocity, joint_velocities);

  // std::cout << "joint_positions\n" << joint_positions << std::endl;

  // Initialize leg command message
  leg_command_array_msg_.leg_commands.resize(num_feet_);

  // Enter state machine for filling motor command message
  if (control_mode_ == SAFETY)
  { 
    for (int i = 0; i < num_feet_; ++i) {

      leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {

        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 0.0;
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0.0;
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kp = safety_kp_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kd = safety_kd_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0.0;
      }
    }
  } else if (control_mode_ == SIT)
  { 
    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {

        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
          sit_joint_angles_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kp = sit_kp_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kd = sit_kd_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
      }
    }
  } 
  else if (control_mode_ == READY)
  {
    executeCustomController();
    
    int n_override = last_leg_override_msg_.leg_index.size();
    int leg_ind;
    quad_msgs::MotorCommand motor_command;
    if (n_override > 0){
      for (int i = 0; i < n_override; i++) {
        leg_ind = last_leg_override_msg_.leg_index.at(i);
        for (int j = 0; j < 3; j++) {
          motor_command = last_leg_override_msg_.leg_commands.at(i).motor_commands.at(j);
          leg_command_array_msg_.leg_commands.at(leg_ind).motor_commands.at(j).pos_setpoint = 
            motor_command.pos_setpoint;
          leg_command_array_msg_.leg_commands.at(leg_ind).motor_commands.at(j).vel_setpoint = 
            motor_command.vel_setpoint;
          leg_command_array_msg_.leg_commands.at(leg_ind).motor_commands.at(j).kp = 
            motor_command.kp;
          leg_command_array_msg_.leg_commands.at(leg_ind).motor_commands.at(j).kd = 
            motor_command.kd;
          leg_command_array_msg_.leg_commands.at(leg_ind).motor_commands.at(j).torque_ff = 
            motor_command.torque_ff;
        }
      }
    }
  }
  else if (control_mode_ == SIT_TO_READY)
  {
    ros::Duration duration = ros::Time::now() - transition_timestamp_;
    double t_interp = duration.toSec()/transition_duration_;

    if (t_interp >= 1) {
      control_mode_ = READY;
      trotting_count_ = 0;
      return valid_cmd;
    }

    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        double ang = stand_joint_angles_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
          (ang - sit_joint_angles_.at(j))*t_interp + 
          sit_joint_angles_.at(j);
        
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kp = stand_kp_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kd = stand_kd_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
      }
    }
  }
  else if (control_mode_ == READY_TO_SIT)
  {
    ros::Duration duration = ros::Time::now() - transition_timestamp_;
    double t_interp = duration.toSec()/transition_duration_;

    if (t_interp >= 1) {
      control_mode_ = SIT;
      return valid_cmd;
    }

    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
          (sit_joint_angles_.at(j) - stand_joint_angles_.at(j))*t_interp + 
          stand_joint_angles_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kp = stand_kp_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kd = stand_kd_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
      }
    }
  } else {
    ROS_WARN_THROTTLE(0.5, "Invalid control mode set in ID node, "
      "exiting publishLegCommandArray()");
      return false;
  }

  for (int i = 0; i < num_feet_; ++i) {
    for (int j = 0; j < 3; ++j) {
      int joint_idx = 3*i+j;
      quad_msgs::MotorCommand cmd = leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j);
      double pos_component = cmd.kp*(cmd.pos_setpoint - joint_positions[joint_idx]);
      double vel_component = cmd.kd*(cmd.vel_setpoint - joint_velocities[joint_idx]);
      double fb_component = pos_component + vel_component;
      double effort = fb_component + cmd.torque_ff;
      double fb_ratio = abs(fb_component)/(abs(fb_component) + abs(cmd.torque_ff));

      if (abs(cmd.torque_ff) >= torque_limits_[j]) {
        ROS_WARN("Leg %d motor %d: ff effort = %5.3f Nm exceeds threshold of %5.3f Nm", i,j,cmd.torque_ff, torque_limits_[j]);

      }      
      if (abs(effort) >= torque_limits_[j]) {
        ROS_WARN("Leg %d motor %d: total effort = %5.3f Nm exceeds threshold of %5.3f Nm", i,j,effort, torque_limits_[j]);
        effort = std::min(std::max(effort, -torque_limits_[j]), torque_limits_[j]);
      }

      leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).pos_component = pos_component;
      leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).vel_component = vel_component;
      leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).fb_component = fb_component;
      leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).effort = effort;
      leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).fb_ratio = fb_ratio;
    }
  }

  return valid_cmd;
}

bool LegControllerInterface::updateState() {

  // Get the newest data from the mainboard (BLOCKING)
  ros::Time t_mb0 = ros::Time::now();
  mblink_converter_->getMBlink(mbdata_);

  if (mbdata_.empty()) {
    ROS_WARN_THROTTLE(1,"No data recieved from mblink");
    return false;
  } else {
  }

  // Record the timestamp of this data
  ros::Time state_timestamp = ros::Time::now();

  double t_diff_mb = mbdata_["y"][20] - last_mainboard_time_;
  double t_diff_ros = (state_timestamp - last_robot_state_msg_.header.stamp).toSec();
  double t_diff_mb_get = (state_timestamp - t_mb0).toSec();
  ROS_INFO_THROTTLE(0.1,"t_diff_mb = %8.5fs, t_diff_ros = %8.5fs, t_diff_mb_get = %8.5fs",
    t_diff_mb, t_diff_ros, t_diff_mb_get);
  last_mainboard_time_ = mbdata_["y"][20];

  // Declare the joint state msg and apply the timestamp
  last_joint_state_msg_.header.stamp = state_timestamp;

  // Add the data corresponding to each joint
  for (int i = 0; i < joint_names_.size(); i++)
  {
    last_joint_state_msg_.name.push_back(joint_names_[i]);
    last_joint_state_msg_.position.push_back(mbdata_["joint_position"][joint_indices_[i]]);
    last_joint_state_msg_.velocity.push_back(mbdata_["joint_velocity"][joint_indices_[i]]);

    // Convert from current to torque
    last_joint_state_msg_.effort.push_back(kt_vec_[i]*mbdata_["joint_current"][joint_indices_[i]]);
  }

  // Declare the imu message
  last_imu_msg_.header.stamp = state_timestamp;

  // Transform from rpy to quaternion
  geometry_msgs::Quaternion orientation_msg;
  tf2::Quaternion quat_tf;
  quat_tf.setRPY(mbdata_["imu_euler"][0],mbdata_["imu_euler"][1],mbdata_["imu_euler"][2]);
  tf2::convert(quat_tf, orientation_msg);

  // Load the data into the imu message
  last_imu_msg_.orientation = orientation_msg;
  last_imu_msg_.angular_velocity.x = mbdata_["imu_angular_velocity"][0];
  last_imu_msg_.angular_velocity.y = mbdata_["imu_angular_velocity"][1];
  last_imu_msg_.angular_velocity.z = mbdata_["imu_angular_velocity"][2];
  last_imu_msg_.linear_acceleration.x = mbdata_["imu_linear_acceleration"][0];
  last_imu_msg_.linear_acceleration.y = mbdata_["imu_linear_acceleration"][1];
  last_imu_msg_.linear_acceleration.z = mbdata_["imu_linear_acceleration"][2];

  // IMU uses a different coordinates
  Eigen::Vector3d acc;
  acc << mbdata_["imu_linear_acceleration"][1], mbdata_["imu_linear_acceleration"][0], mbdata_["imu_linear_acceleration"][2];

  // Rotate body frame to world frame
  if (last_mocap_msg_ != NULL){
    Eigen::Matrix3d rot;
    tf2::Quaternion q(last_mocap_msg_->pose.orientation.x, last_mocap_msg_->pose.orientation.y, last_mocap_msg_->pose.orientation.z, last_mocap_msg_->pose.orientation.w);
    q.normalize();
    tf2::Matrix3x3 m(q);
    Eigen::Vector3d rpy;
    m.getRPY(rpy[0], rpy[1], rpy[2]);
    quadKD_->getRotationMatrix(rpy, rot);
    acc = rot*acc;

    // Ignore gravity
    acc[2] += 9.81;

    // Use new measurement
    imu_vel_estimate_ = (imu_vel_estimate_+acc/update_rate_)*(1-1/update_rate_/filter_time_constant_);
  } else {
    ROS_WARN_THROTTLE(1, "No body pose (mocap) recieved");
  }


  // Begin populating state message
  bool fully_populated = true;

  // last_robot_state_msg_.body.pose.pose.orientation = last_imu_msg_.orientation;
  last_robot_state_msg_.body.twist.angular = last_imu_msg_.angular_velocity;
  // last_robot_state_msg_.body.twist.linear = last_vel_msg_->twist.linear;

  if (last_mocap_msg_ != NULL)
  {
    last_robot_state_msg_.body.pose.orientation = last_mocap_msg_->pose.orientation;
    last_robot_state_msg_.body.pose.position = last_mocap_msg_->pose.position;

  } else {
    fully_populated = false;
    ROS_WARN_THROTTLE(1, "No body pose (mocap) recieved");

    last_robot_state_msg_.body.pose.orientation.x = 0;
    last_robot_state_msg_.body.pose.orientation.y = 0;
    last_robot_state_msg_.body.pose.orientation.z = 0;
    last_robot_state_msg_.body.pose.orientation.w = 1;
    last_robot_state_msg_.body.pose.position.x = 0;
    last_robot_state_msg_.body.pose.position.y = 0;
    last_robot_state_msg_.body.pose.position.z = 0;
  }
  
  if (last_mocap_msg_ != NULL)
  {
    // Complementary filter
    quad_utils::Eigen3ToVector3Msg(mocap_vel_estimate_ + imu_vel_estimate_, last_robot_state_msg_.body.twist.linear);
  }
  else
  {
    fully_populated = false;
    ROS_WARN_THROTTLE(1, "No mocap or imu recieved");

    last_robot_state_msg_.body.twist.linear.x = 0;
    last_robot_state_msg_.body.twist.linear.y = 0;
    last_robot_state_msg_.body.twist.linear.z = 0;
  }

  last_robot_state_msg_.joints.name.resize(joint_indices_.size());
  last_robot_state_msg_.joints.position.resize(joint_indices_.size());
  last_robot_state_msg_.joints.velocity.resize(joint_indices_.size());
  last_robot_state_msg_.joints.effort.resize(joint_indices_.size());

  for (size_t i = 0; i < joint_indices_.size(); i++)
  {
    last_robot_state_msg_.joints.name.at(i) = last_joint_state_msg_.name.at(joint_indices_.at(i));
    last_robot_state_msg_.joints.position.at(i) = last_joint_state_msg_.position.at(joint_indices_.at(i));
    last_robot_state_msg_.joints.velocity.at(i) = last_joint_state_msg_.velocity.at(joint_indices_.at(i));
    last_robot_state_msg_.joints.effort.at(i) = last_joint_state_msg_.effort.at(joint_indices_.at(i));
  }

  quad_utils::fkRobotState(*quadKD_, last_robot_state_msg_.body,last_robot_state_msg_.joints, last_robot_state_msg_.feet);

  quad_utils::updateStateHeaders(last_robot_state_msg_, state_timestamp, "map", 0);
  return fully_populated;
  
}

void LegControllerInterface::publishState() {
  imu_pub_.publish(last_imu_msg_);
  robot_state_pub_.publish(last_robot_state_msg_);
}

void LegControllerInterface::publishHeartbeat() {

  // Publish hearbeat
  std_msgs::Header msg;
  msg.stamp = ros::Time::now();
  robot_heartbeat_pub_.publish(msg);
}

void LegControllerInterface::publishLegCommandArray() {

  // Stamp and send the message
  leg_command_array_msg_.header.stamp = ros::Time::now();
  grf_array_msg_.header.stamp = leg_command_array_msg_.header.stamp;
  leg_command_array_pub_.publish(leg_command_array_msg_);
  grf_pub_.publish(grf_array_msg_);
}

void LegControllerInterface::spin() {
  ros::Rate r(update_rate_);
  ros::Time t_pub = ros::Time::now();
  while (ros::ok()) {

    // Collect new messages on subscriber topics and publish heartbeat
    ros::spinOnce();

    // Get new mainboard data and construct the new state message
    updateState();

    // // Compute the leg command and publish if valid
    bool valid_cmd = computeLegCommandArray();

    // If valid, send to the robot
    if (valid_cmd) {

      // Send command to the robot
      // Todo: support sending to gazebo
      std::cout << "sendMBlink" << std::endl;
      std::cout << "leg_command_array_msg_\n" <<  leg_command_array_msg_ << std::endl;
      mblink_converter_->sendMBlink(leg_command_array_msg_);
      std::cout << "cmd sent" << std::endl;
    } else {
      std::cout << "cmd invalid, not sending" << std::endl;
    }

    // // If publishing period had elapsed, publish the leg command for logging
    // // if ((ros::Time::now() - t_pub).toSec() >= publish_rate_) {
      publishLegCommandArray();
      publishState();
      // publishHeartbeat();
      // t_pub = ros::Time::now();
    // // }

    // Enforce update rate
    r.sleep();
  }
}
