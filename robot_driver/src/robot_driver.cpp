#include "robot_driver/robot_driver.h"

RobotDriver::RobotDriver(ros::NodeHandle nh, int argc, char** argv) {
  nh_ = nh;
  argc_ = argc;
  argv_ = argv;

  // Load rosparams from parameter server
  std::string imu_topic, joint_state_topic, grf_topic, robot_state_topic,
      trajectory_state_topic, local_plan_topic, leg_command_array_topic,
      control_mode_topic, remote_heartbeat_topic, robot_heartbeat_topic,
      single_joint_cmd_topic, mocap_topic, control_restart_flag_topic;
  quad_utils::loadROSParam(nh_, "topics/state/imu", imu_topic);
  quad_utils::loadROSParam(nh_, "topics/state/joints", joint_state_topic);
  quad_utils::loadROSParam(nh_, "topics/local_plan", local_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/state/ground_truth", robot_state_topic);
  quad_utils::loadROSParam(nh_, "topics/state/trajectory",
                           trajectory_state_topic);
  quad_utils::loadROSParam(nh_, "topics/heartbeat/remote",
                           remote_heartbeat_topic);
  quad_utils::loadROSParam(nh_, "topics/heartbeat/robot",
                           robot_heartbeat_topic);
  quad_utils::loadROSParam(nh_, "topics/control/grfs", grf_topic);
  quad_utils::loadROSParam(nh_, "topics/control/joint_command",
                           leg_command_array_topic);
  quad_utils::loadROSParam(nh_, "topics/control/mode", control_mode_topic);
  quad_utils::loadROSParam(nh_, "topics/control/single_joint_command",
                           single_joint_cmd_topic);
  quad_utils::loadROSParam(nh_, "topics/control/restart_flag",
                           control_restart_flag_topic);
  quad_utils::loadROSParam(nh_, "topics/mocap", mocap_topic);

  nh_.param<bool>("robot_driver/is_hardware", is_hardware_, true);
  nh_.param<std::string>("robot_driver/robot_name", robot_name_, "spirit");
  nh_.param<std::string>("robot_driver/controller", controller_id_,
                         "inverse_dynamics");
  quad_utils::loadROSParam(nh_, "robot_driver/update_rate", update_rate_);
  quad_utils::loadROSParam(nh_, "robot_driver/publish_rate", publish_rate_);
  quad_utils::loadROSParam(nh_, "robot_driver/mocap_rate", mocap_rate_);
  quad_utils::loadROSParam(nh_, "robot_driver/mocap_dropout_threshold",
                           mocap_dropout_threshold_);
  quad_utils::loadROSParam(nh_, "robot_driver/filter_time_constant",
                           filter_time_constant_);
  quad_utils::loadROSParam(nh_, "robot_driver/input_timeout", input_timeout_);
  quad_utils::loadROSParam(nh_, "robot_driver/state_timeout", state_timeout_);
  quad_utils::loadROSParam(nh_, "robot_driver/heartbeat_timeout",
                           heartbeat_timeout_);
  quad_utils::loadROSParam(nh_, "robot_driver/sit_kp", sit_kp_);
  quad_utils::loadROSParam(nh_, "robot_driver/sit_kd", sit_kd_);
  quad_utils::loadROSParam(nh_, "robot_driver/stand_kp", stand_kp_);
  quad_utils::loadROSParam(nh_, "robot_driver/stand_kd", stand_kd_);
  quad_utils::loadROSParam(nh_, "robot_driver/stance_kp", stance_kp_);
  quad_utils::loadROSParam(nh_, "robot_driver/stance_kd", stance_kd_);
  quad_utils::loadROSParam(nh_, "robot_driver/swing_kp", swing_kp_);
  quad_utils::loadROSParam(nh_, "robot_driver/swing_kd", swing_kd_);
  quad_utils::loadROSParam(nh_, "robot_driver/swing_kp_cart", swing_kp_cart_);
  quad_utils::loadROSParam(nh_, "robot_driver/swing_kd_cart", swing_kd_cart_);
  quad_utils::loadROSParam(nh_, "robot_driver/safety_kp", safety_kp_);
  quad_utils::loadROSParam(nh_, "robot_driver/safety_kd", safety_kd_);
  quad_utils::loadROSParam(nh_, "robot_driver/stand_joint_angles",
                           stand_joint_angles_);
  quad_utils::loadROSParam(nh_, "robot_driver/sit_joint_angles",
                           sit_joint_angles_);

  // Setup pubs and subs
  local_plan_sub_ =
      nh_.subscribe(local_plan_topic, 1, &RobotDriver::localPlanCallback, this,
                    ros::TransportHints().tcpNoDelay(true));
  control_mode_sub_ = nh_.subscribe(control_mode_topic, 1,
                                    &RobotDriver::controlModeCallback, this);
  single_joint_cmd_sub_ =
      nh_.subscribe(single_joint_cmd_topic, 1,
                    &RobotDriver::singleJointCommandCallback, this);
  remote_heartbeat_sub_ = nh_.subscribe(
      remote_heartbeat_topic, 1, &RobotDriver::remoteHeartbeatCallback, this);
  control_restart_flag_sub_ =
      nh_.subscribe(control_restart_flag_topic, 1,
                    &RobotDriver::controlRestartFlagCallback, this);
  grf_pub_ = nh_.advertise<quad_msgs::GRFArray>(grf_topic, 1);
  leg_command_array_pub_ =
      nh_.advertise<quad_msgs::LegCommandArray>(leg_command_array_topic, 1);
  robot_heartbeat_pub_ =
      nh_.advertise<std_msgs::Header>(robot_heartbeat_topic, 1);
  trajectry_robot_state_pub_ =
      nh_.advertise<quad_msgs::RobotState>(trajectory_state_topic, 1);

  // Set up pubs and subs dependent on robot layer
  if (is_hardware_) {
    ROS_INFO("Loading hardware robot driver");
    mocap_sub_ = nh_.subscribe(mocap_topic, 1000, &RobotDriver::mocapCallback,
                               this, ros::TransportHints().tcpNoDelay(true));
    robot_state_pub_ =
        nh_.advertise<quad_msgs::RobotState>(robot_state_topic, 1);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic, 1);
    joint_state_pub_ =
        nh_.advertise<sensor_msgs::JointState>(joint_state_topic, 1);
  } else {
    ROS_INFO("Loading sim robot driver");
    robot_state_sub_ =
        nh_.subscribe(robot_state_topic, 1, &RobotDriver::robotStateCallback,
                      this, ros::TransportHints().tcpNoDelay(true));
  }

  // Initialize kinematics object
  quadKD_ = std::make_shared<quad_utils::QuadKD>();

  // Initialize hardware interface
  if (is_hardware_) {
    if (robot_name_ == "spirit") {
      hardware_interface_ = std::make_shared<SpiritInterface>();
    } else {
      ROS_ERROR_STREAM("Invalid robot name " << robot_name_
                                             << ", returning nullptr");
      hardware_interface_ = nullptr;
    }
  }

  // Initialize leg controller object
  if (controller_id_ == "inverse_dynamics") {
    leg_controller_ = std::make_shared<InverseDynamicsController>();
  } else if (controller_id_ == "grf_pid") {
    leg_controller_ = std::make_shared<GrfPidController>();
  } else if (controller_id_ == "joint") {
    leg_controller_ = std::make_shared<JointController>();
  } else {
    ROS_ERROR_STREAM("Invalid controller id " << controller_id_
                                              << ", returning nullptr");
    leg_controller_ = nullptr;
  }
  leg_controller_->setGains(stance_kp_, stance_kd_, swing_kp_, swing_kd_,
                            swing_kp_cart_, swing_kd_cart_);

  // Start sitting
  control_mode_ = SIT;
  remote_heartbeat_received_time_ = std::numeric_limits<double>::max();
  last_state_time_ = std::numeric_limits<double>::max();

  // Set joint torque limits
  torque_limits_ << 21, 21, 32;

  // Initialize timing
  last_robot_state_msg_.header.stamp = ros::Time::now();
  t_pub_ = ros::Time::now();

  // Initialize state and control data structures
  double dt = 1.0 / mocap_rate_;
  filter_weight_ = 1.0 - dt / filter_time_constant_;

  vel_estimate_.setZero();
  mocap_vel_estimate_.setZero();
  imu_vel_estimate_.setZero();
  last_joint_state_msg_.name.resize(12);
  last_joint_state_msg_.position.resize(12);
  last_joint_state_msg_.velocity.resize(12);
  last_joint_state_msg_.effort.resize(12);
  grf_array_msg_.vectors.resize(4);
  grf_array_msg_.points.resize(4);
  grf_array_msg_.contact_states.resize(4);
  grf_array_msg_.header.frame_id = "map";
  user_tx_data_.resize(1);

  std::vector<double> high_pass_a, high_pass_b, high_pass_c, high_pass_d;
  quad_utils::loadROSParam(nh_, "robot_driver/high_pass_a", high_pass_a);
  quad_utils::loadROSParam(nh_, "robot_driver/high_pass_b", high_pass_b);
  quad_utils::loadROSParam(nh_, "robot_driver/high_pass_c", high_pass_c);
  quad_utils::loadROSParam(nh_, "robot_driver/high_pass_d", high_pass_d);
  complementary_filter_.high_pass_filter.x.resize(3);
  complementary_filter_.high_pass_filter.init = false;
  complementary_filter_.high_pass_filter.A =
      Eigen::Map<Eigen::Matrix<double, 2, 2> >(high_pass_a.data()).transpose();
  complementary_filter_.high_pass_filter.B =
      Eigen::Map<Eigen::Matrix<double, 1, 2> >(high_pass_b.data()).transpose();
  complementary_filter_.high_pass_filter.C =
      Eigen::Map<Eigen::Matrix<double, 2, 1> >(high_pass_c.data()).transpose();
  complementary_filter_.high_pass_filter.D =
      Eigen::Map<Eigen::Matrix<double, 1, 1> >(high_pass_d.data()).transpose();

  std::vector<double> low_pass_a, low_pass_b, low_pass_c, low_pass_d;
  quad_utils::loadROSParam(nh_, "robot_driver/low_pass_a", low_pass_a);
  quad_utils::loadROSParam(nh_, "robot_driver/low_pass_b", low_pass_b);
  quad_utils::loadROSParam(nh_, "robot_driver/low_pass_c", low_pass_c);
  quad_utils::loadROSParam(nh_, "robot_driver/low_pass_d", low_pass_d);
  complementary_filter_.low_pass_filter.x.resize(3);
  complementary_filter_.low_pass_filter.init = false;
  complementary_filter_.low_pass_filter.A =
      Eigen::Map<Eigen::Matrix<double, 2, 2> >(low_pass_a.data()).transpose();
  complementary_filter_.low_pass_filter.B =
      Eigen::Map<Eigen::Matrix<double, 1, 2> >(low_pass_b.data()).transpose();
  complementary_filter_.low_pass_filter.C =
      Eigen::Map<Eigen::Matrix<double, 2, 1> >(low_pass_c.data()).transpose();
  complementary_filter_.low_pass_filter.D =
      Eigen::Map<Eigen::Matrix<double, 1, 1> >(low_pass_d.data()).transpose();
}

void RobotDriver::controlModeCallback(const std_msgs::UInt8::ConstPtr& msg) {
  // Wait if transitioning
  if ((control_mode_ == SIT_TO_READY) || (control_mode_ == READY_TO_SIT))
    return;
  if ((msg->data == READY) &&
      (control_mode_ == SIT)) {  // Stand if previously sitting
    control_mode_ = SIT_TO_READY;
    transition_timestamp_ = ros::Time::now();
  } else if ((msg->data == SIT) &&
             (control_mode_ == READY)) {  // Sit if previously standing
    control_mode_ = READY_TO_SIT;
    transition_timestamp_ = ros::Time::now();
  } else if (msg->data == SIT ||
             (msg->data == SAFETY)) {  // Allow sit or safety modes
    control_mode_ = msg->data;
  }
}

void RobotDriver::singleJointCommandCallback(
    const geometry_msgs::Vector3::ConstPtr& msg) {
  if (JointController* c =
          dynamic_cast<JointController*>(leg_controller_.get())) {
    c->updateSingleJointCommand(msg);
  }
}

void RobotDriver::controlRestartFlagCallback(
    const std_msgs::Bool::ConstPtr& msg) {
  user_tx_data_[0] = (msg->data) ? 1 : 0;
}

void RobotDriver::localPlanCallback(const quad_msgs::RobotPlan::ConstPtr& msg) {
  last_local_plan_msg_ = msg;

  ros::Time t_now = ros::Time::now();
  double round_trip_time_diff =
      (t_now - last_local_plan_msg_->state_timestamp).toSec();

  leg_controller_->updateLocalPlanMsg(last_local_plan_msg_, t_now);
}

void RobotDriver::mocapCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // Collect position reading
  Eigen::Vector3d pos;
  quad_utils::pointMsgToEigen(msg->pose.position, pos);

  if (complementary_filter_.low_pass_filter.init) {
    // Record time diff between messages
    ros::Time t_now = ros::Time::now();
    double t_diff_mocap_msg =
        (msg->header.stamp - last_mocap_msg_->header.stamp).toSec();
    double t_mocap_ros_latency = (t_now - msg->header.stamp).toSec();
    last_mocap_time_ = t_now;

    // Apply filter
    if (abs(t_diff_mocap_msg - 1.0 / mocap_rate_) < mocap_dropout_threshold_) {
      for (size_t i = 0; i < 3; i++) {
        // Compute outputs
        mocap_vel_estimate_(i) =
            (complementary_filter_.low_pass_filter.C *
                 complementary_filter_.low_pass_filter.x.at(i) +
             complementary_filter_.low_pass_filter.D * pos(i))(0, 0);

        // Compute states
        complementary_filter_.low_pass_filter.x.at(i) =
            complementary_filter_.low_pass_filter.A *
                complementary_filter_.low_pass_filter.x.at(i) +
            complementary_filter_.low_pass_filter.B * pos(i);
      }
    } else {
      ROS_WARN_THROTTLE(
          0.1,
          "Mocap time diff exceeds max dropout threshold, hold the last value");
    }
  } else {
    // Init filter
    Eigen::Matrix<double, 3, 2> left;
    left.topRows(2) =
        complementary_filter_.low_pass_filter.A - Eigen::Matrix2d::Identity();
    left.bottomRows(1) = complementary_filter_.low_pass_filter.C;

    Eigen::Matrix<double, 3, 1> right;
    right.topRows(2) = -complementary_filter_.low_pass_filter.B;
    right.bottomRows(1) = -complementary_filter_.low_pass_filter.D;

    for (size_t i = 0; i < 3; i++) {
      complementary_filter_.low_pass_filter.x.at(i) =
          left.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
              .solve(right * pos(i));
    }

    complementary_filter_.low_pass_filter.init = true;
  }

  // Update our cached mocap position
  last_mocap_msg_ = msg;
}

void RobotDriver::robotStateCallback(
    const quad_msgs::RobotState::ConstPtr& msg) {
  last_robot_state_msg_ = *msg;
}

void RobotDriver::remoteHeartbeatCallback(
    const std_msgs::Header::ConstPtr& msg) {
  // Get the current time and compare to the message time
  double remote_heartbeat_sent_time = msg->stamp.toSec();
  remote_heartbeat_received_time_ = ros::Time::now().toSec();
  double t_latency =
      remote_heartbeat_received_time_ - remote_heartbeat_sent_time;

  // ROS_INFO_THROTTLE(1.0,"Remote latency (+ clock skew) = %6.4fs", t_latency);
}

void RobotDriver::checkMessagesForSafety() {
  // Do nothing if already in safety mode
  if (control_mode_ == SAFETY) return;

  // Check the remote heartbeat for timeout
  // (this adds extra safety if no heartbeat messages are arriving)
  if (abs(ros::Time::now().toSec() - remote_heartbeat_received_time_) >=
          heartbeat_timeout_ &&
      remote_heartbeat_received_time_ != std::numeric_limits<double>::max()) {
    control_mode_ = SAFETY;
    ROS_WARN_THROTTLE(1,
                      "Remote heartbeat lost or late to robot driver node, "
                      "entering safety mode");
  }

  // Check the state message latency
  if (!is_hardware_ &&
      abs(ros::Time::now().toSec() - last_state_time_) >= state_timeout_ &&
      last_state_time_ != std::numeric_limits<double>::max()) {
    control_mode_ = SAFETY;
    transition_timestamp_ = ros::Time::now();
    ROS_WARN_THROTTLE(
        1, "State messages lost in robot driver node, entering safety mode");
  }
}

bool RobotDriver::updateState() {
  if (is_hardware_) {
    // Get the newest data from the robot (BLOCKING)
    bool fully_populated = hardware_interface_->recv(
        last_joint_state_msg_, last_imu_msg_, user_rx_data_);

    ros::Time state_timestamp = ros::Time::now();

    // Check if robot data was recieved
    if (fully_populated) {
      last_robot_state_msg_.body.twist.angular = last_imu_msg_.angular_velocity;
      last_robot_state_msg_.joints = last_joint_state_msg_;
      last_joint_state_msg_.header.stamp = state_timestamp;
      last_imu_msg_.header.stamp = state_timestamp;
    } else {
      ROS_WARN_THROTTLE(1, "No imu or joint state (robot) recieved");
    }

    // Check if mocap data was received
    if (last_mocap_msg_ != NULL) {
      // Copy mocap readings
      last_robot_state_msg_.body.pose.orientation =
          last_mocap_msg_->pose.orientation;
      last_robot_state_msg_.body.pose.position = last_mocap_msg_->pose.position;

      // IMU is in body frame
      Eigen::Vector3d acc;
      acc << last_imu_msg_.linear_acceleration.x,
          last_imu_msg_.linear_acceleration.y,
          last_imu_msg_.linear_acceleration.z;

      Eigen::Matrix3d rot;
      tf2::Quaternion q(last_mocap_msg_->pose.orientation.x,
                        last_mocap_msg_->pose.orientation.y,
                        last_mocap_msg_->pose.orientation.z,
                        last_mocap_msg_->pose.orientation.w);
      q.normalize();
      tf2::Matrix3x3 m(q);
      Eigen::Vector3d rpy;
      m.getRPY(rpy[0], rpy[1], rpy[2]);
      quadKD_->getRotationMatrix(rpy, rot);
      acc = rot * acc;

      // Ignore gravity
      acc[2] += 9.81;

      // I guess the acceleration is negative
      acc = -acc;

      if (!complementary_filter_.high_pass_filter.init) {
        // Init filter
        for (size_t i = 0; i < 3; i++) {
          complementary_filter_.high_pass_filter.x.at(i) << 0, 0;
        }

        complementary_filter_.high_pass_filter.init = true;
      }

      // Apply filter
      for (size_t i = 0; i < 3; i++) {
        // Compute outputs
        imu_vel_estimate_(i) =
            (complementary_filter_.high_pass_filter.C *
                 complementary_filter_.high_pass_filter.x.at(i) +
             complementary_filter_.high_pass_filter.D * acc(i))(0, 0);

        // Compute states
        complementary_filter_.high_pass_filter.x.at(i) =
            complementary_filter_.high_pass_filter.A *
                complementary_filter_.high_pass_filter.x.at(i) +
            complementary_filter_.high_pass_filter.B * acc(i);
      }

      // Complementary filter
      vel_estimate_ = imu_vel_estimate_ + mocap_vel_estimate_;
      quad_utils::Eigen3ToVector3Msg(vel_estimate_,
                                     last_robot_state_msg_.body.twist.linear);

    } else {
      ROS_WARN_THROTTLE(1, "No body pose (mocap) recieved");
      bool fully_populated = false;
      last_robot_state_msg_.body.pose.orientation.w = 1;
    }

    // Fill in the rest of the state message (foot state and headers)
    quad_utils::fkRobotState(*quadKD_, last_robot_state_msg_);
    quad_utils::updateStateHeaders(last_robot_state_msg_, state_timestamp,
                                   "map", 0);
    return fully_populated;

  } else {
    // State information coming through sim subscribers, not hardware interface
    return true;
  }
}

void RobotDriver::publishState() {
  if (is_hardware_) {
    imu_pub_.publish(last_imu_msg_);
    joint_state_pub_.publish(last_joint_state_msg_);
    robot_state_pub_.publish(last_robot_state_msg_);
  }
}

bool RobotDriver::updateControl() {
  // Check if state machine should be skipped
  bool valid_cmd = true;
  if (leg_controller_->overrideStateMachine()) {
    valid_cmd = leg_controller_->computeLegCommandArray(
        last_robot_state_msg_, leg_command_array_msg_, grf_array_msg_);
    return valid_cmd;
  }

  // Check incoming messages to determine if we should enter safety mode
  checkMessagesForSafety();

  if (last_robot_state_msg_.header.stamp.toSec() == 0) {
    return false;
  }

  // Define vectors for joint positions and velocities
  Eigen::VectorXd joint_positions(3 * num_feet_),
      joint_velocities(3 * num_feet_), body_state(12);
  quad_utils::vectorToEigen(last_robot_state_msg_.joints.position,
                            joint_positions);
  quad_utils::vectorToEigen(last_robot_state_msg_.joints.velocity,
                            joint_velocities);

  // Initialize leg command message
  leg_command_array_msg_.leg_commands.resize(num_feet_);

  // Enter state machine for filling motor command message
  if (control_mode_ == SAFETY) {
    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        int joint_idx = 3 * i + j;

        robot_driver_utils::loadMotorCommandMsg(
            0, 0, 0, safety_kp_.at(j), safety_kd_.at(j),
            leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j));
      }
    }
  } else if (control_mode_ == SIT) {
    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        robot_driver_utils::loadMotorCommandMsg(
            sit_joint_angles_.at(j), 0, 0, sit_kp_.at(j), sit_kd_.at(j),
            leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j));
      }
    }
  } else if (control_mode_ == READY) {
    if (leg_controller_->computeLegCommandArray(last_robot_state_msg_,
                                                leg_command_array_msg_,
                                                grf_array_msg_) == false) {
      for (int i = 0; i < num_feet_; ++i) {
        leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);
        for (int j = 0; j < 3; ++j) {
          int joint_idx = 3 * i + j;
          robot_driver_utils::loadMotorCommandMsg(
              stand_joint_angles_.at(j), 0, 0, stand_kp_.at(j), stand_kd_.at(j),
              leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j));
        }
      }
    } else {
      if (InverseDynamicsController* p =
              dynamic_cast<InverseDynamicsController*>(leg_controller_.get())) {
        // Uncomment to publish trajectory reference state
        // quad_msgs::RobotState ref_state_msg = p->getReferenceState();
        // trajectry_robot_state_pub_.publish(ref_state_msg);
      }
    }
  } else if (control_mode_ == SIT_TO_READY) {
    ros::Duration duration = ros::Time::now() - transition_timestamp_;
    double t_interp = duration.toSec() / transition_duration_;
    if (t_interp >= 1) {
      control_mode_ = READY;
      return valid_cmd;
    }
    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        double ang =
            (stand_joint_angles_.at(j) - sit_joint_angles_.at(j)) * t_interp +
            sit_joint_angles_.at(j);

        robot_driver_utils::loadMotorCommandMsg(
            ang, 0, 0, stand_kp_.at(j), stand_kd_.at(j),
            leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j));
      }
    }
  } else if (control_mode_ == READY_TO_SIT) {
    ros::Duration duration = ros::Time::now() - transition_timestamp_;
    double t_interp = duration.toSec() / transition_duration_;

    if (t_interp >= 1) {
      control_mode_ = SIT;
      return valid_cmd;
    }

    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        double ang =
            (sit_joint_angles_.at(j) - stand_joint_angles_.at(j)) * t_interp +
            stand_joint_angles_.at(j);

        robot_driver_utils::loadMotorCommandMsg(
            ang, 0, 0, stand_kp_.at(j), stand_kd_.at(j),
            leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j));
      }
    }
  } else {
    ROS_WARN_THROTTLE(0.5,
                      "Invalid control mode set in ID node, "
                      "exiting updateControl()");
    return false;
  }

  const int knee_idx = 2;
  const int knee_soft_ub = 3.0;
  const int knee_soft_ub_kd = 50.0;

  for (int i = 0; i < num_feet_; ++i) {
    for (int j = 0; j < 3; ++j) {
      int joint_idx = 3 * i + j;

      // Add soft joint limit for knees
      if (j == knee_idx && joint_positions(joint_idx) > knee_soft_ub) {
        leg_command_array_msg_.leg_commands.at(i)
            .motor_commands.at(j)
            .torque_ff = std::max(
            leg_command_array_msg_.leg_commands.at(i)
                    .motor_commands.at(j)
                    .torque_ff -
                knee_soft_ub_kd * (joint_positions(joint_idx) - knee_soft_ub),
            -torque_limits_[j]);
      }

      quad_msgs::MotorCommand cmd =
          leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j);
      double pos_component =
          cmd.kp * (cmd.pos_setpoint - joint_positions[joint_idx]);
      double vel_component =
          cmd.kd * (cmd.vel_setpoint - joint_velocities[joint_idx]);
      double fb_component = pos_component + vel_component;
      double effort = fb_component + cmd.torque_ff;
      double fb_ratio =
          abs(fb_component) / (abs(fb_component) + abs(cmd.torque_ff));

      if (abs(cmd.torque_ff) >= torque_limits_[j]) {
        ROS_WARN(
            "Leg %d motor %d: ff effort = %5.3f Nm exceeds threshold of %5.3f "
            "Nm",
            i, j, cmd.torque_ff, torque_limits_[j]);
      }
      if (abs(effort) >= torque_limits_[j]) {
        ROS_WARN(
            "Leg %d motor %d: total effort = %5.3f Nm exceeds threshold of "
            "%5.3f Nm",
            i, j, effort, torque_limits_[j]);
        effort =
            std::min(std::max(effort, -torque_limits_[j]), torque_limits_[j]);
      }

      leg_command_array_msg_.leg_commands.at(i)
          .motor_commands.at(j)
          .pos_component = pos_component;
      leg_command_array_msg_.leg_commands.at(i)
          .motor_commands.at(j)
          .vel_component = vel_component;
      leg_command_array_msg_.leg_commands.at(i)
          .motor_commands.at(j)
          .fb_component = fb_component;
      leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).effort =
          effort;
      leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).fb_ratio =
          fb_ratio;
    }
  }

  return valid_cmd;
}

void RobotDriver::publishControl(bool is_valid) {
  // Stamp and send the message
  // if ((ros::Time::now() - leg_command_array_msg_.header.stamp).toSec()
  // >= 1.0/publish_rate_) {
  leg_command_array_msg_.header.stamp = ros::Time::now();
  leg_command_array_pub_.publish(leg_command_array_msg_);
  grf_array_msg_.header.stamp = leg_command_array_msg_.header.stamp;
  grf_pub_.publish(grf_array_msg_);
  // }

  // Send command to the robot
  if (is_hardware_ && is_valid) {
    ros::Time t_start = ros::Time::now();
    hardware_interface_->send(leg_command_array_msg_, user_tx_data_);
    ros::Time t_end = ros::Time::now();

    ROS_INFO_THROTTLE(1.0, "t_diff_mb_send = %6.4f", (t_end - t_start).toSec());
  }
}

void RobotDriver::publishHeartbeat() {
  // Publish hearbeat
  if ((ros::Time::now() - last_robot_heartbeat_msg_.stamp).toSec() >=
      1.0 / publish_rate_) {
    last_robot_heartbeat_msg_.stamp = ros::Time::now();
    robot_heartbeat_pub_.publish(last_robot_heartbeat_msg_);
  }
}

void RobotDriver::spin() {
  // Initialize timing params
  ros::Rate r(update_rate_);

  // Start the mblink connection
  if (is_hardware_) {
    hardware_interface_->loadInterface(argc_, argv_);
  }

  while (ros::ok()) {
    // Collect new messages on subscriber topics and publish heartbeat
    ros::spinOnce();

    // Get the newest state information
    updateState();

    // Compute the leg command and publish if valid
    bool is_valid = updateControl();
    publishControl(is_valid);

    // // Publish state and heartbeat
    publishState();
    publishHeartbeat();

    // Enforce update rate
    r.sleep();
  }

  // Close the mblink connection
  if (is_hardware_) {
    hardware_interface_->unloadInterface();
  }
}
