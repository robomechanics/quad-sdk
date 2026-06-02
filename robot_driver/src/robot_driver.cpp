#include "robot_driver/robot_driver.hpp"

RobotDriver::RobotDriver(std::shared_ptr<rclcpp::Node> node, int argc,
                         char** argv)
    : node_(node), argc_(argc), argv_(argv) {
  std::string imu_topic, joint_state_topic, grf_topic, robot_state_topic,
      trajectory_state_topic, local_plan_topic, leg_command_array_topic,
      control_mode_topic, remote_heartbeat_topic, robot_heartbeat_topic,
      single_joint_cmd_topic, mocap_topic, control_restart_flag_topic,
      body_force_estimate_topic, cmd_vel_topic, cmd_vel_stamped_topic,
      state_estimate_topic;
  bool found_torque_limits = false;

  quad_utils::loadROSParamDefault(node_, "namespace", robot_ns,
                                  std::string("robot_1"));
  quad_utils::loadROSParam(node_, "robot_description", robot_description);
  quad_utils::loadROSParamDefault(node_, "robot_type", robot_name,
                                  std::string("spirit"));
  quad_utils::loadROSParam(node_, "topics.state.imu", imu_topic);
  quad_utils::loadROSParam(node_, "topics.state.joints", joint_state_topic);
  quad_utils::loadROSParam(node_, "topics.local_plan", local_plan_topic);
  quad_utils::loadROSParam(node_, "topics.state.ground_truth",
                           robot_state_topic);
  quad_utils::loadROSParam(node_, "topics.state.trajectory",
                           trajectory_state_topic);
  quad_utils::loadROSParam(node_, "topics.heartbeat.remote",
                           remote_heartbeat_topic);
  quad_utils::loadROSParam(node_, "topics.heartbeat.robot",
                           robot_heartbeat_topic);
  quad_utils::loadROSParam(node_, "topics.body_force.joint_torques",
                           body_force_estimate_topic);
  quad_utils::loadROSParam(node_, "topics.control.grfs", grf_topic);
  quad_utils::loadROSParam(node_, "topics.control.joint_command",
                           leg_command_array_topic);
  quad_utils::loadROSParam(node_, "topics.control.mode", control_mode_topic);
  quad_utils::loadROSParam(node_, "topics.control.single_joint_command",
                           single_joint_cmd_topic);
  quad_utils::loadROSParam(node_, "topics.control.restart_flag",
                           control_restart_flag_topic);
  quad_utils::loadROSParam(node_, "topics.mocap", mocap_topic);
  quad_utils::loadROSParam(node_, "topics.cmd_vel", cmd_vel_topic);
  quad_utils::loadROSParam(node_, "topics.cmd_vel_stamped",
                           cmd_vel_stamped_topic);
  quad_utils::loadROSParam(node_, "topics.state.estimate",
                           state_estimate_topic);
  quad_utils::loadROSParamDefault(node_, "is_hardware", is_hardware_, true);
  quad_utils::loadROSParamDefault(node_, "controller", controller_id_,
                                  std::string("inverse_dynamics"));
  quad_utils::loadROSParamDefault(node_, "estimator_id", estimator_id_,
                                  std::string("comp_filter"));
  quad_utils::loadROSParamDefault(node_, "debug_estimator_id",
                                  debug_estimator_id_, std::string("none"));
  quad_utils::loadROSParam(node_, "robot_driver.update_rate", update_rate_);
  quad_utils::loadROSParam(node_, "robot_driver.publish_rate", publish_rate_);
  quad_utils::loadROSParam(node_, "robot_driver.mocap_rate", mocap_rate_);
  quad_utils::loadROSParam(node_, "robot_driver.mocap_dropout_threshold",
                           mocap_dropout_threshold_);
  quad_utils::loadROSParam(node_, "robot_driver.filter_time_constant",
                           filter_time_constant_);
  quad_utils::loadROSParam(node_, "robot_driver.input_timeout", input_timeout_);
  quad_utils::loadROSParam(node_, "robot_driver.state_timeout", state_timeout_);
  quad_utils::loadROSParam(node_, "robot_driver.heartbeat_timeout",
                           heartbeat_timeout_);
  quad_utils::loadROSParam(node_, "robot_driver.sit_kp", sit_kp_);
  quad_utils::loadROSParam(node_, "robot_driver.sit_kd", sit_kd_);
  quad_utils::loadROSParam(node_, "robot_driver.stand_kp", stand_kp_);
  quad_utils::loadROSParam(node_, "robot_driver.stand_kd", stand_kd_);
  quad_utils::loadROSParam(node_, "robot_driver.stance_kp", stance_kp_);
  quad_utils::loadROSParam(node_, "robot_driver.stance_kd", stance_kd_);
  quad_utils::loadROSParam(node_, "robot_driver.swing_kp", swing_kp_);
  quad_utils::loadROSParam(node_, "robot_driver.swing_kd", swing_kd_);
  quad_utils::loadROSParam(node_, "robot_driver.swing_kp_cart", swing_kp_cart_);
  quad_utils::loadROSParam(node_, "robot_driver.swing_kd_cart", swing_kd_cart_);
  quad_utils::loadROSParam(node_, "robot_driver.safety_kp", safety_kp_);
  quad_utils::loadROSParam(node_, "robot_driver.safety_kd", safety_kd_);
  quad_utils::loadROSParam(node_, "robot_driver.stand_joint_angles",
                           stand_joint_angles_);
  quad_utils::loadROSParam(node_, "robot_driver.sit_joint_angles",
                           sit_joint_angles_);
  found_torque_limits =
      quad_utils::loadROSParam(node_, "motor_limits.torque", torque_limits_);
  if (!found_torque_limits) {
    quad_utils::loadROSParam(node_, "robot_driver.torque_limit",
                             torque_limits_);
    RCLCPP_WARN(node_->get_logger(),
                "Using legacy parameter 'robot_driver.torque_limit'; migrate "
                "to 'motor_limits.torque'");
  }
  quad_utils::loadROSParam(node_, "robot_driver.model_path", model_path_);
  quad_utils::loadROSParamDefault(node_, "robot_driver.policy_inference_rate",
                                  policy_inference_rate_, 50.0);
  quad_utils::loadROSParam(node_, "robot_driver.cmd_vel_filter_const",
                           cmd_vel_filter_const_);
  quad_utils::loadROSParam(node_, "robot_driver.cmd_vel_scale", cmd_vel_scale_);

  // Setup pubs and subs
  local_plan_sub_ = node_->create_subscription<quad_msgs::msg::RobotPlan>(
      local_plan_topic, rclcpp::QoS(1).best_effort().reliable().keep_last(1),
      std::bind(&RobotDriver::localPlanCallback, this, std::placeholders::_1));

  control_mode_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      control_mode_topic, 1,
      std::bind(&RobotDriver::controlModeCallback, this,
                std::placeholders::_1));

  single_joint_cmd_sub_ =
      node_->create_subscription<geometry_msgs::msg::Vector3>(
          single_joint_cmd_topic, 1,
          std::bind(&RobotDriver::singleJointCommandCallback, this,
                    std::placeholders::_1));

  body_force_estimate_sub_ =
      node_->create_subscription<quad_msgs::msg::BodyForceEstimate>(
          body_force_estimate_topic, 1,
          std::bind(&RobotDriver::bodyForceEstimateCallback, this,
                    std::placeholders::_1));

  remote_heartbeat_sub_ = node_->create_subscription<std_msgs::msg::Header>(
      remote_heartbeat_topic, 1,
      std::bind(&RobotDriver::remoteHeartbeatCallback, this,
                std::placeholders::_1));

  control_restart_flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      control_restart_flag_topic, 1,
      std::bind(&RobotDriver::controlRestartFlagCallback, this,
                std::placeholders::_1));

  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      std::bind(&RobotDriver::cmdVelCallback, this, std::placeholders::_1));

  grf_pub_ = node_->create_publisher<quad_msgs::msg::GRFArray>(grf_topic, 1);
  leg_command_array_pub_ =
      node_->create_publisher<quad_msgs::msg::LegCommandArray>(
          leg_command_array_topic, 1);
  robot_heartbeat_pub_ =
      node_->create_publisher<std_msgs::msg::Header>(robot_heartbeat_topic, 1);
  trajectry_robot_state_pub_ =
      node_->create_publisher<quad_msgs::msg::RobotState>(
          trajectory_state_topic, 1);
  cmd_vel_stamped_pub_ =
      node_->create_publisher<geometry_msgs::msg::TwistStamped>(
          cmd_vel_stamped_topic, 1);
  state_estimate_pub_ = node_->create_publisher<quad_msgs::msg::RobotState>(
      state_estimate_topic, 1);

  // Foot-contact publisher (raw + thresholded foot-force from Unitree).
  // Topic name kept simple; logger and any subscriber pick it up by name.
  std::string foot_contact_topic = "state/foot_contact";
  quad_utils::loadROSParamDefault(node_, "topics.state.foot_contact",
                                  foot_contact_topic,
                                  std::string("state/foot_contact"));
  foot_contact_pub_ = node_->create_publisher<quad_msgs::msg::FootContact>(
      foot_contact_topic, 10);
  quad_utils::loadROSParamDefault(node_, "robot_driver.foot_contact_threshold",
                                  foot_contact_threshold_, 30);

  // Set up pubs and subs dependent on robot layer
  if (is_hardware_) {
    RCLCPP_INFO(node_->get_logger(), "Loading Hardware Robot Driver");
    mocap_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        mocap_topic, 1000,
        std::bind(&RobotDriver::mocapCallback, this, std::placeholders::_1));
    robot_state_pub_ = node_->create_publisher<quad_msgs::msg::RobotState>(
        robot_state_topic, 1);
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 1);
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        joint_state_topic, rclcpp::SensorDataQoS());
  } else {
    RCLCPP_INFO(node_->get_logger(), "Loading Sim Robot Driver");
    robot_state_sub_ = node_->create_subscription<quad_msgs::msg::RobotState>(
        robot_state_topic, 1,
        std::bind(&RobotDriver::robotStateCallback, this,
                  std::placeholders::_1));
  }

  // Initialize kinematics object
  quadKD2_ = std::make_shared<quad_utils::QuadKD2>(node_, robot_ns);

  // Initialize hardware interface
  if (is_hardware_) {
    if (robot_name == "spirit" || robot_name == "spirit_rotors") {
      hardware_interface_ = std::make_shared<SpiritInterface>();
    } else if (robot_name == "go2" || robot_name == "go2w") {
      hardware_interface_ = std::make_shared<UnitreeInterface>(robot_name);
    } else {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Invalid robot name "
                                                   << robot_name
                                                   << ", returning nullptr");
      hardware_interface_ = nullptr;
    }
  }

  initLegController();

  // Start sitting
  control_mode_ = SIT;
  remote_heartbeat_received_time_ = std::numeric_limits<double>::max();
  last_state_time_ = std::numeric_limits<double>::max();

  // Initialize timing
  last_robot_state_msg_.header.stamp = node_->now();
  t_pub_ = node_->now();

  // Initialize state and control data structures
  double dt = 1.0 / mocap_rate_;
  filter_weight_ = 1.0 - dt / filter_time_constant_;

  // Initialize state and control strucutres
  initStateControlStructs();

  // Initialize state estimator object
  initStateEstimator();
}

void RobotDriver::initStateEstimator() {
  if (estimator_id_ == "none") {
    RCLCPP_INFO(node_->get_logger(),
                "State estimator disabled (estimator_id='none')");
    state_estimator_ = nullptr;
    return;
  } else if (estimator_id_ == "comp_filter") {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Comp Filter");
    state_estimator_ =
        std::make_shared<CompFilterEstimator>(node_, robot_ns, quadKD2_);
  } else if (estimator_id_ == "ekf_filter") {
    state_estimator_ =
        std::make_shared<EKFEstimator>(node_, robot_ns, quadKD2_);
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "Invalid estimator id '%s', returning nullptr",
                 estimator_id_.c_str());
    state_estimator_ = nullptr;
  }

  if (state_estimator_ != nullptr) {
    state_estimator_->init();
  }

  // Optional parallel "ride-along" estimator. Runs on the same sensor inputs
  // and publishes to topics.state.estimate for comparison, but its output
  // never feeds the controller.
  if (debug_estimator_id_ == "none" || debug_estimator_id_.empty()) {
    debug_state_estimator_ = nullptr;
  } else if (debug_estimator_id_ == estimator_id_) {
    RCLCPP_WARN(node_->get_logger(),
                "debug_estimator_id ('%s') matches active estimator_id; "
                "skipping ride-along.",
                debug_estimator_id_.c_str());
    debug_state_estimator_ = nullptr;
  } else if (debug_estimator_id_ == "comp_filter") {
    RCLCPP_INFO(node_->get_logger(), "Ride-along estimator: comp_filter");
    debug_state_estimator_ =
        std::make_shared<CompFilterEstimator>(node_, robot_ns, quadKD2_);
  } else if (debug_estimator_id_ == "ekf_filter") {
    RCLCPP_INFO(node_->get_logger(), "Ride-along estimator: ekf_filter");
    debug_state_estimator_ =
        std::make_shared<EKFEstimator>(node_, robot_ns, quadKD2_);
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "Invalid debug_estimator_id '%s', skipping ride-along",
                 debug_estimator_id_.c_str());
    debug_state_estimator_ = nullptr;
  }

  if (debug_state_estimator_ != nullptr) {
    debug_state_estimator_->init();
  }
}

void RobotDriver::initLegController() {
  if (controller_id_ == "inverse_dynamics") {
    leg_controller_ =
        std::make_shared<InverseDynamicsController>(node_, robot_ns, quadKD2_);
  } else if (controller_id_ == "grf_pid") {
    leg_controller_ =
        std::make_shared<GrfPidController>(node_, robot_ns, quadKD2_);
  } else if (controller_id_ == "joint") {
    leg_controller_ =
        std::make_shared<JointController>(node_, robot_ns, quadKD2_);
  } else if (controller_id_ == "underbrush") {
    leg_controller_ = std::make_shared<UnderbrushInverseDynamicsController>(
        node_, robot_ns, quadKD2_);
    double retract_vel, tau_push, tau_contact_start, tau_contact_end,
        min_switch, t_down, t_up;
    quad_utils::loadROSParam(node_, "underbrush_swing.retract_vel",
                             retract_vel);
    quad_utils::loadROSParam(node_, "underbrush_swing.tau_push", tau_push);
    quad_utils::loadROSParam(node_, "underbrush_swing.tau_contact_start",
                             tau_contact_start);
    quad_utils::loadROSParam(node_, "underbrush_swing.tau_contact_end",
                             tau_contact_end);
    quad_utils::loadROSParam(node_, "underbrush_swing.min_switch", min_switch);
    quad_utils::loadROSParam(node_, "underbrush_swing.t_down", t_down);
    quad_utils::loadROSParam(node_, "underbrush_swing.t_up", t_up);
    UnderbrushInverseDynamicsController* c =
        dynamic_cast<UnderbrushInverseDynamicsController*>(
            leg_controller_.get());
    c->setUnderbrushParams(retract_vel, tau_push, tau_contact_start,
                           tau_contact_end, min_switch, t_down, t_up);
  } else if (controller_id_ == "inertia_estimation") {
    leg_controller_ = std::make_shared<InertiaEstimationController>(
        node_, robot_ns, quadKD2_);
  } else if (controller_id_ == "learned") {
#ifdef HAS_ONNXRUNTIME
    leg_controller_ =
        std::make_shared<LearnedPolicy>(node_, robot_ns, quadKD2_);
#else
    RCLCPP_FATAL(node_->get_logger(),
                 "Learned policy requested but built without ONNX Runtime");
    leg_controller_ = nullptr;
#endif
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "Invalid controller id %s, returning nullptr",
                 controller_id_.c_str());
    leg_controller_ = nullptr;
  }
  if (leg_controller_ != nullptr && controller_id_ != "learned") {
    leg_controller_->init(stance_kp_, stance_kd_, swing_kp_, swing_kd_,
                          swing_kp_cart_, swing_kd_cart_);
  } else {
    leg_controller_->init(stance_kp_, stance_kd_, swing_kp_, swing_kd_,
                          swing_kp_cart_, swing_kd_cart_, model_path_,
                          policy_inference_rate_, stand_joint_angles_);
  }
}

void RobotDriver::initStateControlStructs() {
  vel_estimate_.setZero();
  mocap_vel_estimate_.setZero();
  imu_vel_estimate_.setZero();

  // Joint state size: 12 leg motors, plus 4 wheel motors for go2w.
  const bool has_wheels = (robot_name == "go2w");
  const int num_joints = has_wheels ? 16 : 12;
  last_joint_state_msg_.name.resize(num_joints);
  last_joint_state_msg_.position.resize(num_joints);
  last_joint_state_msg_.velocity.resize(num_joints);
  last_joint_state_msg_.effort.resize(num_joints);

  grf_array_msg_.vectors.resize(4);
  grf_array_msg_.points.resize(4);
  grf_array_msg_.contact_states.resize(4);
  grf_array_msg_.header.frame_id = "map";

  // user_tx_data layout:
  //   [0]   control_restart_flag (used by SpiritInterface; ignored by Go2)
  //   [1..] wheel commands for Go2-W: per leg (vel, kd, tau_ff)
  user_tx_data_.resize(has_wheels ? 13 : 1);
  user_tx_data_.setZero();

  cmd_vel_.setZero(6);
}

void RobotDriver::controlModeCallback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  // Wait if transitioning
  if ((control_mode_ == SIT_TO_READY) || (control_mode_ == READY_TO_SIT))
    return;
  if ((msg->data == READY) &&
      (control_mode_ == SIT)) {  // Stand if previously sitting
    control_mode_ = SIT_TO_READY;
    transition_timestamp_ = node_->now();
  } else if ((msg->data == SIT) &&
             (control_mode_ == READY)) {  // Sit if previously standing
    control_mode_ = READY_TO_SIT;
    transition_timestamp_ = node_->now();
  } else if (msg->data == SIT ||
             (msg->data == SAFETY)) {  // Allow sit or safety modes
    control_mode_ = msg->data;
  }
}

void RobotDriver::singleJointCommandCallback(
    const geometry_msgs::msg::Vector3::SharedPtr msg) {
  if (JointController* c =
          dynamic_cast<JointController*>(leg_controller_.get())) {
    c->updateSingleJointCommand(msg);
  }
}

void RobotDriver::controlRestartFlagCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  user_tx_data_[0] = (msg->data) ? 1 : 0;
}

void RobotDriver::localPlanCallback(
    const quad_msgs::msg::RobotPlan::SharedPtr msg) {
  last_local_plan_msg_ = msg;

  rclcpp::Time t_now = node_->now();
  double round_trip_time_diff =
      (t_now - last_local_plan_msg_->state_timestamp).seconds();

  leg_controller_->updateLocalPlanMsg(last_local_plan_msg_, t_now);
}

void RobotDriver::mocapCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // Collect position readings
  Eigen::Vector3d pos;
  quad_utils::pointMsgToEigen(msg->pose.position, pos);

  if (last_mocap_msg_) {
    // Record time diff between messages
    rclcpp::Time t_now = node_->now();
    double t_diff_mocap_msg = (rclcpp::Time(msg->header.stamp) -
                               rclcpp::Time(last_mocap_msg_->header.stamp))
                                  .seconds();

    // If time diff between messages < mocap dropout threshould then
    // apply filter
    if (abs(t_diff_mocap_msg - 1.0 / mocap_rate_) < mocap_dropout_threshold_) {
      if (CompFilterEstimator* c =
              dynamic_cast<CompFilterEstimator*>(state_estimator_.get())) {
        c->mocapCallBackHelper(msg, pos);
      }
      if (CompFilterEstimator* dc = dynamic_cast<CompFilterEstimator*>(
              debug_state_estimator_.get())) {
        dc->mocapCallBackHelper(msg, pos);
      }
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "Mocap time diff exceeds max dropout threshold: "
                           "t_diff=%.6f, expected=%.6f, threshold=%.6f",
                           t_diff_mocap_msg, 1.0 / mocap_rate_,
                           mocap_dropout_threshold_);
    }
  } else {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 100,
        "Mocap time diff exceeds max dropout threshold, hold the last value");
  }
  // Update our cached mocap position
  last_mocap_msg_ = msg;
}

void RobotDriver::robotStateCallback(
    const quad_msgs::msg::RobotState::SharedPtr msg) {
  last_robot_state_msg_ = *msg;
}

void RobotDriver::bodyForceEstimateCallback(
    const quad_msgs::msg::BodyForceEstimate::SharedPtr msg) {
  if (controller_id_ == "underbrush") {
    UnderbrushInverseDynamicsController* c =
        reinterpret_cast<UnderbrushInverseDynamicsController*>(
            leg_controller_.get());
    c->updateBodyForceEstimate(msg);
  }
}

void RobotDriver::remoteHeartbeatCallback(
    const std_msgs::msg::Header::SharedPtr msg) {
  // Get the current time and compare to the message time
  rclcpp::Time sent_time(msg->stamp);
  double remote_heartbeat_sent_time = sent_time.seconds();
  remote_heartbeat_received_time_ = node_->now().seconds();
  double t_latency =
      remote_heartbeat_received_time_ - remote_heartbeat_sent_time;
}

void RobotDriver::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
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
  last_cmd_vel_msg_ = *msg;
  // Record when this was last reached for safety
#ifdef HAS_ONNXRUNTIME
  if (auto c = std::dynamic_pointer_cast<LearnedPolicy>(leg_controller_)) {
    last_cmd_vel_msg_time_ = node_->now();
    c->updateCmdVelMsg(cmd_vel_, last_cmd_vel_msg_time_);
  }
#endif
}

void RobotDriver::checkMessagesForSafety() {
  // Do nothing if already in safety mode
  if (control_mode_ == SAFETY) return;

  // Check the remote heartbeat for timeout
  // (this adds extra safety if no heartbeat messages are arriving)
  if (abs(node_->now().seconds() - remote_heartbeat_received_time_) >=
          heartbeat_timeout_ &&
      remote_heartbeat_received_time_ != std::numeric_limits<double>::max()) {
    control_mode_ = SAFETY;
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Remote heartbeat lost or late to robot driver node, "
                         "entering safety mode");
  }

  // Check the state message latency
  if (!is_hardware_ &&
      abs(node_->now().seconds() - last_state_time_) >= state_timeout_ &&
      last_state_time_ != std::numeric_limits<double>::max()) {
    control_mode_ = SAFETY;
    transition_timestamp_ = node_->now();
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "State messages lost in robot driver node, entering safety mode");
  }
}

bool RobotDriver::updateState() {
  if (is_hardware_) {
    // grab data from hardware
    bool fully_populated = hardware_interface_->recv(
        last_joint_state_msg_, last_imu_msg_, user_rx_data_);

    if (!fully_populated) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "updateState returning false: recv() not fully populated");
      return false;
    }

    // Publish raw + thresholded foot contact (Unitree foot force sensor).
    // Only Unitree hardware exposes this; dynamic_cast is the cleanest way
    // to avoid leaking it into the abstract HardwareInterface API.
    if (auto* unitree =
            dynamic_cast<UnitreeInterface*>(hardware_interface_.get())) {
      quad_msgs::msg::FootContact fc_msg;
      fc_msg.header.stamp = node_->now();
      fc_msg.header.frame_id = "map";
      const auto raw = unitree->getFootForcesRaw();
      fc_msg.foot_force_raw.resize(num_feet_);
      fc_msg.contact_states.resize(num_feet_);
      for (int i = 0; i < num_feet_; ++i) {
        fc_msg.foot_force_raw[i] = raw[i];
        fc_msg.contact_states[i] = (raw[i] > foot_contact_threshold_);
      }
      foot_contact_pub_->publish(fc_msg);

      // Push the measured foot contact into the estimator(s) so the EKF uses
      // the real foot-force contact instead of the planner's expected GRFs.
      // Pushed (not subscribed) since the data is already in-process and must
      // stay aligned with this tick's IMU/joint sample.
      if (state_estimator_ != nullptr) {
        state_estimator_->loadFootContactMsg(fc_msg);
      }
      if (debug_state_estimator_ != nullptr) {
        debug_state_estimator_->loadFootContactMsg(fc_msg);
      }
    }

    // For learned controllers on hardware, populate state directly from
    // IMU + joint encoders without requiring mocap or a full state estimator.
    // The learned policy only needs orientation, angular velocity, joint
    // positions, and joint velocities — all available from onboard sensors.
    if (controller_id_ == "learned") {
      rclcpp::Time state_timestamp = node_->now();

      // Joint state from encoders
      last_robot_state_msg_.joints = last_joint_state_msg_;
      last_robot_state_msg_.joints.header.stamp = state_timestamp;

      // Orientation from IMU quaternion
      last_robot_state_msg_.body.pose.orientation = last_imu_msg_.orientation;

      // Angular velocity from IMU gyroscope
      last_robot_state_msg_.body.twist.angular = last_imu_msg_.angular_velocity;

      // Pass IMU to learned policy for acceleration access. Guarded
      // because LearnedPolicy is only declared when the workspace was
      // built with ONNX Runtime; without it, controller_id_ == "learned"
      // would have been rejected earlier in initLegController() so this
      // path is unreachable anyway.
#ifdef HAS_ONNXRUNTIME
      if (auto c = std::dynamic_pointer_cast<LearnedPolicy>(leg_controller_)) {
        c->updateImuMsg(last_imu_msg_);
      }
#endif

      // Update headers
      last_robot_state_msg_.header.stamp = state_timestamp;

      return true;
    }

    // For other controllers, use the full state estimator (requires mocap)
    state_estimator_->loadSensorMsg(last_imu_msg_, last_joint_state_msg_);

    if (last_mocap_msg_ != NULL) {
      state_estimator_->loadMocapMsg(last_mocap_msg_);
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "updateState: no mocap message received yet");
    }

    // update robot state using state estimator
    if (state_estimator_ != nullptr) {
      // READY gate for an EKF primary: the contact-aided EKF only produces a
      // valid horizontal estimate while standing (feet in contact), and
      // initializing in the folded sit pose corrupts it. So defer its
      // init/run until the robot first reaches READY (standing). Until then,
      // passthrough raw sensor state (joints + IMU orientation) so the SIT
      // controller still has joints + attitude to work with. Once started it
      // keeps running. Non-EKF estimators (comp_filter) run every tick.
      bool is_ekf_primary =
          (dynamic_cast<EKFEstimator*>(state_estimator_.get()) != nullptr);
      if (is_ekf_primary && !ekf_primary_started_ && control_mode_ != READY) {
        last_robot_state_msg_.joints = last_joint_state_msg_;
        last_robot_state_msg_.joints.header.stamp = node_->now();
        last_robot_state_msg_.body.pose.orientation = last_imu_msg_.orientation;
        last_robot_state_msg_.header.stamp = node_->now();
        return true;
      }
      if (is_ekf_primary) ekf_primary_started_ = true;

      bool result = state_estimator_->updateOnce(last_robot_state_msg_);
      if (!result) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "updateState returning false: estimator updateOnce failed");
      }

      // Run the ride-along estimator on the same sensor inputs. Its output
      // goes to debug_estimate_msg_ and is published on topics.state.estimate
      // for comparison; it never reaches the controller.
      //
      // Gate the first seed on READY (standing): if we initialize during sit
      // or the sit->stand transient, the contact-aided EKF dead-reckons
      // through legs extending / feet breaking contact and accumulates x/y
      // drift before the robot is even walking. Waiting for a settled standing
      // pose makes it start coincident with the mocap-fused comp filter.
      // Once seeded it keeps running across all modes.
      if (debug_state_estimator_ != nullptr && result &&
          (debug_estimator_seeded_ || control_mode_ == READY)) {
        bool first_seed = !debug_estimator_seeded_;
        if (!debug_estimator_seeded_) {
          // Seed from the primary's first valid output so both filters
          // initialize from the same pose (EKF reads X0 from this msg).
          debug_estimate_msg_ = last_robot_state_msg_;
          debug_estimator_seeded_ = true;
        }
        debug_state_estimator_->loadSensorMsg(last_imu_msg_,
                                              last_joint_state_msg_);
        if (last_mocap_msg_ != nullptr) {
          debug_state_estimator_->loadMocapMsg(last_mocap_msg_);
        }
        debug_state_estimator_->updateOnce(debug_estimate_msg_);

        // One-time init diagnostic: shows the seed (mocap-fused comp filter
        // pose) vs the ride-along's first published estimate, so we can see
        // whether they start coincident.
        if (first_seed) {
          RCLCPP_INFO(
              node_->get_logger(),
              "EKF ride-along init: seed(comp/mocap)=(%.3f, %.3f, %.3f) "
              "first_est=(%.3f, %.3f, %.3f)",
              last_robot_state_msg_.body.pose.position.x,
              last_robot_state_msg_.body.pose.position.y,
              last_robot_state_msg_.body.pose.position.z,
              debug_estimate_msg_.body.pose.position.x,
              debug_estimate_msg_.body.pose.position.y,
              debug_estimate_msg_.body.pose.position.z);
        }
      }

      return result;
    } else {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "updateState returning false: no state estimator initialized");
      return false;
    }
  } else {
    // State information coming through sim subscribers, not hardware interface.
    // Optionally run EKF in parallel for testing (does not affect control).
    if (debug_estimator_ && state_estimator_ != nullptr &&
        rclcpp::Time(last_robot_state_msg_.header.stamp).seconds() != 0) {
      // Initialize EKF once robot is in stand mode (control_mode_ == READY)
      if (!ekf_initialized_) {
        if (control_mode_ != READY) {
          return true;  // Not standing yet, skip EKF
        }
        ekf_estimate_msg_ = last_robot_state_msg_;
        ekf_initialized_ = true;
        RCLCPP_INFO(node_->get_logger(),
                    "EKF initialized from ground truth state (Z=%.3f)",
                    last_robot_state_msg_.body.pose.position.z);
      }

      // Build IMU msg from ground truth state
      sensor_msgs::msg::Imu imu_from_gt;
      imu_from_gt.header = last_robot_state_msg_.header;
      imu_from_gt.orientation = last_robot_state_msg_.body.pose.orientation;
      imu_from_gt.angular_velocity = last_robot_state_msg_.body.twist.angular;

      // Derive accelerometer reading from ground truth velocity.
      // A real IMU measures specific force = (linear_accel - gravity) in body
      // frame. We compute world-frame accel from finite differences, then
      // convert to what an accelerometer would read:
      //   accel_imu = R^T * (a_world - g)   but since a_world already excludes
      //   gravity in Newton's law, the IMU actually reads
      // R^T * (a_world + g_up)
      //   i.e. R^T * (dv/dt + [0,0,9.81])
      Eigen::Vector3d vel_world(last_robot_state_msg_.body.twist.linear.x,
                                last_robot_state_msg_.body.twist.linear.y,
                                last_robot_state_msg_.body.twist.linear.z);
      double dt_gt = 1.0 / update_rate_;
      Eigen::Vector3d accel_world = (vel_world - ekf_last_vel_) / dt_gt;
      ekf_last_vel_ = vel_world;

      // IMU reads specific force in body frame: R^T * (a + g)
      Eigen::Quaterniond q_orient(
          last_robot_state_msg_.body.pose.orientation.w,
          last_robot_state_msg_.body.pose.orientation.x,
          last_robot_state_msg_.body.pose.orientation.y,
          last_robot_state_msg_.body.pose.orientation.z);
      Eigen::Matrix3d R_world_body = q_orient.toRotationMatrix();
      Eigen::Vector3d g_world(0.0, 0.0, 9.81);
      Eigen::Vector3d accel_body =
          R_world_body.transpose() * (accel_world + g_world);
      imu_from_gt.linear_acceleration.x = accel_body.x();
      imu_from_gt.linear_acceleration.y = accel_body.y();
      imu_from_gt.linear_acceleration.z = accel_body.z();

      // Feed ground truth sensor data to the estimator
      state_estimator_->loadSensorMsg(imu_from_gt,
                                      last_robot_state_msg_.joints);
      state_estimator_->updateOnce(ekf_estimate_msg_);
    }
    return true;
  }
}

void RobotDriver::publishState() {
  if (is_hardware_) {
    last_joint_state_msg_.header.stamp = node_->now();
    imu_pub_->publish(last_imu_msg_);
    joint_state_pub_->publish(last_joint_state_msg_);
    robot_state_pub_->publish(last_robot_state_msg_);

    // Publish ride-along estimator output on topics.state.estimate for
    // side-by-side comparison with the active estimator (drives no control).
    if (debug_state_estimator_ != nullptr && debug_estimator_seeded_) {
      debug_estimate_msg_.header.stamp = node_->now();
      state_estimate_pub_->publish(debug_estimate_msg_);
    }
  }
}

bool RobotDriver::updateControl() {
  // Check if state machine should be skipped
  bool valid_cmd = true;

  // Check incoming messages to determine if we should enter safety mode
  checkMessagesForSafety();

  if (rclcpp::Time(last_robot_state_msg_.header.stamp).seconds() == 0) {
    return false;
  }

  if (last_robot_state_msg_.joints.position.empty()) {
    // RCLCPP_WARN(node_->get_logger(),
    //             "updateControl(): received RobotState with empty
    //             joint.position → aborting control update");
    return false;
  }

  // Define vectors for joint positions and velocities
  Eigen::VectorXd joint_positions(3 * num_feet_),
      joint_velocities(3 * num_feet_), body_state(12);

  joint_positions.setZero();
  joint_velocities.setZero();
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
    rclcpp::Time t_now = node_->now();
    rclcpp::Duration duration = t_now - transition_timestamp_;
    double t_interp = duration.seconds() / transition_duration_;
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
    rclcpp::Duration duration = node_->now() - transition_timestamp_;
    double t_interp = duration.seconds() / transition_duration_;

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
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
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
        RCLCPP_INFO(node_->get_logger(), "Triggering Soft Knee Joint Limit");
        leg_command_array_msg_.leg_commands.at(i)
            .motor_commands.at(j)
            .torque_ff = std::max(
            leg_command_array_msg_.leg_commands.at(i)
                    .motor_commands.at(j)
                    .torque_ff -
                knee_soft_ub_kd * (joint_positions(joint_idx) - knee_soft_ub),
            -torque_limits_[j]);
      }
      quad_msgs::msg::MotorCommand cmd =
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
        RCLCPP_WARN(
            node_->get_logger(),
            "Leg %d motor %d: ff effort = %5.3f Nm exceeds threshold of %5.3f "
            "Nm",
            i, j, cmd.torque_ff, torque_limits_[j]);
      }
      if (abs(effort) >= torque_limits_[j]) {
        RCLCPP_WARN(
            node_->get_logger(),
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
  leg_command_array_msg_.header.stamp = node_->now();
  leg_command_array_pub_->publish(leg_command_array_msg_);
  grf_array_msg_.header.stamp = leg_command_array_msg_.header.stamp;
  grf_pub_->publish(grf_array_msg_);
  // }
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = node_->now();
  msg.twist = last_cmd_vel_msg_;
  cmd_vel_stamped_pub_->publish(msg);
  if (debug_estimator_ && ekf_initialized_) {
    ekf_estimate_msg_.header.stamp = node_->now();
    state_estimate_pub_->publish(ekf_estimate_msg_);
  }

  // Send command to the robot
  if (is_hardware_ && is_valid) {
    rclcpp::Time t_start = node_->now();
    hardware_interface_->send(leg_command_array_msg_, user_tx_data_);
    rclcpp::Time t_end = node_->now();

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "t_diff_mb_send = %6.4f", (t_end - t_start).seconds());
  }
}

void RobotDriver::publishHeartbeat() {
  // Publish hearbeat
  rclcpp::Time time_stamp(last_robot_heartbeat_msg_.stamp);
  if ((node_->now() - last_robot_heartbeat_msg_.stamp).seconds() >=
      1.0 / publish_rate_) {
    last_robot_heartbeat_msg_.stamp = node_->now();
    robot_heartbeat_pub_->publish(last_robot_heartbeat_msg_);
  }
}

void RobotDriver::testDynamics() {
  // Pinocchio dynamics update (M, N, J) for inverse_dynamics_controller.
  // Was gated by !is_hardware_ which silently zeroed tau_ff on real hardware
  // because computeInverseDynamics asserts updated_=true and runs against
  // stale data otherwise — the SVD on a singular blk_mat returns NaN, which
  // the safety check in quad_kd2.cpp:881 zeros out. Must run on hardware too.
  if (rclcpp::Time(last_robot_state_msg_.header.stamp).seconds() != 0) {
    quad_utils::updateDynamics(*quadKD2_, last_robot_state_msg_);
  }
}

void RobotDriver::spin() {
  // Initialize timing params
  rclcpp::Rate r(update_rate_);

  // Start the mblink connection
  if (is_hardware_) {
    hardware_interface_->loadInterface(argc_, argv_);
  }

  while (rclcpp::ok()) {
    rclcpp::Time t0 = node_->now();
    // Collect new messages on subscriber topics and publish heartbeat
    rclcpp::spin_some(node_);

    // Get the newest state information
    rclcpp::Time t1 = node_->now();
    bool state_valid = updateState();

    if (!state_valid) {
      publishHeartbeat();
      r.sleep();
      continue;
    }

    rclcpp::Time t2 = node_->now();
    testDynamics();

    // Compute the leg command and publish if valid
    rclcpp::Time t3 = node_->now();
    bool is_valid = updateControl();

    rclcpp::Time t4 = node_->now();
    publishControl(is_valid);
    publishState();
    publishHeartbeat();
    r.sleep();
  }

  // Close the mblink connection
  if (is_hardware_) {
    hardware_interface_->unloadInterface();
  }
}
