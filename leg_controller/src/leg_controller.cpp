#include "leg_controller/leg_controller.h"

namespace plt = matplotlibcpp;

LegController::LegController(ros::NodeHandle nh) {
	nh_ = nh;

    // Load rosparams from parameter server
  std::string grf_topic, trajectory_state_topic, robot_state_topic, local_plan_topic,
    leg_command_array_topic, control_mode_topic, leg_override_topic,
    remote_heartbeat_topic, robot_heartbeat_topic, single_joint_cmd_topic;
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

  quad_utils::loadROSParam(nh_,"leg_controller/controller", controller_id_);
  quad_utils::loadROSParam(nh_,"leg_controller/update_rate", update_rate_);
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
  local_plan_sub_ = nh_.subscribe(local_plan_topic,1,&LegController::localPlanCallback, this);
  robot_state_sub_= nh_.subscribe(robot_state_topic,1,&LegController::robotStateCallback, this);
  grf_sub_ = nh_.subscribe(grf_topic,1,&LegController::grfInputCallback, this);
  trajectory_state_sub_ = nh_.subscribe(
    trajectory_state_topic,1,&LegController::trajectoryStateCallback, this);
  control_mode_sub_ = nh_.subscribe(
    control_mode_topic,1,&LegController::controlModeCallback, this);
  single_joint_cmd_sub_ = nh_.subscribe(
    single_joint_cmd_topic,1,&LegController::singleJointCommandCallback, this);
  leg_override_sub_ = nh_.subscribe(
    leg_override_topic,1,&LegController::legOverrideCallback, this);
  remote_heartbeat_sub_ = nh_.subscribe(
    remote_heartbeat_topic,1,&LegController::remoteHeartbeatCallback, this);
  leg_command_array_pub_ = nh_.advertise<quad_msgs::LegCommandArray>(leg_command_array_topic,1);
  grf_pub_ = nh_.advertise<quad_msgs::GRFArray>(grf_topic,1);
  robot_heartbeat_pub_ = nh_.advertise<std_msgs::Header>(robot_heartbeat_topic,1);

  // Start sitting
  control_mode_ = SIT;
  last_remote_heartbeat_time_ = std::numeric_limits<double>::max();
  last_state_time_ = std::numeric_limits<double>::max();

  // Initialize kinematics object
  quadKD_ = std::make_shared<quad_utils::QuadKD>();

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
  for (int i = 0; i < num_feet_; i++) {
    geometry_msgs::Vector3 vec;
    geometry_msgs::Point point;
    bool contact_state = true;
    grf_array_msg_.vectors[i] = vec;
    grf_array_msg_.points[i] = point;
    grf_array_msg_.contact_states[i] = contact_state;
  }
}

void LegController::controlModeCallback(const std_msgs::UInt8::ConstPtr& msg) {
  
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

void LegController::singleJointCommandCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
  // ROS_INFO("In controlInputCallback");
  if (JointController* c = dynamic_cast<JointController*>(leg_controller_.get())) {
    c->updateSingleJointCommand(msg);
  }
}

void LegController::localPlanCallback(const quad_msgs::RobotPlan::ConstPtr& msg) {
  last_local_plan_msg_ = msg;

  double round_trip_time_diff = (ros::Time::now() - last_local_plan_msg_->state_timestamp).toSec();
  ROS_INFO_STREAM("round trip time difference: " << round_trip_time_diff);

  double local_plan_time_diff = (ros::Time::now() - last_local_plan_msg_->header.stamp).toSec();
  ROS_INFO_STREAM("local plan time difference: " << local_plan_time_diff);

  if (InverseDynamicsController* c = dynamic_cast<InverseDynamicsController*>(leg_controller_.get())) {
    c->updateLocalPlanMsg(last_local_plan_msg_);
  }
}

void LegController::robotStateCallback(const quad_msgs::RobotState::ConstPtr& msg) {
  // ROS_INFO("In robotStateCallback");
  // if (last_robot_state_msg_ != NULL) {
  //   first_robot_state_msg_ = msg;
  // }

  // last_robot_state_msg_ = msg;
  // last_state_time_ = msg->header.stamp.toSec();

}

void LegController::grfInputCallback(const quad_msgs::GRFArray::ConstPtr& msg) {
  // ROS_INFO("In controlInputCallback");
  // last_grf_array_msg_ = msg;
}

void LegController::trajectoryStateCallback(const quad_msgs::RobotState::ConstPtr& msg) {
  // ROS_INFO("In footPlanContinuousCallback");
  last_trajectory_state_msg_ = msg;
}

void LegController::legOverrideCallback(const quad_msgs::LegOverride::ConstPtr& msg) {
  last_leg_override_msg_ = *msg;
}

void LegController::remoteHeartbeatCallback(const std_msgs::Header::ConstPtr& msg) {

  // Get the current time and compare to the message time
  last_remote_heartbeat_time_ = msg->stamp.toSec();
  double t_now = ros::Time::now().toSec();
  double t_latency = t_now - last_remote_heartbeat_time_;

  ROS_INFO_THROTTLE(1.0,"Remote latency = %6.4fs", t_latency);

  if (abs(t_latency) >= remote_latency_threshold_warn_) {
    ROS_WARN_THROTTLE(1.0,"Remote latency = %6.4fs which exceeds the warning threshold of %6.4fs\n",
      t_latency, remote_latency_threshold_warn_);
  }

  if (abs(t_latency) >= remote_latency_threshold_error_) {
    ROS_WARN_THROTTLE(1.0,"Remote latency = %6.4fs which exceeds the maximum threshold of %6.4fs, "
      "entering safety mode\n", t_latency, remote_latency_threshold_error_);
    control_mode_ = SAFETY;
  }
}

void LegController::checkMessages() {

  // Do nothing if already in safety mode
  if (control_mode_ == SAFETY)
    return;

  // Check the remote heartbeat for timeout
  // (this adds extra safety if no heartbeat messages are arriving)
  if (abs(ros::Time::now().toSec() - last_remote_heartbeat_time_) >= heartbeat_timeout_ && 
    last_remote_heartbeat_time_ != std::numeric_limits<double>::max())
  {
    control_mode_ = SAFETY;
    ROS_WARN_THROTTLE(1,"Remote heartbeat lost or late to ID node, entering safety mode");
  }

  // Check the state message latency
  if (abs(ros::Time::now().toSec() - last_state_time_) >= state_timeout_ && 
    last_state_time_ != std::numeric_limits<double>::max())
  {
    control_mode_ = SAFETY;
    transition_timestamp_ = ros::Time::now();
    ROS_WARN_THROTTLE(1,"State messages lost in ID node, entering safety mode");
  }

}

void LegController::executeCustomController() {

  if (GrfPidController* c = dynamic_cast<GrfPidController*>(leg_controller_.get())) {
    c->setDesiredState(first_robot_state_msg_);
  }

  if (leg_controller_->computeLegCommandArray(last_robot_state_msg_,
      leg_command_array_msg_, grf_array_msg_) == false) {

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
  
  // grf_pid_controller_->setDesiredState(first_robot_state_msg_);
  // grf_pid_controller_->computeLegCommandArray(last_robot_state_msg_,
  //     leg_command_array_msg_, grf_array_msg_);

  // if (last_local_plan_msg_ != NULL && 
  //   (ros::Time::now() - last_local_plan_msg_->header.stamp).toSec() < input_timeout_) {

  //   inverse_dynamics_controller_->computeLegCommandArray(last_robot_state_msg_,
  //     leg_command_array_msg_, grf_array_msg_);

  // } else {
  //   for (int i = 0; i < num_feet_; ++i) {
  //     leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);
  //     for (int j = 0; j < 3; ++j) {
  //       leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
  //         stand_joint_angles_.at(j);
  //       leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
  //       leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kp = stand_kp_.at(j);
  //       leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kd = stand_kd_.at(j);
  //       leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
  //     }
  //   }
  // }
}

void LegController::computeLegCommandArray() {

  // Define vectors for joint positions and velocities
  Eigen::VectorXd joint_positions(3*num_feet_), joint_velocities(3*num_feet_), body_state(12);
  quad_utils::vectorToEigen(last_robot_state_msg_->joints.position, joint_positions);
  quad_utils::vectorToEigen(last_robot_state_msg_->joints.velocity, joint_velocities);

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
      return;
    }

    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        double ang = stand_joint_angles_.at(j);
        // if (j == 1) {
        //   if (i == 0 || i == 2) {
        //     ang += 0.6;
        //   } else {
        //     ang -= 0.6;
        //   }
        // }
        // if (j == 0) {
        //   if (i == 0 || i == 1) {
        //     ang += 0.5;
        //   } else {
        //     ang -= 0.5;
        //   }
        // }
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
      return;
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
      return;
  }

  for (int i = 0; i < num_feet_; ++i) {
    for (int j = 0; j < 3; ++j) {
      int joint_idx = 3*i+j;
      quad_msgs::MotorCommand cmd =leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j);
      double pos_component = cmd.kp*(cmd.pos_setpoint - joint_positions[joint_idx]);
      double vel_component = cmd.kd*(cmd.vel_setpoint - joint_velocities[joint_idx]);
      double fb_component = pos_component + vel_component;
      double effort = fb_component + cmd.torque_ff;
      double fb_ratio = abs(fb_component)/(abs(fb_component) + abs(cmd.torque_ff));

      double effort_threshold = 30.0;
      if (abs(cmd.torque_ff) >= effort_threshold) {
        ROS_WARN("Leg %d motor %d: ff effort = %5.3f Nm exceeds threshold of %5.3f Nm", i,j,cmd.torque_ff, effort_threshold);
      }      
      if (abs(effort) >= effort_threshold) {
        ROS_WARN("Leg %d motor %d: total effort = %5.3f Nm exceeds threshold of %5.3f Nm", i,j,effort, effort_threshold);
      }

      leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).pos_component = pos_component;
      leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).vel_component = vel_component;
      leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).fb_component = fb_component;
      leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).effort = effort;
      leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).fb_ratio = fb_ratio;
    }
  }
}

void LegController::publishHeartbeat() {

  // Publish hearbeat
  std_msgs::Header msg;
  msg.stamp = ros::Time::now();
  robot_heartbeat_pub_.publish(msg);
}

void LegController::publishLegCommandArray() {

  // Stamp and send the message
  leg_command_array_msg_.header.stamp = ros::Time::now();
  grf_array_msg_.header.stamp = leg_command_array_msg_.header.stamp;
  leg_command_array_pub_.publish(leg_command_array_msg_);
  // grf_pub_.publish(grf_array_msg_);
}

void LegController::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {

    // Collect new messages on subscriber topics and publish heartbeat
    ros::spinOnce();
    publishHeartbeat();

    // // Wait until we have our first state messages
    // if (last_robot_state_msg_ != NULL)
    // {
    //   // Check that messages are still fresh
    //   checkMessages();

    //   // Compute and publish control input data
    //   computeLegCommandArray();
    //   publishLegCommandArray();
    // }
    if (JointController* c = dynamic_cast<JointController*>(leg_controller_.get())) {
      c->computeLegCommandArray(last_robot_state_msg_, leg_command_array_msg_, grf_array_msg_);
      publishLegCommandArray();
    }

    // Enforce update rate
    r.sleep();
  }
}
