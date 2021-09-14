#include "inverse_dynamics/inverse_dynamics.h"

namespace plt = matplotlibcpp;

InverseDynamics::InverseDynamics(ros::NodeHandle nh) {
	nh_ = nh;

    // Load rosparams from parameter server
  std::string grf_topic, trajectory_state_topic, robot_state_topic, local_plan_topic,
    leg_command_array_topic, control_mode_topic, leg_override_topic,
    remote_heartbeat_topic, robot_heartbeat_topic;
  spirit_utils::loadROSParam(nh_,"topics/local_plan",local_plan_topic);
  spirit_utils::loadROSParam(nh_,"topics/state/ground_truth",robot_state_topic);
  spirit_utils::loadROSParam(nh_,"topics/state/trajectory",trajectory_state_topic);
  spirit_utils::loadROSParam(nh_,"topics/heartbeat/remote",remote_heartbeat_topic);
  spirit_utils::loadROSParam(nh_,"topics/heartbeat/robot",robot_heartbeat_topic);
  spirit_utils::loadROSParam(nh_,"topics/control/grfs",grf_topic);
  spirit_utils::loadROSParam(nh_,"topics/control/joint_command",leg_command_array_topic);
  spirit_utils::loadROSParam(nh_,"topics/control/leg_override",leg_override_topic);
  spirit_utils::loadROSParam(nh_,"topics/control/mode",control_mode_topic);

  spirit_utils::loadROSParam(nh_,"inverse_dynamics/update_rate", update_rate_);
  spirit_utils::loadROSParam(nh_,"inverse_dynamics/input_timeout", input_timeout_);
  spirit_utils::loadROSParam(nh_,"inverse_dynamics/state_timeout", state_timeout_);
  spirit_utils::loadROSParam(nh_,"inverse_dynamics/heartbeat_timeout", heartbeat_timeout_);
  spirit_utils::loadROSParam(nh_, "inverse_dynamics/sit_kp", sit_kp_);
  spirit_utils::loadROSParam(nh_, "inverse_dynamics/sit_kd", sit_kd_);
  spirit_utils::loadROSParam(nh_, "inverse_dynamics/stand_kp", stand_kp_);
  spirit_utils::loadROSParam(nh_, "inverse_dynamics/stand_kd", stand_kd_);
  spirit_utils::loadROSParam(nh_, "inverse_dynamics/stance_kp", stance_kp_);
  spirit_utils::loadROSParam(nh_, "inverse_dynamics/stance_kd", stance_kd_);
  spirit_utils::loadROSParam(nh_, "inverse_dynamics/swing_kp", swing_kp_);
  spirit_utils::loadROSParam(nh_, "inverse_dynamics/swing_kd", swing_kd_);
  spirit_utils::loadROSParam(nh_, "inverse_dynamics/safety_kp", safety_kp_);
  spirit_utils::loadROSParam(nh_, "inverse_dynamics/safety_kd", safety_kd_);
  spirit_utils::loadROSParam(nh_, "inverse_dynamics/underbrush_swing_kp", underbrush_swing_kp_);
  spirit_utils::loadROSParam(nh_, "inverse_dynamics/underbrush_swing_kd", underbrush_swing_kd_);
  spirit_utils::loadROSParam(nh_, "inverse_dynamics/remote_latency_threshold_warn",
    remote_latency_threshold_warn_);
  spirit_utils::loadROSParam(nh_, "inverse_dynamics/remote_latency_threshold_error",
    remote_latency_threshold_error_);

  spirit_utils::loadROSParam(nh_,"local_planner/timestep", dt_);

  // Setup pubs and subs
  local_plan_sub_ = nh_.subscribe(local_plan_topic,1,&InverseDynamics::localPlanCallback, this);
  robot_state_sub_= nh_.subscribe(robot_state_topic,1,&InverseDynamics::robotStateCallback, this);
  grf_sub_ = nh_.subscribe(grf_topic,1,&InverseDynamics::grfInputCallback, this);
  trajectory_state_sub_ = nh_.subscribe(
    trajectory_state_topic,1,&InverseDynamics::trajectoryStateCallback, this);
  control_mode_sub_ = nh_.subscribe(
    control_mode_topic,1,&InverseDynamics::controlModeCallback, this);
  leg_override_sub_ = nh_.subscribe(
    leg_override_topic,1,&InverseDynamics::legOverrideCallback, this);
  remote_heartbeat_sub_ = nh_.subscribe(
    remote_heartbeat_topic,1,&InverseDynamics::remoteHeartbeatCallback, this);
  leg_command_array_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>(leg_command_array_topic,1);
  grf_pub_ = nh_.advertise<spirit_msgs::GRFArray>(grf_topic,1);
  robot_heartbeat_pub_ = nh_.advertise<std_msgs::Header>(robot_heartbeat_topic,1);

  // Start sitting
  control_mode_ = SIT;
  last_remote_heartbeat_time_ = std::numeric_limits<double>::max();
  last_state_time_ = std::numeric_limits<double>::max();  

  kinematics_ = std::make_shared<spirit_utils::SpiritKinematics>();

  underbrush_swing_ = true;//false;
}

void InverseDynamics::controlModeCallback(const std_msgs::UInt8::ConstPtr& msg) {
  
  // Wait if transitioning
  if ((control_mode_ == SIT_TO_STAND) || (control_mode_ == STAND_TO_SIT))
    return;

  if ((msg->data == STAND) && (control_mode_ == SIT)) { // Stand if previously sitting

    control_mode_ = SIT_TO_STAND;
    transition_timestamp_ = ros::Time::now();

  } else if ((msg->data == SIT) && (control_mode_ == STAND)) { // Sit if previously standing

    control_mode_ = STAND_TO_SIT;
    transition_timestamp_ = ros::Time::now();

  } else if (msg->data == SIT || (msg->data == SAFETY)) { // Allow sit or safety modes
    
    control_mode_ = msg->data;
  }
}

void InverseDynamics::localPlanCallback(const spirit_msgs::RobotPlan::ConstPtr& msg) {
  last_local_plan_msg_ = msg;

  double round_trip_time_diff = (ros::Time::now() - last_local_plan_msg_->state_timestamp).toSec();
  ROS_INFO_STREAM("round trip time difference: " << round_trip_time_diff);

  double local_plan_time_diff = (ros::Time::now() - last_local_plan_msg_->header.stamp).toSec();
  ROS_INFO_STREAM("local plan time difference: " << local_plan_time_diff);
}

void InverseDynamics::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {
  // ROS_INFO("In robotStateCallback");
  last_robot_state_msg_ = msg;
  last_state_time_ = msg->header.stamp.toSec();
}

void InverseDynamics::grfInputCallback(const spirit_msgs::GRFArray::ConstPtr& msg) {
  // ROS_INFO("In controlInputCallback");
  last_grf_array_msg_ = msg;
}

void InverseDynamics::trajectoryStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {
  // ROS_INFO("In footPlanContinuousCallback");
  last_trajectory_state_msg_ = msg;
}

void InverseDynamics::legOverrideCallback(const spirit_msgs::LegOverride::ConstPtr& msg) {
  last_leg_override_msg_ = *msg;
}

void InverseDynamics::remoteHeartbeatCallback(const std_msgs::Header::ConstPtr& msg) {

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

void InverseDynamics::checkMessages() {

  // Do nothing if already in safety mode
  if (control_mode_ == SAFETY)
    return;

  // Check the remote heartbeat for timeout
  // (this adds extra safety if no heartbeat messages are arriving)
  if (abs(ros::Time::now().toSec() - last_remote_heartbeat_time_) >= heartbeat_timeout_ && last_remote_heartbeat_time_ != std::numeric_limits<double>::max())
  {
    control_mode_ = SAFETY;
    ROS_WARN_THROTTLE(1,"Remote heartbeat lost or late to ID node, entering safety mode");
  }

  // Check the state message latency
  if (abs(ros::Time::now().toSec() - last_state_time_) >= state_timeout_ && last_state_time_ != std::numeric_limits<double>::max())
  {
    control_mode_ = SAFETY;
    transition_timestamp_ = ros::Time::now();
    ROS_WARN_THROTTLE(1,"State messages lost in ID node, entering safety mode");
  }

}

void InverseDynamics::publishLegCommandArray() {

  // Define static position setpoints and gains
  static const std::vector<double> stand_joint_angles_{0,0.76,2*0.76};
  static const std::vector<double> sit_joint_angles_{0.0,0.0,0.0};

  // Define vectors for joint positions and velocities
  Eigen::VectorXd joint_positions(3*num_feet_), joint_velocities(3*num_feet_), body_state(12);
  spirit_utils::vectorToEigen(last_robot_state_msg_->joints.position, joint_positions);
  spirit_utils::vectorToEigen(last_robot_state_msg_->joints.velocity, joint_velocities);
  body_state = spirit_utils::bodyStateMsgToEigen(last_robot_state_msg_->body);

  // Define vectors for state positions and velocities 
  Eigen::VectorXd state_positions(3*num_feet_+6), state_velocities(3*num_feet_+6);
  state_positions << joint_positions, body_state.head(6);
  state_velocities << joint_velocities, body_state.tail(6);

  // Compute jacobians
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3*num_feet_, state_velocities.size());
  kinematics_->getJacobianBodyAngVel(state_positions,jacobian);

  // Initialize variables for ff and fb
  spirit_msgs::RobotState ref_state_msg;
  spirit_msgs::GRFArray grf_array_msg;
  Eigen::VectorXd tau_array(3*num_feet_), tau_swing_leg_array(3*num_feet_);
  leg_command_array_msg_.leg_commands.resize(num_feet_);

  spirit_msgs::RobotState ref_underbrush_msg;

  // Set the input handling based on what data we've recieved, prioritizing local plan over grf
  // JN - removed GRFS option, mpc_controller would need updating to publish local plans now
  int input_type;
  if (last_local_plan_msg_ != NULL && 
    (ros::Time::now() - last_local_plan_msg_->header.stamp).toSec() < input_timeout_) {
    
    input_type = LOCAL_PLAN;
  } else {
    input_type = NONE;
  }

  // Get reference state and grf from local plan or traj + grf messages
  if (input_type == LOCAL_PLAN) {

    double t_now = ros::Time::now().toSec();
    
    if ( (t_now < last_local_plan_msg_->states.front().header.stamp.toSec()) || 
         (t_now > last_local_plan_msg_->states.back().header.stamp.toSec()) ) {
      ROS_ERROR("ID node couldn't find the correct ref state!");
    }    

    // Interpolate the local plan to get the reference state and ff GRF
    for (int i = 0; i < last_local_plan_msg_->states.size()-1; i++) {
      
      if ( (t_now >= last_local_plan_msg_->states[i].header.stamp.toSec()) && 
          ( t_now <  last_local_plan_msg_->states[i+1].header.stamp.toSec() )) {

        double t_interp = (t_now - last_local_plan_msg_->states[i].header.stamp.toSec())/
          (last_local_plan_msg_->states[i+1].header.stamp.toSec() - 
           last_local_plan_msg_->states[i].header.stamp.toSec());
        
        spirit_utils::interpRobotState(last_local_plan_msg_->states[i],
          last_local_plan_msg_->states[i+1], t_interp, ref_state_msg);

        // spirit_utils::interpGRFArray(last_local_plan_msg_->grfs[i],
        //   last_local_plan_msg_->grfs[i+1], t_interp, grf_array_msg);

        // ref_state_msg = last_local_plan_msg_->states[i];
        grf_array_msg = last_local_plan_msg_->grfs[i];

        break;
      }
    }

    // Underbrush swing leg motion
    ref_underbrush_msg = ref_state_msg;
    if (underbrush_swing_) {
      for (int i = 0; i < 4; i++) {
        if (!ref_state_msg.feet.feet.at(i).contact){
          for (int j = 0; j < last_local_plan_msg_->states.size()-1; j++) {
            if (t_now < last_local_plan_msg_->states[j].header.stamp.toSec() &&
                bool(last_local_plan_msg_->states[j].feet.feet.at(i).contact)) {
              ref_underbrush_msg.feet.feet.at(i).position.x = last_local_plan_msg_->states[j].feet.feet.at(i).position.x;
              ref_underbrush_msg.feet.feet.at(i).position.y = last_local_plan_msg_->states[j].feet.feet.at(i).position.y;
              ref_underbrush_msg.feet.feet.at(i).position.z = last_local_plan_msg_->states[j].feet.feet.at(i).position.z;
              ref_underbrush_msg.feet.feet.at(i).velocity.x = 0;
              ref_underbrush_msg.feet.feet.at(i).velocity.y = 0;
              ref_underbrush_msg.feet.feet.at(i).velocity.z = 0;
              ref_underbrush_msg.feet.feet.at(i).acceleration.x = 0;
              ref_underbrush_msg.feet.feet.at(i).acceleration.y = 0;
              ref_underbrush_msg.feet.feet.at(i).acceleration.z = 0;

              break;
            }
          }
        }
      }
      spirit_utils::ikRobotState(*kinematics_, ref_underbrush_msg);
      for (int i = 0; i < 4; i++) {
        int abad_idx = 3*i+0;
        int hip_idx = 3*i+1;
        int knee_idx = 3*i+2;
        ref_underbrush_msg.joints.position.at(knee_idx) +=
          -0.5*std::abs(state_positions[hip_idx] - ref_underbrush_msg.joints.position.at(hip_idx))
          -0.5*std::abs(state_positions[abad_idx] - ref_underbrush_msg.joints.position.at(abad_idx));
      }
      ref_state_msg = ref_underbrush_msg;
      //std::cout << ref_state_msg.feet.feet.at(1).position.x;
      //std::cout << " " << (ref_state_msg.feet.feet.at(1).contact?"y":"n") << std::endl;
    }

  } else if (input_type == GRFS) {
    ref_state_msg = *(last_trajectory_state_msg_);
    grf_array_msg = *(last_grf_array_msg_);

    Eigen::VectorXd grf_array(3*num_feet_);
    grf_array = spirit_utils::grfArrayMsgToEigen(grf_array_msg);
  }

  // Load feedforward torques if provided
  if (input_type != NONE) {

    // Declare plan and state data as Eigen vectors
    Eigen::VectorXd ref_body_state(12), grf_array(3 * num_feet_),
        ref_foot_positions(3 * num_feet_), ref_foot_velocities(3 * num_feet_), ref_foot_acceleration(3 * num_feet_);

    // Load plan and state data from messages
    ref_body_state = spirit_utils::bodyStateMsgToEigen(ref_state_msg.body);
    spirit_utils::multiFootStateMsgToEigen(
        ref_state_msg.feet, ref_foot_positions, ref_foot_velocities, ref_foot_acceleration);
    grf_array = spirit_utils::grfArrayMsgToEigen(grf_array_msg);

    tau_array = -jacobian.transpose().block<12,12>(0,0)*grf_array;

    kinematics_->compInvDyn(state_positions, state_velocities, ref_foot_acceleration, grf_array, tau_swing_leg_array);
  } else {
    grf_array_msg.header.stamp = ros::Time::now();
    grf_array_msg.points.resize(num_feet_);
    grf_array_msg.vectors.resize(num_feet_);
  }

  // Enter state machine for filling motor command message
  if (control_mode_ == SAFETY)
  { 
    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        int joint_idx = 3*i+j;
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
        int joint_idx = 3*i+j;
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
          sit_joint_angles_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kp = sit_kp_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kd = sit_kd_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
      }
    }
  } 
  else if (control_mode_ == STAND)
  {
    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {

        int joint_idx = 3*i+j;

        if (input_type != NONE) {
          leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
            ref_state_msg.joints.position.at(joint_idx);
          leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 
            ref_state_msg.joints.velocity.at(joint_idx);

          // Contact should be boolean, but it seems to need to be converted
          if (bool(ref_state_msg.feet.feet.at(i).contact))
          {
            leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kp = stance_kp_.at(j);
            leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kd = stance_kd_.at(j);

            leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff =
                tau_array(joint_idx);
          }
          else
          {
            if (underbrush_swing_) {
              leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kp = underbrush_swing_kp_.at(j);
              leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kd = underbrush_swing_kd_.at(j);

              leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
            } else {
              leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kp = swing_kp_.at(j);
              leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kd = swing_kd_.at(j);

              leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff =
                  tau_swing_leg_array(joint_idx);
            }
          }
        } else {
          leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
            stand_joint_angles_.at(j);
          leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kp = stand_kp_.at(j);
          leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kd = stand_kd_.at(j);
          leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
        }
      }
    }
    int n_override = last_leg_override_msg_.leg_index.size();
    int leg_ind;
    spirit_msgs::MotorCommand motor_command;
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
  else if (control_mode_ == SIT_TO_STAND)
  {
    ros::Duration duration = ros::Time::now() - transition_timestamp_;
    double t_interp = duration.toSec()/transition_duration_;

    if (t_interp >= 1) {
      control_mode_ = STAND;
      return;
    }

    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
          (stand_joint_angles_.at(j) - sit_joint_angles_.at(j))*t_interp + 
          sit_joint_angles_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kp = stand_kp_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).kd = stand_kd_.at(j);
        leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
      }
    }
  }
  else if (control_mode_ == STAND_TO_SIT)
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
      spirit_msgs::MotorCommand cmd =leg_command_array_msg_.leg_commands.at(i).motor_commands.at(j);
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

  // Stamp and send the message
  leg_command_array_msg_.header.stamp = ros::Time::now();
  leg_command_array_pub_.publish(leg_command_array_msg_);
  grf_pub_.publish(grf_array_msg);
}

void InverseDynamics::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {

    // Publish hearbeat
    std_msgs::Header msg;
    msg.stamp = ros::Time::now();
    robot_heartbeat_pub_.publish(msg);

    // Collect new messages on subscriber topics
    ros::spinOnce();

    // Wait until we have our first state messages
    if (last_robot_state_msg_ != NULL)
    {
      // Check that messages are still fresh
      checkMessages();

      // Compute and publish control input data
      publishLegCommandArray();
    }

    // Enforce update rate
    r.sleep();
  }
}
