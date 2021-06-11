#include "inverse_dynamics/inverse_dynamics.h"

namespace plt = matplotlibcpp;

InverseDynamics::InverseDynamics(ros::NodeHandle nh) {
	nh_ = nh;

    // Load rosparams from parameter server
  std::string grf_input_topic, trajectory_state_topic, robot_state_topic, local_plan_topic,
    leg_command_array_topic, control_mode_topic, leg_override_topic; 
  spirit_utils::loadROSParam(nh_,"topics/local_plan",local_plan_topic);
  spirit_utils::loadROSParam(nh_,"topics/state/ground_truth",robot_state_topic);
  spirit_utils::loadROSParam(nh_,"topics/state/trajectory",trajectory_state_topic);
  spirit_utils::loadROSParam(nh_,"topics/control/grfs",grf_input_topic);
  spirit_utils::loadROSParam(nh_,"topics/control/joint_command",leg_command_array_topic);
  spirit_utils::loadROSParam(nh_,"topics/control/leg_override",leg_override_topic);
  spirit_utils::loadROSParam(nh_,"topics/control/mode",control_mode_topic);
  spirit_utils::loadROSParam(nh_,"inverse_dynamics/update_rate", update_rate_);
  spirit_utils::loadROSParam(nh_,"local_planner/timestep", dt_);

  // Setup pubs and subs
  local_plan_sub_ = nh_.subscribe(local_plan_topic,1,&InverseDynamics::localPlanCallback, this);
  robot_state_sub_= nh_.subscribe(robot_state_topic,1,&InverseDynamics::robotStateCallback, this);
  grf_input_sub_ = nh_.subscribe(grf_input_topic,1,&InverseDynamics::grfInputCallback, this);
  trajectory_state_sub_ = nh_.subscribe(trajectory_state_topic,1,&InverseDynamics::trajectoryStateCallback, this);
  control_mode_sub_ = nh_.subscribe(control_mode_topic,1,&InverseDynamics::controlModeCallback, this);
  leg_override_sub_ = nh_.subscribe(leg_override_topic,1,&InverseDynamics::legOverrideCallback, this);
  leg_command_array_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>(leg_command_array_topic,1);

  // Start sitting
  control_mode_ = SIT;

  step_number = 0;
}

void InverseDynamics::controlModeCallback(const std_msgs::UInt8::ConstPtr& msg) {
  
  if ((control_mode_ == SIT_TO_STAND) || (control_mode_ == STAND_TO_SIT))
    return;

  if ((msg->data == STAND) && (control_mode_ == SIT)) {

    control_mode_ = SIT_TO_STAND;
    transition_timestamp_ = ros::Time::now();

  } else if ((msg->data == SIT) && (control_mode_ == STAND)) {

    control_mode_ = STAND_TO_SIT;
    transition_timestamp_ = ros::Time::now();

  } else if (msg->data == SIT || msg->data == STAND) {
    
    control_mode_ = msg->data;
  }
}

void InverseDynamics::localPlanCallback(const spirit_msgs::RobotPlan::ConstPtr& msg) {
  last_local_plan_msg_ = msg;
}

void InverseDynamics::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {
  // ROS_INFO("In robotStateCallback");
  last_robot_state_msg_ = msg;
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

void InverseDynamics::publishLegCommandArray() {

  if (last_robot_state_msg_ == NULL)
    return;

  // Set the input handling based on what data we've recieved, prioritizing local plan over grf
  int input_type;
  if (last_local_plan_msg_ != NULL) {
    input_type = LOCAL_PLAN;
  } else if (last_grf_array_msg_ != NULL){
    input_type = GRFS;
  } else {
    input_type = NONE;
  }

  spirit_msgs::LegCommandArray msg;
  msg.leg_commands.resize(num_feet_);

  static const int testingValue = 0;

  static const std::vector<double> stand_joint_angles_{0,0.76,2*0.76};
  static const std::vector<double> sit_joint_angles_{0.0,0.0,0.0};
  static const std::vector<double> stand_kp_{50,50,50};
  static const std::vector<double> stand_kd_{1,1,1};

  static const std::vector<double> ID_kp_{10,10,10};
  static const std::vector<double> ID_kd_{0.2,0.2,0.2};

  static const std::vector<double> walk_kp_{25,25,25};  // 50
  static const std::vector<double> walk_kd_{0.5,0.5,0.5};     // 1

  // Initialize for plan interpolation
  spirit_msgs::RobotState ref_state_msg;
  spirit_msgs::GRFArray grf_array_msg;

  if (input_type == LOCAL_PLAN) {
    double current_time = spirit_utils::getDurationSinceTime(last_local_plan_msg_->global_plan_timestamp);
    int current_plan_index = spirit_utils::getPlanIndex(last_local_plan_msg_->global_plan_timestamp, dt_);
    double t_interp = std::fmod(current_time,dt_)/dt_;

    if ((current_plan_index < last_local_plan_msg_->plan_indices.front()) || 
        (current_plan_index > last_local_plan_msg_->plan_indices.back()) ) {
      ROS_ERROR("ID node couldn't find the correct ref state!");
    }

    // Interpolate the local plan to get the reference state and ff GRF
    for (int i = 0; i < last_local_plan_msg_->states.size()-1; i++) {
      if ((current_plan_index >= last_local_plan_msg_->plan_indices[i]) && 
          (current_plan_index <  last_local_plan_msg_->plan_indices[i+1])) {
        
        spirit_utils::interpRobotState(last_local_plan_msg_->states[i],
          last_local_plan_msg_->states[i+1], t_interp, ref_state_msg);

        spirit_utils::interpGRFArray(last_local_plan_msg_->grfs[i],
          last_local_plan_msg_->grfs[i+1], t_interp, grf_array_msg);

        break;
      }
    }
  } else if (input_type == GRFS) {
    ref_state_msg = *(last_trajectory_state_msg_);
    grf_array_msg = *(last_grf_array_msg_);

    Eigen::VectorXd grf_array(3*num_feet_);
    grf_array = spirit_utils::grfArrayMsgToEigen(grf_array_msg);
  }

  Eigen::MatrixXd tau_array;
  if (input_type != NONE) {

    // Declare plan and state data as Eigen vectors
    Eigen::VectorXd ref_body_state(12), body_state(12), grf_array(3*num_feet_),
      ref_foot_positions(3*num_feet_), ref_foot_velocities(3*num_feet_);

    // Load plan and state data from messages
    body_state = spirit_utils::odomMsgToEigen(last_robot_state_msg_->body);
    ref_body_state = spirit_utils::odomMsgToEigen(ref_state_msg.body);
    spirit_utils::multiFootStateMsgToEigen(
      ref_state_msg.feet, ref_foot_positions, ref_foot_velocities);
    grf_array = spirit_utils::grfArrayMsgToEigen(grf_array_msg);

    // Define vectors for joint positions and velocities
    Eigen::VectorXd joint_positions(3*num_feet_), joint_velocities(3*num_feet_);
    spirit_utils::vectorToEigen(last_robot_state_msg_->joints.position, joint_positions);
    spirit_utils::vectorToEigen(last_robot_state_msg_->joints.velocity, joint_velocities);

    // Define vectors for state positions and velocities 
    Eigen::VectorXd state_positions(3*num_feet_+6), state_velocities(3*num_feet_+6);
    state_positions << joint_positions, body_state.head(6);
    state_velocities << joint_velocities, body_state.tail(6);

    // Compute joint torques
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3*num_feet_, state_velocities.size());
    spirit_utils::getJacobian(state_positions,jacobian);
    tau_array = -jacobian.transpose().block<12,12>(0,0)*grf_array;
  }

  // Enter state machine for filling motor command message
  if (control_mode_ == SIT)
  { 
    for (int i = 0; i < num_feet_; ++i) {
      msg.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = sit_joint_angles_.at(j);
        msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
        msg.leg_commands.at(i).motor_commands.at(j).kp = stand_kp_.at(j);
        msg.leg_commands.at(i).motor_commands.at(j).kd = stand_kd_.at(j);
        msg.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
      }
    }
  } 
  else if (control_mode_ == STAND)
  {
    for (int i = 0; i < num_feet_; ++i) {
      msg.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {

        int joint_idx = 3*i+j;

        if (input_type != NONE) {
          msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = ref_state_msg.joints.position.at(joint_idx);
          msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0; // Just need kinematics::legIKVel to do this
          msg.leg_commands.at(i).motor_commands.at(j).kp = walk_kp_.at(j);
          msg.leg_commands.at(i).motor_commands.at(j).kd = walk_kd_.at(j);
          msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tau_array(joint_idx);
        } else {
          msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
          msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          msg.leg_commands.at(i).motor_commands.at(j).kp = stand_kp_.at(j);
          msg.leg_commands.at(i).motor_commands.at(j).kd = stand_kd_.at(j);
          msg.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
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
          msg.leg_commands.at(leg_ind).motor_commands.at(j).pos_setpoint = motor_command.pos_setpoint;
          msg.leg_commands.at(leg_ind).motor_commands.at(j).vel_setpoint = motor_command.vel_setpoint;
          msg.leg_commands.at(leg_ind).motor_commands.at(j).kp = motor_command.kp;
          msg.leg_commands.at(leg_ind).motor_commands.at(j).kd = motor_command.kd;
          msg.leg_commands.at(leg_ind).motor_commands.at(j).torque_ff = motor_command.torque_ff;
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
      msg.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
          (stand_joint_angles_.at(j) - sit_joint_angles_.at(j))*t_interp + 
          sit_joint_angles_.at(j);
        msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
        msg.leg_commands.at(i).motor_commands.at(j).kp = stand_kp_.at(j);
        msg.leg_commands.at(i).motor_commands.at(j).kd = stand_kd_.at(j);
        msg.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
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
      msg.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
          (sit_joint_angles_.at(j) - stand_joint_angles_.at(j))*t_interp + 
          stand_joint_angles_.at(j);
        msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
        msg.leg_commands.at(i).motor_commands.at(j).kp = stand_kp_.at(j);
        msg.leg_commands.at(i).motor_commands.at(j).kd = stand_kd_.at(j);
        msg.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
      }
    }
  } else {
    ROS_WARN_THROTTLE(0.5, "Invalid control mode set in ID node, "
      "exiting publishLegCommandArray()");
      return;
  }

  //ROS_INFO_THROTTLE(0.5, " ID control mode: %d", control_mode_);

  // Pack 4 LegCommands in the LegCommandArray
  // Pack 3 MotorCommands in a LegCommand
  msg.header.stamp = ros::Time::now();
  leg_command_array_pub_.publish(msg);
}
void InverseDynamics::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {

    // Collect new messages on subscriber topics
    ros::spinOnce();

    // Publish control input data
    publishLegCommandArray();

    // Enforce update rate
    r.sleep();
  }
}
