#include "inverse_dynamics/inverse_dynamics.h"

namespace plt = matplotlibcpp;

InverseDynamics::InverseDynamics(ros::NodeHandle nh) {
	nh_ = nh;

    // Load rosparams from parameter server
  std::string body_plan_topic, local_plan_topic, leg_command_array_topic, 
    control_mode_topic, leg_override_topic; 
  spirit_utils::loadROSParam(nh_,"topics/local_plan",local_plan_topic);
  spirit_utils::loadROSParam(nh_,"topics/control/joint_command",leg_command_array_topic);
  spirit_utils::loadROSParam(nh_,"topics/control/leg_override",leg_override_topic);
  spirit_utils::loadROSParam(nh_,"topics/control/mode",control_mode_topic);
  spirit_utils::loadROSParam(nh_,"inverse_dynamics/update_rate", update_rate_);
  spirit_utils::loadROSParam(nh_,"local_planner/timestep", dt_);

  // Setup pubs and subs
  body_plan_sub_ = nh_.subscribe(body_plan_topic,1,&InverseDynamics::bodyPlanCallback, this);
  local_plan_sub_ = nh_.subscribe(local_plan_topic,1,&InverseDynamics::localPlanCallback, this);
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

void InverseDynamics::bodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg) {
  // ROS_INFO("In controlInputCallback");
  last_body_plan_msg_ = msg;
}

void InverseDynamics::localPlanCallback(const spirit_msgs::LocalPlan::ConstPtr& msg) {
  // ROS_INFO("In controlInputCallback");
  last_local_plan_msg_ = msg;
}

void InverseDynamics::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {
  // ROS_INFO("In robotStateCallback");
  last_robot_state_msg_ = msg;
}

void InverseDynamics::legOverrideCallback(const spirit_msgs::LegOverride::ConstPtr& msg) {
  last_leg_override_msg_ = *msg;
}

void InverseDynamics::publishLegCommandArray() {

  if (last_robot_state_msg_ == NULL)
    return;

  bool hasTrajectory;

  if (last_local_plan_msg_ == NULL) {
    hasTrajectory = false;
  } else {
    hasTrajectory = true;
  }

  spirit_msgs::LegCommandArray msg;
  msg.leg_commands.resize(4);

  static const int testingValue = 0;

  static const std::vector<double> stand_joint_angles_{0,0.76,2*0.76};
  static const std::vector<double> sit_joint_angles_{0.0,0.0,0.0};
  static const std::vector<double> stand_kp_{50,50,50};
  static const std::vector<double> stand_kd_{1,1,1};

  static const std::vector<double> ID_kp_{10,10,10};
  static const std::vector<double> ID_kd_{0.2,0.2,0.2};

  static const std::vector<double> walk_kp_{50,50,50};
  static const std::vector<double> walk_kd_{1,1,1};

  Eigen::Vector3d grf, grf_des, kp_grf, kd_grf, grfMPC0, grfMPC1, grfMPC2, grfMPC3;
  Eigen::Vector3d body_pos, body_vel, body_pos_des, body_vel_des, body_pos_error, body_vel_error;
  Eigen::Vector3f tau0, tau1, tau2, tau3, dq0, dq1, dq2, dq3, df0, df1, df2, df3, tauMPC0, tauMPC1, tauMPC2, tauMPC3;
  grf_des << 0, 0, 0.5*11.5*9.81;
  kp_grf << 600.0, 400.0, 200.0;
  kd_grf << 20.0, 10.0, 10.0;

  spirit_msgs::RobotState ref_state_msg;
  spirit_msgs::GRFArray grf_array_msg;
  double current_time = spirit_utils::getDurationSinceTime(last_body_plan_msg_->header.stamp);
  int current_plan_index = spirit_utils::getPlanIndex(last_body_plan_msg_->header.stamp, dt_);
  double t_interp = current_time/dt_ - current_plan_index;

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

  Eigen::VectorXd ref_body_state = spirit_utils::odomMsgToEigen(ref_state_msg.body);
  Eigen::VectorXd body_state = spirit_utils::odomMsgToEigen(last_robot_state_msg_->body);

  body_pos_error = ref_body_state.segment<3>(0).array() - body_state.segment<3>(0).array();
  body_vel_error = ref_body_state.segment<3>(6).array() - body_state.segment<3>(6).array();

  grf = grf_des.array() + kp_grf.array()*body_pos_error.array() + 
    kd_grf.array()*body_vel_error.array();
  
  // std::cout << "Got here" << std::endl;

  // std::cout << last_grf_input_msg_.vectors.size() << std::endl;
  // std::cout << hasTrajectory << std::endl;

  if (hasTrajectory) {
    grfMPC0 << grf_array_msg.vectors.at(0).x, grf_array_msg.vectors.at(0).y, grf_array_msg.vectors.at(0).z;
    grfMPC1 << grf_array_msg.vectors.at(1).x, grf_array_msg.vectors.at(1).y, grf_array_msg.vectors.at(1).z;
    grfMPC2 << grf_array_msg.vectors.at(2).x, grf_array_msg.vectors.at(2).y, grf_array_msg.vectors.at(2).z;
    grfMPC3 << grf_array_msg.vectors.at(3).x, grf_array_msg.vectors.at(3).y, grf_array_msg.vectors.at(3).z;
  }

  // std::cout << "Set grfs done" << std::endl;

  // std::cout << grf_des << std::endl;
  // std::cout << "Foot 0: " << std::endl;
  // std::cout << grfMPC0 << std::endl;
  // std::cout << "Foot 1: " << std::endl;
  // std::cout << grfMPC1 << std::endl;
  // std::cout << "Foot 2: " << std::endl;
  // std::cout << grfMPC2 << std::endl;
  // std::cout << "Foot 3: " << std::endl;
  // std::cout << grfMPC3 << std::endl;

  double velocities[12];
  for (int i = 0; i < 12; i++) {
    velocities[i] = last_robot_state_msg_->joints.velocity.at(i);
  }

  double states[18];
  for (int i = 0; i < 12; i++) {
    states[i] = last_robot_state_msg_->joints.position.at(i);
  }

  double qx = last_robot_state_msg_->body.pose.pose.orientation.x;
  double qy = last_robot_state_msg_->body.pose.pose.orientation.y;
  double qz = last_robot_state_msg_->body.pose.pose.orientation.z;
  double qw = last_robot_state_msg_->body.pose.pose.orientation.w;

  double roll, pitch, yaw;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (qw * qx + qy * qz);
  double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (qw * qy - qz * qx);
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(MATH_PI / 2, sinp); // use 90 degrees if out of range
  else
  pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (qw * qz + qx * qy);
  double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  yaw = std::atan2(siny_cosp, cosy_cosp);

  states[12] = last_robot_state_msg_->body.pose.pose.position.x;
  states[13] = last_robot_state_msg_->body.pose.pose.position.y;
  states[14] = last_robot_state_msg_->body.pose.pose.position.z;
  states[15] = roll;
  states[16] = pitch;
  states[17] = yaw;

  // std::cout << "robotState: ";
  // for (int i = 0; i < 18; ++i) {
  //   std::cout << states[i] << " ";
  // }
  // std::cout << std::endl;

  Eigen::MatrixXf foot_jacobian0(3,3);
  spirit_utils::calc_foot_jacobian0(states,foot_jacobian0);

  Eigen::MatrixXf foot_jacobian1(3,3);
  spirit_utils::calc_foot_jacobian1(states,foot_jacobian1);

  Eigen::MatrixXf foot_jacobian2(3,3);
  spirit_utils::calc_foot_jacobian2(states,foot_jacobian2);

  Eigen::MatrixXf foot_jacobian3(3,3);
  spirit_utils::calc_foot_jacobian3(states,foot_jacobian3);

  Eigen::MatrixXf jacobian(12,18);
  spirit_utils::calc_jacobian(states,jacobian);

  Eigen::MatrixXf stateVelocity(18,1);
  Eigen::MatrixXf footVelocity(12,1);
  stateVelocity << velocities[0], velocities[1], velocities[2], velocities[3], velocities[4], velocities[5],
                   velocities[6], velocities[7], velocities[8], velocities[9], velocities[10], velocities[11],
                   last_robot_state_msg_->body.twist.twist.linear.x, last_robot_state_msg_->body.twist.twist.linear.y,
                   last_robot_state_msg_->body.twist.twist.linear.z, last_robot_state_msg_->body.twist.twist.angular.x,
                   last_robot_state_msg_->body.twist.twist.angular.y, last_robot_state_msg_->body.twist.twist.angular.z; 

  tau0 = -foot_jacobian0.transpose() * grf.cast<float>();
  tau1 = -foot_jacobian1.transpose() * grf.cast<float>();
  tau2 = -foot_jacobian2.transpose() * grf.cast<float>();
  tau3 = -foot_jacobian3.transpose() * grf.cast<float>();

  tauMPC0 = -foot_jacobian0.transpose() * grfMPC0.cast<float>();
  tauMPC1 = -foot_jacobian1.transpose() * grfMPC1.cast<float>();
  tauMPC2 = -foot_jacobian2.transpose() * grfMPC2.cast<float>();
  tauMPC3 = -foot_jacobian3.transpose() * grfMPC3.cast<float>();

  // std::cout << "Foot 0: " << std::endl;
  // std::cout << tauMPC0 << std::endl;
  // std::cout << "Foot 1: " << std::endl;
  // std::cout << tauMPC1 << std::endl;
  // std::cout << "Foot 2: " << std::endl;
  // std::cout << tauMPC2 << std::endl;
  // std::cout << "Foot 3: " << std::endl;
  // std::cout << tauMPC3 << std::endl;

  footVelocity = jacobian*stateVelocity;

  f0x.push_back(last_robot_state_msg_->feet.feet[0].velocity.x);
  f1x.push_back(last_robot_state_msg_->feet.feet[1].velocity.x);
  f2x.push_back(last_robot_state_msg_->feet.feet[2].velocity.x);
  f3x.push_back(last_robot_state_msg_->feet.feet[3].velocity.x);
  f0y.push_back(last_robot_state_msg_->feet.feet[0].velocity.y);
  f1y.push_back(last_robot_state_msg_->feet.feet[1].velocity.y);
  f2y.push_back(last_robot_state_msg_->feet.feet[2].velocity.y);
  f3y.push_back(last_robot_state_msg_->feet.feet[3].velocity.y);
  f0z.push_back(last_robot_state_msg_->feet.feet[0].velocity.z);
  f1z.push_back(last_robot_state_msg_->feet.feet[1].velocity.z);
  f2z.push_back(last_robot_state_msg_->feet.feet[2].velocity.z);
  f3z.push_back(last_robot_state_msg_->feet.feet[3].velocity.z);

  f0xJ.push_back(footVelocity(0,0));
  f1xJ.push_back(footVelocity(3,0));
  f2xJ.push_back(footVelocity(6,0));
  f3xJ.push_back(footVelocity(9,0));
  f0yJ.push_back(footVelocity(1,0));
  f1yJ.push_back(footVelocity(4,0));
  f2yJ.push_back(footVelocity(7,0));
  f3yJ.push_back(footVelocity(10,0));
  f0zJ.push_back(footVelocity(2,0));
  f1zJ.push_back(footVelocity(5,0));
  f2zJ.push_back(footVelocity(8,0));
  f3zJ.push_back(footVelocity(11,0));

  counterVec.push_back(step_number);
  step_number++;

  // plt::clf();
  // plt::ion();
  // // plt::subplot(1,3,1);
  // plt::named_plot("F1x_G", counterVec, f0x, "k-");
  // // plt::named_plot("F2x_G", counterVec, f1x, "r-");
  // // plt::named_plot("F3x_G", counterVec, f2x, "b-");
  // // plt::named_plot("F4x_G", counterVec, f3x, "g-");
  // plt::named_plot("F1x_J", counterVec, f0xJ, "k--");
  // // plt::named_plot("F2x_J", counterVec, f1xJ, "r--");
  // // plt::named_plot("F3x_J", counterVec, f2xJ, "b--");
  // // plt::named_plot("F4x_J", counterVec, f3xJ, "g--");
  // plt::xlabel("Time (steps)");
  // plt::ylabel("Velocity (m/s)");
  // plt::title("X Velocities");
  // plt::legend();

  // // plt::subplot(1,3,2);
  // plt::named_plot("F1y_G", counterVec, f0y, "k-");
  // // plt::named_plot("F2y_G", counterVec, f1y, "r-");
  // // plt::named_plot("F3y_G", counterVec, f2y, "b-");
  // // plt::named_plot("F4y_G", counterVec, f3y, "g-");
  // plt::named_plot("F1y_J", counterVec, f0yJ, "k--");
  // // plt::named_plot("F2y_J", counterVec, f1yJ, "r--");
  // // plt::named_plot("F3y_J", counterVec, f2yJ, "b--");
  // // plt::named_plot("F4y_J", counterVec, f3yJ, "g--");
  // plt::xlabel("Time (steps)");
  // plt::ylabel("Velocity (m/s)");
  // plt::title("Y Velocities");
  // plt::legend();

  // // plt::subplot(1,3,3);
  // plt::named_plot("F1z_G", counterVec, f0z, "k-");
  // // plt::named_plot("F2z_G", counterVec, f1z, "r-");
  // // plt::named_plot("F3z_G", counterVec, f2z, "b-");
  // // plt::named_plot("F4z_G", counterVec, f3z, "g-");
  // plt::named_plot("F1z_J", counterVec, f0zJ, "k--");
  // // plt::named_plot("F2z_J", counterVec, f1zJ, "r--");
  // // plt::named_plot("F3z_J", counterVec, f2zJ, "b--");
  // // plt::named_plot("F4z_J", counterVec, f3zJ, "g--");
  // plt::xlabel("Time (steps)");
  // plt::ylabel("Velocity (m/s)");
  // plt::title("Z Velocities");
  // plt::legend();

  // plt::show();
  // plt::pause(0.001);

  if (control_mode_ == SIT)
  { 
    for (int i = 0; i < 4; ++i) {
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
    int count = -1;
    for (int i = 0; i < 4; ++i) {
      msg.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        count++;

        if (hasTrajectory) {
          msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = ref_state_msg.joints.position.at(count);
          // msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
          msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          msg.leg_commands.at(i).motor_commands.at(j).kp = walk_kp_.at(j);
          msg.leg_commands.at(i).motor_commands.at(j).kd = walk_kd_.at(j);
          switch (i) {
            case 0:
              msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tauMPC0[j];
              break;
            case 1:
              msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tauMPC1[j];
              break;
            case 2:
              msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tauMPC2[j];
              break;
            case 3:
              msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tauMPC3[j];
              break;
              
            // case 0:
            //   msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tau0[j];
            //   break;
            // case 1:
            //   msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tau1[j];
            //   break;
            // case 2:
            //   msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tau2[j];
            //   break;
            // case 3:
            //   msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tau3[j];
            //   break;
          }
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

    for (int i = 0; i < 4; ++i) {
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

    for (int i = 0; i < 4; ++i) {
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
