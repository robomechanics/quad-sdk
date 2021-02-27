#include "inverse_dynamics/inverse_dynamics.h"

namespace plt = matplotlibcpp;

InverseDynamics::InverseDynamics(ros::NodeHandle nh) {
  nh.param<double>("mpc_controller/update_rate", update_rate_, 100);
	nh_ = nh;

    // Load rosparams from parameter server
  std::string control_input_topic, robot_state_topic, trajectory_topic, leg_command_array_topic, control_mode_topic; 
  // nh.param<std::string>("topics/control_input", control_input_topic, "/control_input");
  spirit_utils::loadROSParam(nh_,"topics/joint_command",leg_command_array_topic);
  spirit_utils::loadROSParam(nh_,"open_loop_controller/control_mode_topic",control_mode_topic);
  spirit_utils::loadROSParam(nh_,"topics/state/ground_truth",robot_state_topic);
  spirit_utils::loadROSParam(nh_,"topics/state/trajectory",trajectory_topic);

  // Setup pubs and subs
  // control_input_sub_ = nh_.subscribe(control_input_topic,1,&InverseDynamics::controlInputCallback, this);
  robot_state_sub_= nh_.subscribe(robot_state_topic,1,&InverseDynamics::robotStateCallback, this);
  trajectory_sub_ = nh_.subscribe(trajectory_topic,1,&InverseDynamics::trajectoryCallback, this);
  control_mode_sub_ = nh_.subscribe(control_mode_topic,1,&InverseDynamics::controlModeCallback, this);
  leg_command_array_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>(leg_command_array_topic,1);

  // Start standing
  control_mode_ = 0;

  step_number = 0;
}

void InverseDynamics::controlModeCallback(const std_msgs::UInt8::ConstPtr& msg) {
  if (0 <= msg->data && msg->data <= 2)
  {
    control_mode_ = msg->data;
  }
}

// void InverseDynamics::controlInputCallback(const spirit_msgs::ControlInput::ConstPtr& msg) {
//   // ROS_INFO("In controlInputCallback");
//   last_control_input_msg_ = *msg;
// }

void InverseDynamics::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {
  // ROS_INFO("In robotStateCallback");
  last_robot_state_msg_ = *msg;
}

void InverseDynamics::trajectoryCallback(const spirit_msgs::RobotState::ConstPtr& msg) {
  // ROS_INFO("In footPlanContinuousCallback");
  last_trajectory_msg_ = *msg;
}

void InverseDynamics::publishLegCommandArray() {

  if (last_robot_state_msg_.joints.position.size() == 0)
    return;

  bool hasTrajectory = true;

  if (last_trajectory_msg_.joints.position.size() == 0) {
    hasTrajectory = false;
  }

  // if (hasTrajectory) {
  //   math_utils::ikRobotState(last_robot_state_msg_.body, last_trajectory_msg_.feet, last_trajectory_msg_.joints);
  // }

  // ROS_INFO("In InverseDynamics");
  spirit_msgs::LegCommandArray msg;
  msg.leg_commands.resize(4);

  static const int testingValue = 0;

  static const std::vector<double> stand_joint_angles_{0,0.76,2*0.76};
  static const std::vector<double> stand_kp_{50,50,50};
  static const std::vector<double> stand_kd_{2,2,2};

  static const std::vector<double> ID_kp_{10,10,10};
  static const std::vector<double> ID_kd_{0.2,0.2,0.2};

  static const std::vector<double> walk_kp_{100,100,100};
  static const std::vector<double> walk_kd_{2,2,2};

  Eigen::Vector3d grf, grf_des, kp_grf, kd_grf;
  Eigen::Vector3d body_pos, body_vel, body_pos_des, body_vel_des, body_pos_error, body_vel_error;
  Eigen::Vector3f tau0, tau1, tau2, tau3, dq0, dq1, dq2, dq3, df0, df1, df2, df3;
  grf_des << 0, 0, 0.5*11.5*9.81;
  kp_grf << 600.0, 400.0, 200.0;
  kd_grf << 20.0, 10.0, 10.0;

  tf::pointMsgToEigen(last_robot_state_msg_.body.pose.pose.position, body_pos);
  tf::vectorMsgToEigen(last_robot_state_msg_.body.twist.twist.linear, body_vel);
  tf::pointMsgToEigen(last_trajectory_msg_.body.pose.pose.position, body_pos_des);
  tf::vectorMsgToEigen(last_trajectory_msg_.body.twist.twist.linear, body_vel_des);

  body_pos_error = body_pos_des.array() - body_pos.array();
  body_vel_error = body_vel_des.array() - body_vel.array();

  grf = grf_des.array() + kp_grf.array()*body_pos_error.array() + 
    kd_grf.array()*body_vel_error.array();
  
  // std::cout << grf_des << std::endl;

  double velocities[12];

  velocities[0] = last_robot_state_msg_.joints.velocity.at(0);
  velocities[1] = last_robot_state_msg_.joints.velocity.at(1);
  velocities[2] = last_robot_state_msg_.joints.velocity.at(2);
  velocities[3] = last_robot_state_msg_.joints.velocity.at(3);
  velocities[4] = last_robot_state_msg_.joints.velocity.at(4);
  velocities[5] = last_robot_state_msg_.joints.velocity.at(5);
  velocities[6] = last_robot_state_msg_.joints.velocity.at(6);
  velocities[7] = last_robot_state_msg_.joints.velocity.at(7);
  velocities[8] = last_robot_state_msg_.joints.velocity.at(8);
  velocities[9] = last_robot_state_msg_.joints.velocity.at(9);
  velocities[10] = last_robot_state_msg_.joints.velocity.at(10);
  velocities[11] = last_robot_state_msg_.joints.velocity.at(11);

  double states[18];
  // std::vector<double> states(18);
  static const double parameters[] = {0.07,0.2263,0.0,0.1010,0.206,0.206};
  // double states[] = {q00, q01, q02, q10, q11, q12, q20, q21, q22, q30, q31, q32, x, y, z, roll, pitch, yaw};

  states[0] = last_robot_state_msg_.joints.position.at(0);
  states[1] = last_robot_state_msg_.joints.position.at(1);
  states[2] = last_robot_state_msg_.joints.position.at(2);
  states[3] = last_robot_state_msg_.joints.position.at(3);
  states[4] = last_robot_state_msg_.joints.position.at(4);
  states[5] = last_robot_state_msg_.joints.position.at(5);
  states[6] = last_robot_state_msg_.joints.position.at(6);
  states[7] = last_robot_state_msg_.joints.position.at(7);
  states[8] = last_robot_state_msg_.joints.position.at(8);
  states[9] = last_robot_state_msg_.joints.position.at(9);
  states[10] = last_robot_state_msg_.joints.position.at(10);
  states[11] = last_robot_state_msg_.joints.position.at(11);

  double qx = last_robot_state_msg_.body.pose.pose.orientation.x;
  double qy = last_robot_state_msg_.body.pose.pose.orientation.y;
  double qz = last_robot_state_msg_.body.pose.pose.orientation.z;
  double qw = last_robot_state_msg_.body.pose.pose.orientation.w;

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

  states[12] = last_robot_state_msg_.body.pose.pose.position.x;
  states[13] = last_robot_state_msg_.body.pose.pose.position.y;
  states[14] = last_robot_state_msg_.body.pose.pose.position.z;
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
                   last_robot_state_msg_.body.twist.twist.linear.x, last_robot_state_msg_.body.twist.twist.linear.y,
                   last_robot_state_msg_.body.twist.twist.linear.z, last_robot_state_msg_.body.twist.twist.angular.x,
                   last_robot_state_msg_.body.twist.twist.angular.y, last_robot_state_msg_.body.twist.twist.angular.z; 

  tau0 = -foot_jacobian0.transpose() * grf.cast<float>();
  tau1 = -foot_jacobian1.transpose() * grf.cast<float>();
  tau2 = -foot_jacobian2.transpose() * grf.cast<float>();
  tau3 = -foot_jacobian3.transpose() * grf.cast<float>();

  footVelocity = jacobian*stateVelocity;

  f0x.push_back(last_robot_state_msg_.feet.feet[0].velocity.x);
  f1x.push_back(last_robot_state_msg_.feet.feet[1].velocity.x);
  f2x.push_back(last_robot_state_msg_.feet.feet[2].velocity.x);
  f3x.push_back(last_robot_state_msg_.feet.feet[3].velocity.x);
  f0y.push_back(last_robot_state_msg_.feet.feet[0].velocity.y);
  f1y.push_back(last_robot_state_msg_.feet.feet[1].velocity.y);
  f2y.push_back(last_robot_state_msg_.feet.feet[2].velocity.y);
  f3y.push_back(last_robot_state_msg_.feet.feet[3].velocity.y);
  f0z.push_back(last_robot_state_msg_.feet.feet[0].velocity.z);
  f1z.push_back(last_robot_state_msg_.feet.feet[1].velocity.z);
  f2z.push_back(last_robot_state_msg_.feet.feet[2].velocity.z);
  f3z.push_back(last_robot_state_msg_.feet.feet[3].velocity.z);

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

  switch (control_mode_) {
    case 0: //standing
    { 
      int count = -1;
      for (int i = 0; i < 4; ++i) {
        msg.leg_commands.at(i).motor_commands.resize(3);
        for (int j = 0; j < 3; ++j) {
          count++;

          if (hasTrajectory) {
            msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = last_trajectory_msg_.joints.position.at(count);
            // msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
            msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
            msg.leg_commands.at(i).motor_commands.at(j).kp = walk_kp_.at(j);
            msg.leg_commands.at(i).motor_commands.at(j).kd = walk_kd_.at(j);
            switch (i) {
              case 0:
                msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tau0[j];
                break;
              case 1:
                msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tau1[j];
                break;
              case 2:
                msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tau2[j];
                break;
              case 3:
                msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tau3[j];
                break;
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
      break;
    }

    case 1: //feed-forward torques
    {
      for (int i = 0; i < 4; ++i) {
        msg.leg_commands.at(i).motor_commands.resize(3);
        for (int j = 0; j < 3; ++j) {
          msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
          msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          msg.leg_commands.at(i).motor_commands.at(j).kp = ID_kp_.at(j);
          msg.leg_commands.at(i).motor_commands.at(j).kd = ID_kd_.at(j);
          switch (i) {
            case 0:
              msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tau0[j];
              break;
            case 1:
              msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tau1[j];
              break;
            case 2:
              msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tau2[j];
              break;
            case 3:
              msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tau3[j];
              break;
          }
        }
      }
      break;
    }
  }

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