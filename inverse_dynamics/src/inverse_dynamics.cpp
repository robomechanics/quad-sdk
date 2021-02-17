#include "inverse_dynamics/inverse_dynamics.h"

inverseDynamics::inverseDynamics(ros::NodeHandle nh) {
  nh.param<double>("mpc_controller/update_rate", update_rate_, 100);
	nh_ = nh;

    // Load rosparams from parameter server
  std::string control_input_topic, robot_state_topic, swing_leg_plan_topic, leg_command_array_topic; //foot_step_plan_topic, 
  // nh.param<std::string>("topics/control_input", control_input_topic, "/control_input");
  spirit_utils::loadROSParam(nh_,"topics/joint_command",leg_command_array_topic);
  spirit_utils::loadROSParam(nh_,"topics/state/ground_truth",robot_state_topic);

  // Setup pubs and subs
  // control_input_sub_ = nh_.subscribe(control_input_topic,1,&inverseDynamics::controlInputCallback, this);
  robot_state_sub_= nh_.subscribe(robot_state_topic,1,&inverseDynamics::robotStateCallback, this);
  // swing_leg_plan_sub_= nh_.subscribe(swing_leg_plan_topic,1,&inverseDynamics::swingLegPlanCallback, this);
  // foot_step_plan_sub_= nh_.subscribe(foot_step_plan_topic,1,&inverseDynamics::footStepPlanCallback, this);
  leg_command_array_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>(leg_command_array_topic,1);
}

// void inverseDynamics::controlInputCallback(const spirit_msgs::ControlInput::ConstPtr& msg) {
//   // ROS_INFO("In controlInputCallback");
//   last_control_input_msg_ = *msg;
// }
void inverseDynamics::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {
  // ROS_INFO("In robotStateCallback");
  last_robot_state_msg_ = *msg;
}
// void inverseDynamics::swingLegPlanCallback(const spirit_msgs::SwingLegPlan::ConstPtr& msg) {
//   // ROS_INFO("In swingLegPlanCallback");
//   last_swing_leg_plan_msg_ = *msg;
// }
// void inverseDynamics::footStepPlanCallback(const spirit_msgs::FootStepPlan::ConstPtr& msg) {
//   // ROS_INFO("In footSteplsPlanCallback");
//   last_foot_step_plan_msg_ = *msg;
// }

void inverseDynamics::publishLegCommandArray() {
  // ROS_INFO("In inverseDynamics");
  spirit_msgs::LegCommandArray msg;
  msg.leg_commands.resize(4);

  static const int testingValue = 0;

  static const std::vector<double> stand_joint_angles_{0,0.7,1.2};
  static const std::vector<double> stand_kp_{100,100,100};
  static const std::vector<double> stand_kd_{2,2,2};

  Eigen::Vector3f grf;
  grf << 0, 0, 50;
  // std::cout << grf << std::endl;

  double states[18];
  static const double parameters[] = {0.07,0.2263,0.0,0.1010,0.206,0.206};
  // double states[] = {roll, pitch, yaw, x, y, z, q00, q01, q02, q10, q11, q12, q20, q21, q22, q30, q31, q32};

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

  states[0] = roll;
  states[1] = pitch;
  states[2] = yaw;
  states[3] = last_robot_state_msg_.body.pose.pose.position.x;
  states[4] = last_robot_state_msg_.body.pose.pose.position.y;
  states[5] = last_robot_state_msg_.body.pose.pose.position.z;

  states[6] = last_robot_state_msg_.joints.position.at(0);
  states[7] = last_robot_state_msg_.joints.position.at(1);
  states[8] = last_robot_state_msg_.joints.position.at(2);
  states[9] = last_robot_state_msg_.joints.position.at(3);
  states[10] = last_robot_state_msg_.joints.position.at(4);
  states[11] = last_robot_state_msg_.joints.position.at(5);
  states[12] = last_robot_state_msg_.joints.position.at(6);
  states[13] = last_robot_state_msg_.joints.position.at(7);
  states[14] = last_robot_state_msg_.joints.position.at(8);
  states[15] = last_robot_state_msg_.joints.position.at(9);
  states[16] = last_robot_state_msg_.joints.position.at(10);
  states[17] = last_robot_state_msg_.joints.position.at(11);

  std::cout << "robotState: ";
  for (int i = 0; i < 18; ++i) {
    std::cout << states[i] << " ";
  }
  std::cout << std::endl;

  Eigen::MatrixXf foot_jacobian0(3,3);
  // spirit_utils::calc_foot_jacobian0(states,parameters,foot_jacobian0); //failed here

  // std::cout<<"Foot Jacobian: "<<std::endl;
  // std::cout<<foot_jacobian0;
  // std::cout<<std::endl;

  switch (testingValue) {
    case 0: //standing
    {
      for (int i = 0; i < 4; ++i)
      {
        msg.leg_commands.at(i).motor_commands.resize(3);
        for (int j = 0; j < 3; ++j)
        {
          msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
          msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          msg.leg_commands.at(i).motor_commands.at(j).kp = stand_kp_.at(j);
          msg.leg_commands.at(i).motor_commands.at(j).kd = stand_kd_.at(j);
          msg.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
        }
      }
    }

    // case 1: //feed-forward torques
    // {
    //   for (int i = 0; i < 4; ++i)
    //   {
    //     msg.leg_commands.at(i).motor_commands.resize(3);
    //     for (int j = 0; j < 3; ++j)
    //     {
    //       msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
    //       msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
    //       msg.leg_commands.at(i).motor_commands.at(j).kp = 0;
    //       msg.leg_commands.at(i).motor_commands.at(j).kd = 0;
    //       msg.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
    //     }
    //   }
    // }
  }

  // Pack 4 LegCommands in the LegCommandArray
  // Pack 3 MotorCommands in a LegCommand
  msg.header.stamp = ros::Time::now();
  leg_command_array_pub_.publish(msg);
}
void inverseDynamics::spin() {
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