#include "open_loop_controller/open_loop_controller.h"

OpenLoopController::OpenLoopController(ros::NodeHandle nh)
{
  nh_ = nh;

  // Get rosparams
  std::string leg_control_topic, control_mode_topic, robot_state_topic;
  spirit_utils::loadROSParam(nh_, "topics/state/ground_truth", robot_state_topic);
  spirit_utils::loadROSParam(nh_, "topics/control/joint_command", leg_control_topic);
  spirit_utils::loadROSParam(nh_, "topics/control/mode", control_mode_topic);
  spirit_utils::loadROSParam(nh_, "open_loop_controller/update_rate", update_rate_);
  spirit_utils::loadROSParam(nh_, "open_loop_controller/stand_angles", stand_joint_angles_);
  spirit_utils::loadROSParam(nh_, "open_loop_controller/stand_kp", stand_kp_);
  spirit_utils::loadROSParam(nh_, "open_loop_controller/stand_kd", stand_kd_);
  spirit_utils::loadROSParam(nh_, "open_loop_controller/walk_kp", walk_kp_);
  spirit_utils::loadROSParam(nh_, "open_loop_controller/walk_kd", walk_kd_);
  spirit_utils::loadROSParam(nh_, "open_loop_controller/leg_phases", leg_phases_);
  spirit_utils::loadROSParam(nh_, "open_loop_controller/use_diff_for_velocity", use_diff_for_velocity_);

  // Start sitting
  control_mode_ = 0;

  // Setup pubs and subs
  joint_control_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>(leg_control_topic, 1);
  robot_state_sub_ = nh_.subscribe(robot_state_topic, 1, &OpenLoopController::robotStateCallback, this);
  control_mode_sub_ = nh_.subscribe(control_mode_topic, 1, &OpenLoopController::controlModeCallback, this);

  x_.resize(4);
  y_.resize(4);

  x_ = {0, 0, 0, 0};
  y_ = {1, -1, -1, 1};
}

void OpenLoopController::robotStateCallback(const spirit_msgs::RobotState::ConstPtr &msg)
{
  last_robot_state_msg_ = msg;
}

void OpenLoopController::controlModeCallback(const std_msgs::UInt8::ConstPtr &msg)
{
  if (0 <= msg->data && msg->data <= 2)
  {
    control_mode_ = msg->data;
  }
}

void OpenLoopController::sendJointPositions(double &elapsed_time)
{
  spirit_msgs::LegCommandArray msg;
  msg.leg_commands.resize(4);

  switch (control_mode_)
  {
  case 0: // sit
  {
    for (int i = 0; i < 4; ++i)
    {
      msg.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j)
      {
        msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 0;
        msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
        msg.leg_commands.at(i).motor_commands.at(j).kp = 5;
        msg.leg_commands.at(i).motor_commands.at(j).kd = 0.1;
        msg.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
      }
    }

    x_ = {0, 0, 0, 0};
    y_ = {1, -1, -1, 1};
  }
  break;

  case 1: // stand
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

    x_ = {0, 0, 0, 0};
    y_ = {1, -1, -1, 1};
  }
  break;

  case 2: // walk
  {

    if (last_robot_state_msg_ == NULL)
      return;

    Eigen::VectorXd joint_positions(12), joint_velocities(12);
    spirit_utils::vectorToEigen(last_robot_state_msg_->joints.position, joint_positions);
    spirit_utils::vectorToEigen(last_robot_state_msg_->joints.velocity, joint_velocities);

    std::vector<double> new_x(4), new_y(4);

    for (size_t i = 0; i < 4; i++)
    {
      double x = x_.at(i);
      double y = y_.at(i);

      double mu = 1;
      double alpha = 5;
      double beta = 50;
      double w_stance = 4 * 3.14;
      double w_swing = 4 * 3.14;
      double b = 100;
      double r = sqrt(x * x + y * y);

      Eigen::MatrixXd k(4, 4);
      // k << 0, -1, 1, -1,
      //     -1, 0, -1, 1,
      //     1, -1, 0, -1,
      //     -1, 1, -1, 0;
      // k << 0, 1, -1, -1,
      //     1, 0, -1, -1,
      //     -1, -1, 0, 1,
      //     -1, -1, 1, 0;
      k << 0, -1, -1, 1,
          -1, 0, 1, -1,
          -1, 1, 0, -1,
          1, -1, -1, 0;
      // k << 0, -1, 1, -1,
      //     -1, 0, -1, 1,
      //     -1, 1, 0, -1,
      //     1, -1, -1, 0;

      double w = w_stance / (exp(-b * y) + 1) + w_swing / (exp(b * y) + 1);

      double x_dot = alpha * (mu - r * r) * x - w * y;
      double y_dot = beta * (mu - r * r) * y + w * x;
      for (size_t j = 0; j < 4; j++)
      {
        y_dot += k(i, j) * y_.at(j);
      }

      x += 0.002 * x_dot;
      y += 0.002 * y_dot;

      Eigen::VectorXd joint_command(6);

      joint_command(0) = 0;
      joint_command(3) = 0;
      joint_command(1) = x * stand_joint_angles_.at(1) / 4 + stand_joint_angles_.at(1);
      joint_command(4) = stand_joint_angles_.at(1) / 4 * x_dot;

      if (y <= 0)
      {
        joint_command(2) = stand_joint_angles_.at(2) / 4 * 2 * 0 + stand_joint_angles_.at(2) / 4 * 3;
        joint_command(5) = stand_joint_angles_.at(2) / 4 * 2 * 0;
      }
      else if (y > 0 && abs(x) > 0.9)
      {
        joint_command(2) = stand_joint_angles_.at(2) / 4 * 2 * 10 * (1 - abs(x)) + stand_joint_angles_.at(2) / 4 * 3;
        if (x > 0)
        {
          joint_command(5) = -stand_joint_angles_.at(2) / 4 * 2 * 10 * x_dot;
        }
        else
        {
          joint_command(5) = stand_joint_angles_.at(2) / 4 * 2 * 10 * x_dot;
        }
      }
      else
      {
        joint_command(2) = stand_joint_angles_.at(2) / 4 * 2 * 1 + stand_joint_angles_.at(2) / 4 * 3;
        joint_command(5) = stand_joint_angles_.at(2) / 4 * 2 * 0;
      }

      // joint_command(0) = x * stand_joint_angles_.at(1) / 8;
      // joint_command(3) = stand_joint_angles_.at(1) / 8 * x_dot;

      // if (y <= 0)
      // {
      //   joint_command(2) = stand_joint_angles_.at(2) / 4 * 2 * 0 + stand_joint_angles_.at(2) / 4 * 3;
      //   joint_command(5) = stand_joint_angles_.at(2) / 4 * 2 * 0;

      //   joint_command(1) = stand_joint_angles_.at(1) / 4 * 2 * 0 + stand_joint_angles_.at(1) / 4 * 3;
      //   joint_command(4) = stand_joint_angles_.at(1) / 4 * 2 * 0;
      // }
      // else if (y > 0 && abs(x) > 0.9)
      // {
      //   joint_command(2) = stand_joint_angles_.at(2) / 4 * 2 * 10 * (1 - abs(x)) + stand_joint_angles_.at(2) / 4 * 3;
      //   joint_command(1) = stand_joint_angles_.at(1) / 4 * 2 * 10 * (1 - abs(x)) + stand_joint_angles_.at(1) / 4 * 3;
      //   if (x > 0)
      //   {
      //     joint_command(5) = -stand_joint_angles_.at(2) / 4 * 2 * 10 * x_dot;
      //     joint_command(1) = -stand_joint_angles_.at(1) / 4 * 2 * 10 * x_dot;
      //   }
      //   else
      //   {
      //     joint_command(5) = stand_joint_angles_.at(2) / 4 * 2 * 10 * x_dot;
      //     joint_command(1) = stand_joint_angles_.at(1) / 4 * 2 * 10 * x_dot;
      //   }
      // }
      // else
      // {
      //   joint_command(2) = stand_joint_angles_.at(2) / 4 * 2 * 1 + stand_joint_angles_.at(2) / 4 * 3;
      //   joint_command(5) = stand_joint_angles_.at(2) / 4 * 2 * 0;

      //   joint_command(1) = stand_joint_angles_.at(1) / 4 * 2 * 1 + stand_joint_angles_.at(2) / 4 * 3;
      //   joint_command(4) = stand_joint_angles_.at(1) / 4 * 2 * 0;
      // }

      // Fill out motor command
      int idx;
      if (i == 1)
      {
        idx = 2;
      }
      else if (i == 2)
      {
        idx = 1;
      }
      else
      {
        idx = i;
      }

      msg.leg_commands.at(idx).motor_commands.resize(3);

      msg.leg_commands.at(idx).motor_commands.at(0).pos_setpoint = joint_command(0);
      msg.leg_commands.at(idx).motor_commands.at(0).kp = walk_kp_.at(0);
      msg.leg_commands.at(idx).motor_commands.at(0).kd = walk_kd_.at(0);
      msg.leg_commands.at(idx).motor_commands.at(0).vel_setpoint = joint_command(3);
      msg.leg_commands.at(idx).motor_commands.at(0).torque_ff = 0;

      msg.leg_commands.at(idx).motor_commands.at(1).pos_setpoint = joint_command(1);
      msg.leg_commands.at(idx).motor_commands.at(1).kp = walk_kp_.at(1);
      msg.leg_commands.at(idx).motor_commands.at(1).kd = walk_kd_.at(1);
      msg.leg_commands.at(idx).motor_commands.at(1).vel_setpoint = joint_command(4);
      msg.leg_commands.at(idx).motor_commands.at(1).torque_ff = 0;

      msg.leg_commands.at(idx).motor_commands.at(2).pos_setpoint = joint_command(2);
      msg.leg_commands.at(idx).motor_commands.at(2).kp = walk_kp_.at(2);
      msg.leg_commands.at(idx).motor_commands.at(2).kd = walk_kd_.at(2);
      msg.leg_commands.at(idx).motor_commands.at(2).vel_setpoint = joint_command(5);
      msg.leg_commands.at(idx).motor_commands.at(2).torque_ff = 0;

      new_x.at(i) = x;
      new_y.at(i) = y;
    }

    x_ = new_x;
    y_ = new_y;
  }
  break;
  }
  msg.header.stamp = ros::Time::now();
  joint_control_pub_.publish(msg);
}

void OpenLoopController::spin()
{
  double start_time = ros::Time::now().toSec();
  ros::Rate r(update_rate_);
  while (ros::ok())
  {
    double elapsed_time = ros::Time::now().toSec() - start_time;
    this->sendJointPositions(elapsed_time);
    ros::spinOnce();
    r.sleep();
  }
}