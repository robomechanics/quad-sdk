#include "open_loop_controller/open_loop_controller.h"

OpenLoopController::OpenLoopController(ros::NodeHandle nh)
{
  nh_ = nh;

  // Get rosparams
  std::string leg_control_topic, control_mode_topic, robot_state_topic, cmd_vel_topic;
  spirit_utils::loadROSParam(nh_, "topics/cmd_vel", cmd_vel_topic);
  spirit_utils::loadROSParam(nh_, "topics/state/ground_truth", robot_state_topic);
  spirit_utils::loadROSParam(nh_, "topics/control/joint_command", leg_control_topic);
  spirit_utils::loadROSParam(nh_, "topics/control/mode", control_mode_topic);
  spirit_utils::loadROSParam(nh_, "open_loop_controller/update_rate", update_rate_);
  spirit_utils::loadROSParam(nh_, "open_loop_controller/stand_angles", stand_joint_angles_);
  spirit_utils::loadROSParam(nh_, "open_loop_controller/sit_angles", sit_joint_angles_);
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
  cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic, 1, &OpenLoopController::cmdVelCallback, this);

  // Convert kinematics
  kinematics_ = std::make_shared<spirit_utils::SpiritKinematics>();

  x_.resize(4);
  y_.resize(4);

  x_ = {0, 0, 0, 0};
  y_ = {1, -1, -1, 1};
}

void OpenLoopController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  if (abs(msg->linear.x) < 1e-3 && abs(msg->linear.y) < 1e-3)
  {
    control_mode_ = 1;
  }
  else if (abs(msg->linear.x) > abs(msg->linear.y))
  {
    if (msg->linear.x >= 0)
    {
      control_mode_ = 2;
    }
    else
    {
      control_mode_ = 3;
    }
  }
  else
  {
    if (msg->linear.y >= 0)
    {
      control_mode_ = 4;
    }
    else
    {
      control_mode_ = 5;
    }
  }
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
  if (last_robot_state_msg_ == NULL)
  {
    return;
  }

  spirit_msgs::LegCommandArray msg;
  msg.leg_commands.resize(4);

  // Define vectors for joint positions and velocities
  Eigen::VectorXd joint_positions(3 * 4), joint_velocities(3 * 4), body_state(12), current_foot_positions(3 * 4);
  spirit_utils::vectorToEigen(last_robot_state_msg_->joints.position, joint_positions);
  spirit_utils::vectorToEigen(last_robot_state_msg_->joints.velocity, joint_velocities);
  body_state = spirit_utils::bodyStateMsgToEigen(last_robot_state_msg_->body);

  spirit_utils::multiFootStateMsgToEigen(last_robot_state_msg_->feet, current_foot_positions);
  for (size_t i = 0; i < 4; i++)
  {
    current_foot_positions.segment(3 * i, 3) = current_foot_positions.segment(3 * i, 3) - body_state.head(3);
  }

  // Define vectors for state positions and velocities
  Eigen::VectorXd state_positions(3 * 4 + 6), state_velocities(3 * 4 + 6);
  state_positions << joint_positions, body_state.head(6);
  state_velocities << joint_velocities, body_state.tail(6);

  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3 * 4, 3 * 4 + 6);
  kinematics_->getJacobianBodyAngVel(state_positions, jacobian);
  // Eigen::MatrixXd force_basis(3, 4);
  // for (size_t i = 0; i < 4; i++)
  // {
  //   force_basis.col(i) = -current_foot_positions.segment(3 * i, 3);
  //   force_basis.col(i) = force_basis.col(i) / force_basis.col(i).norm();
  // }
  Eigen::MatrixXd force_basis(6, 4);
  for (size_t i = 0; i < 4; i++)
  {
    force_basis.col(i) << 0, 0, 1, current_foot_positions(3 * i + 1), -current_foot_positions(3 * i + 0), 0;
  }

  switch (control_mode_)
  {
  case 0: // sit
  {
    for (int i = 0; i < 4; ++i)
    {
      msg.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j)
      {
        msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = sit_joint_angles_.at(j);
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
    Eigen::VectorXd grf_vec(4), grf_array(3 * 4);
    // Eigen::Vector3d gravity;
    // gravity << 0, 0, 11.51 * 9.81;
    Eigen::VectorXd gravity(6);
    gravity << 0, 0, 11.51 * 9.81, 0, 0, 0;

    grf_vec = force_basis.colPivHouseholderQr().solve(gravity);
    grf_array.setZero();
    for (size_t i = 0; i < 4; i++)
    {
      // grf_array.segment(3 * i, 3) = grf_vec(i) * force_basis.col(i);
      grf_array.segment(3 * i, 3) << 0, 0, grf_vec(i);
    }

    Eigen::VectorXd tau_array(3 * 4);
    tau_array = -jacobian.transpose().block<12, 12>(0, 0) * grf_array;

    // ROS_INFO_STREAM_THROTTLE(0.1, "force_basis:" << force_basis);
    // ROS_INFO_STREAM_THROTTLE(0.1, "grf_vec:" << grf_vec);
    // ROS_INFO_STREAM_THROTTLE(0.1, "grf_array:" << grf_array);
    // ROS_INFO_STREAM_THROTTLE(0.1, "tau_array:" << tau_array);

    for (int i = 0; i < 4; ++i)
    {
      msg.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j)
      {
        msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
        msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
        msg.leg_commands.at(i).motor_commands.at(j).kp = stand_kp_.at(j);
        msg.leg_commands.at(i).motor_commands.at(j).kd = stand_kd_.at(j);
        msg.leg_commands.at(i).motor_commands.at(j).torque_ff = tau_array(3 * i + j);
      }
    }

    x_ = {0, 0, 0, 0};
    y_ = {1, -1, -1, 1};
  }
  break;

  case 2:
  case 3:
  case 4:
  case 5:
  {
    std::vector<double> new_x(4), new_y(4);
    std::vector<int> contact_leg;

    for (size_t i = 0; i < 4; i++)
    {
      bool contact = false;

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

      switch (control_mode_)
      {
      case 2:
      case 3:
      {
        int direction = 1;
        switch (control_mode_)
        {
        case 2:
        {
          direction = 1;
        }
        break;
        case 3:
        {
          direction = -1;
        }
        break;
        }
        joint_command(0) = 0;
        joint_command(3) = 0;
        joint_command(1) = direction * x * stand_joint_angles_.at(1) / 4 + stand_joint_angles_.at(1);
        joint_command(4) = direction * stand_joint_angles_.at(1) / 4 * x_dot;

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
          contact = true;

          joint_command(2) = stand_joint_angles_.at(2) / 4 * 2 * 1 + stand_joint_angles_.at(2) / 4 * 3;
          joint_command(5) = stand_joint_angles_.at(2) / 4 * 2 * 0;
        }
      }
      break;

      case 4:
      case 5:
      {
        int direction = 1;
        switch (control_mode_)
        {
        case 4:
        {
          direction = 1;
        }
        break;
        case 5:
        {
          direction = -1;
        }
        break;
        }
        joint_command(1) = stand_joint_angles_.at(1);
        joint_command(4) = 0;
        joint_command(0) = direction * x * stand_joint_angles_.at(1) / 8 - direction * stand_joint_angles_.at(1) / 2;
        joint_command(3) = direction * stand_joint_angles_.at(1) / 8 * x_dot;

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
          contact = true;

          joint_command(2) = stand_joint_angles_.at(2) / 4 * 2 * 1 + stand_joint_angles_.at(2) / 4 * 3;
          joint_command(5) = stand_joint_angles_.at(2) / 4 * 2 * 0;
        }
      }
      break;
      }
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

      contact_leg.push_back(idx);

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

    // Eigen::MatrixXd partial_force_basis(3, contact_leg.size());
    Eigen::MatrixXd partial_force_basis(6, contact_leg.size());
    for (size_t i = 0; i < contact_leg.size(); i++)
    {
      partial_force_basis.col(i) = force_basis.col(contact_leg.at(i));
    }

    Eigen::VectorXd grf_vec(contact_leg.size()), grf_array(3 * 4);
    // Eigen::Vector3d gravity;
    // gravity << 0, 0, 11.51 * 9.81;
    Eigen::VectorXd gravity(6);
    gravity << 0, 0, 11.51 * 9.81, 0, 0, 0;

    grf_vec = partial_force_basis.colPivHouseholderQr().solve(gravity);
    grf_array.setZero();
    for (size_t i = 0; i < contact_leg.size(); i++)
    {
      // grf_array.segment(3 * contact_leg.at(i), 3) = grf_vec(i) * partial_force_basis.col(i);
      grf_array.segment(3 * contact_leg.at(i), 3) << 0, 0, grf_vec(i);
    }

    Eigen::VectorXd tau_array(3 * 4);
    tau_array = -jacobian.transpose().block<12, 12>(0, 0) * grf_array;

    for (size_t i = 0; i < contact_leg.size(); i++)
    {
      msg.leg_commands.at(contact_leg.at(i)).motor_commands.at(0).torque_ff = tau_array(contact_leg.at(i) * 3 + 0);
      msg.leg_commands.at(contact_leg.at(i)).motor_commands.at(1).torque_ff = tau_array(contact_leg.at(i) * 3 + 1);
      msg.leg_commands.at(contact_leg.at(i)).motor_commands.at(2).torque_ff = tau_array(contact_leg.at(i) * 3 + 2);
    }

    // ROS_INFO_STREAM_THROTTLE(0.1, "force_basis:" << force_basis);
    // ROS_INFO_STREAM_THROTTLE(0.1, "grf_vec:" << grf_vec);
    // ROS_INFO_STREAM_THROTTLE(0.1, "grf_array:" << grf_array);
    // ROS_INFO_STREAM_THROTTLE(0.1, "tau_array:" << tau_array);
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