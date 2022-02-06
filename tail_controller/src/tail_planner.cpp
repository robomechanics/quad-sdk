#include "tail_controller/tail_planner.h"

TailPlanner::TailPlanner(ros::NodeHandle nh)
{
  nh_ = nh;

  // Get rosparams
  std::string body_plan_topic, robot_state_topic, local_plan_topic, tail_plan_topic, cmd_vel_topic, grf_topic;
  quad_utils::loadROSParam(nh_, "/topics/control/tail_plan", tail_plan_topic);
  quad_utils::loadROSParam(nh_, "/topics/global_plan", body_plan_topic);
  quad_utils::loadROSParam(nh_, "/topics/state/ground_truth", robot_state_topic);
  quad_utils::loadROSParam(nh_, "/topics/local_plan", local_plan_topic);
  quad_utils::loadROSParam(nh_, "/tail_controller/planner_update_rate", update_rate_);
  nh_.param<std::string>("/topics/cmd_vel", cmd_vel_topic, "/cmd_vel");
  quad_utils::loadROSParam(nh_, "/topics/state/grfs", grf_topic);

  // Setup pubs and subs
  tail_plan_pub_ = nh_.advertise<quad_msgs::LegCommandArray>(tail_plan_topic, 1);
  body_plan_sub_ = nh_.subscribe(body_plan_topic, 1, &TailPlanner::robotPlanCallback, this, ros::TransportHints().tcpNoDelay(true));
  robot_state_sub_ = nh_.subscribe(robot_state_topic, 1, &TailPlanner::robotStateCallback, this,ros::TransportHints().tcpNoDelay(true));
  local_plan_sub_ = nh_.subscribe(local_plan_topic, 1, &TailPlanner::localPlanCallback, this,ros::TransportHints().tcpNoDelay(true));
  cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic, 1, &TailPlanner::cmdVelCallback, this, ros::TransportHints().tcpNoDelay(true));
  grf_sub_ = nh_.subscribe(grf_topic, 1, &TailPlanner::grfCallback, this, ros::TransportHints().tcpNoDelay(true));

  nh.param<int>("/tail_controller/tail_type", tail_type_, 0);
  nh.param<bool>("/local_planner/use_twist_input", use_twist_input_, false);

  nh_.param<double>("/twist_body_planner/cmd_vel_scale", cmd_vel_scale_, 1);
  nh_.param<double>("/twist_body_planner/last_cmd_vel_msg_time_max", last_cmd_vel_msg_time_max_, 1.0);

  tail_planner_ = std::make_shared<NMPCController>(tail_type_);

  std::string param_ns;
  switch (tail_type_)
  {
  case NONE:
    // Leg controller
    param_ns = "leg";
    break;
  case CENTRALIZED:
    // Centralized tail controller
    param_ns = "centralized_tail";
    break;
  case DISTRIBUTED:
    // Distributed tail controller
    param_ns = "distributed_tail";
    break;
  case DECENTRALIZED:
    // Decentralized tail controller
    param_ns = "decentralized_tail";
    break;
  default:
    param_ns = "leg";
    break;
  }

  ros::param::get("/nmpc_controller/" + param_ns + "/horizon_length", N_);
  ros::param::get("/nmpc_controller/" + param_ns + "/step_length", dt_);

  contact_schedule_.resize(N_);
  for (size_t i = 0; i < N_; i++)
  {
    contact_schedule_[i].resize(4);
  }

  ref_body_plan_ = Eigen::MatrixXd::Zero(N_ + 1, 12);
  foot_positions_body_ = Eigen::MatrixXd::Zero(N_, 12);
  body_plan_ = Eigen::MatrixXd::Zero(N_ + 1, 12);
  grf_plan_ = Eigen::MatrixXd::Zero(N_, 12);
  tail_plan_ = Eigen::MatrixXd::Zero(N_ + 1, 4);
  tail_torque_plan_ = Eigen::MatrixXd::Zero(N_, 2);
  current_state_ = Eigen::VectorXd::Zero(12);
  current_foot_positions_world_ = Eigen::VectorXd::Zero(12);
  ref_ground_height_ = Eigen::VectorXd::Zero(N_ + 1);

  current_plan_index_ = 0;

  // Initialize twist input
  cmd_vel_.resize(6);

  // Zero the velocity to start
  std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0);

  // Assume we know the step height
  z_des_ == std::numeric_limits<double>::max();

  miss_contact_leg_.resize(4);

  // Initialize the time duration to the next plan index
  first_element_duration_ = dt_;

  // Initialize the plan index boolean
  same_plan_index_ = true;
}

void TailPlanner::robotPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg)
{
  body_plan_msg_ = msg;
}

void TailPlanner::robotStateCallback(const quad_msgs::RobotState::ConstPtr &msg)
{
  // Make sure the data is actually populated
  if (msg->feet.feet.empty() || msg->joints.position.empty())
    return;

  robot_state_msg_ = msg;
}

void TailPlanner::localPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg)
{
  last_local_plan_msg_ = msg;
}

void TailPlanner::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  if ((cmd_vel_[0] != msg->linear.x) || (cmd_vel_[1] != msg->linear.y) ||
      (cmd_vel_[5] != msg->angular.z))
  {

    //initial_timestamp_ = ros::Time::now();
  }
  // Ignore non-planar components of desired twist
  cmd_vel_[0] = cmd_vel_scale_ * msg->linear.x;
  cmd_vel_[1] = cmd_vel_scale_ * msg->linear.y;
  cmd_vel_[2] = 0;
  cmd_vel_[3] = 0;
  cmd_vel_[4] = 0;
  cmd_vel_[5] = cmd_vel_scale_ * msg->angular.z;

  // Record when this was last reached for safety
  last_cmd_vel_msg_time_ = ros::Time::now();
}

void TailPlanner::grfCallback(const quad_msgs::GRFArray::ConstPtr &msg)
{
  grf_msg_ = msg;
}

void TailPlanner::computeTailPlan()
{
  if (last_local_plan_msg_ == NULL || (body_plan_msg_ == NULL && !use_twist_input_) || robot_state_msg_ == NULL || grf_msg_ == NULL)
    return;

  // Start the timer
  quad_utils::FunctionTimer timer(__FUNCTION__);

  // Start from current time
  tail_current_state_ = quad_utils::odomMsgToEigenForTail(*robot_state_msg_);
  current_state_ = quad_utils::bodyStateMsgToEigen(robot_state_msg_->body);
  quad_utils::multiFootStateMsgToEigen(robot_state_msg_->feet, current_foot_positions_world_);

  // Define reference tail plan
  ref_tail_plan_ = Eigen::MatrixXd::Zero(N_ + 1, 4);
  ref_tail_plan_.col(0) = Eigen::MatrixXd::Constant(N_ + 1, 1, 0.76);
  ref_tail_plan_.row(0) = tail_current_state_.transpose();

  double t_now = robot_state_msg_->header.stamp.toSec();

  if ((t_now < last_local_plan_msg_->states.front().header.stamp.toSec()) ||
      (t_now > last_local_plan_msg_->states.back().header.stamp.toSec()))
  {
    ROS_ERROR("Tail planner node couldn't find the correct ref state!");
    ROS_ERROR_STREAM("t_now: " << t_now << ","
                               << "last_local_plan_msg_->states.front().header.stamp.toSec(): " << last_local_plan_msg_->states.front().header.stamp.toSec());
    return;
  }

  for (int i = 0; i < last_local_plan_msg_->states.size() - 1; i++)
  {
    if ((t_now >= last_local_plan_msg_->states[i].header.stamp.toSec()) &&
        (t_now < last_local_plan_msg_->states[i + 1].header.stamp.toSec()))
    {
      // // Take the closer one
      // if (t_now - last_local_plan_msg_->states[i].header.stamp.toSec() <=
      //     last_local_plan_msg_->states[i + 1].header.stamp.toSec() - t_now)
      // {
      //   current_plan_index_ = i + last_local_plan_msg_->plan_indices[0];
      // }
      // else
      // {
      //   current_plan_index_ = i + 1 + last_local_plan_msg_->plan_indices[0];
      // }

      int previous_plan_index = current_plan_index_;
      current_plan_index_ = i + last_local_plan_msg_->plan_indices[0];
      first_element_duration_ = dt_ - (t_now - last_local_plan_msg_->states[i].header.stamp.toSec());
      same_plan_index_ = previous_plan_index == current_plan_index_;

      break;
    }
  }

  if (use_twist_input_)
  {
    for (size_t i = 0; i < N_ + 1; i++)
    {
      int idx = current_plan_index_ - last_local_plan_msg_->plan_indices[0] + i;

      if (idx > last_local_plan_msg_->plan_indices.size() - 1)
      {
        ROS_ERROR_STREAM("Tail planner node couldn't find the correct ref leg plan at index: " << i);
        idx = last_local_plan_msg_->plan_indices.size() - 1;
      }

      ref_body_plan_.row(i) = quad_utils::bodyStateMsgToEigen(last_local_plan_msg_->ref_states[idx].body).transpose();
    }
  }
  else
  {
    for (size_t i = 0; i < N_ + 1; i++)
    {
      if (i + current_plan_index_ > body_plan_msg_->plan_indices.back())
      {
        ref_body_plan_.row(i) = quad_utils::bodyStateMsgToEigen(body_plan_msg_->states.back().body);
      }
      else
      {
        ref_body_plan_.row(i) = quad_utils::bodyStateMsgToEigen(body_plan_msg_->states[i + current_plan_index_].body);
      }
    }
  }

  for (size_t i = 0; i < N_ + 1; i++)
  {
    int idx = current_plan_index_ - last_local_plan_msg_->plan_indices[0] + i;

    if (idx > last_local_plan_msg_->plan_indices.size() - 1)
    {
      ROS_ERROR_STREAM("Tail planner node couldn't find the correct ref leg plan at index: " << i);
      idx = last_local_plan_msg_->plan_indices.size() - 1;
    }

    body_plan_.row(i) = quad_utils::bodyStateMsgToEigen(last_local_plan_msg_->states[idx].body).transpose();
    // ref_tail_plan_.row(i) << -body_plan_(i, 3), -body_plan_(i, 4), 0, 0;

    // Tail cannot move the body linearly so we should not use the ground height constraints here
    ref_ground_height_(i) = 2e-19;

    if (i < N_)
    {
      grf_plan_.row(i) = quad_utils::grfArrayMsgToEigen(last_local_plan_msg_->grfs[idx]).transpose();

      Eigen::VectorXd foot_positions(12);
      quad_utils::multiFootStateMsgToEigen(last_local_plan_msg_->states[idx].feet, foot_positions);
      for (size_t j = 0; j < 4; j++)
      {
        contact_schedule_[i][j] = bool(last_local_plan_msg_->grfs[idx].contact_states[j]);

        quad_utils::multiFootStateMsgToEigen(last_local_plan_msg_->states[idx].feet_body, foot_positions);
        foot_positions_body_.row(i) = foot_positions.transpose();
      }
    }
  }

  if (!tail_planner_->computeDistributedTailPlan(current_state_,
                                                 ref_body_plan_,
                                                 foot_positions_body_,
                                                 contact_schedule_,
                                                 tail_current_state_,
                                                 ref_tail_plan_,
                                                 body_plan_,
                                                 grf_plan_,
                                                 ref_ground_height_,
                                                 first_element_duration_,
                                                 same_plan_index_,
                                                 tail_plan_,
                                                 tail_torque_plan_))
    return;

  // Record computation time and update exponential filter
  double compute_time = 1000.0 * timer.reportSilent();

  // if (compute_time >= 1000.0 / update_rate_)
  // {
  //   ROS_WARN("TailPlanner took %5.3fms, exceeding %5.3fms allowed",
  //            compute_time, 1000.0 / update_rate_);
  // }
  // else
  // {
    // ROS_INFO("TailPlanner took %5.3f ms", compute_time);
  // };

  quad_msgs::LegCommandArray tail_plan_msg;
  tail_plan_msg.header.stamp = robot_state_msg_->header.stamp;

  for (size_t i = 0; i < N_; i++)
  {
    quad_msgs::LegCommand tail_msg;
    tail_msg.motor_commands.resize(2);

    tail_msg.motor_commands.at(0).pos_setpoint = tail_plan_(i, 0);
    tail_msg.motor_commands.at(0).vel_setpoint = tail_plan_(i, 2);
    tail_msg.motor_commands.at(0).torque_ff = tail_torque_plan_(i, 0);

    tail_msg.motor_commands.at(1).pos_setpoint = tail_plan_(i, 1);
    tail_msg.motor_commands.at(1).vel_setpoint = tail_plan_(i, 3);
    tail_msg.motor_commands.at(1).torque_ff = tail_torque_plan_(i, 1);

    // The first duration may vary
    if (i == 0)
    {
      tail_msg.header.stamp = tail_plan_msg.header.stamp;
    }
    else if (i == 1)
    {
      tail_msg.header.stamp = tail_plan_msg.header.stamp + ros::Duration(first_element_duration_);
    }
    else
    {
      tail_msg.header.stamp = tail_plan_msg.header.stamp + ros::Duration(first_element_duration_) + ros::Duration((i - 1) * dt_);
    }

    tail_plan_msg.leg_commands.push_back(tail_msg);
  }

  tail_plan_pub_.publish(tail_plan_msg);
}

void TailPlanner::spin()
{
  ros::Rate r(update_rate_);
  while (ros::ok())
  {
    ros::spinOnce();
    entrance_time_ = ros::Time::now();
    this->computeTailPlan();
    r.sleep();
  }
}