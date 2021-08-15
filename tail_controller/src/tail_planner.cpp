#include "tail_controller/tail_planner.h"

TailPlanner::TailPlanner(ros::NodeHandle nh)
{
  nh_ = nh;

  // Get rosparams
  std::string body_plan_topic, robot_state_topic, local_plan_topic, tail_plan_topic, cmd_vel_topic;
  spirit_utils::loadROSParam(nh_, "/topics/control/tail_plan", tail_plan_topic);
  spirit_utils::loadROSParam(nh_, "/topics/global_plan", body_plan_topic);
  spirit_utils::loadROSParam(nh_, "/topics/state/ground_truth", robot_state_topic);
  spirit_utils::loadROSParam(nh_, "/topics/local_plan", local_plan_topic);
  spirit_utils::loadROSParam(nh_, "/tail_controller/planner_update_rate", update_rate_);
  nh_.param<std::string>("/topics/cmd_vel", cmd_vel_topic, "/cmd_vel");

  // Setup pubs and subs
  tail_plan_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>(tail_plan_topic, 1);
  body_plan_sub_ = nh_.subscribe(body_plan_topic, 1, &TailPlanner::robotPlanCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic, 1, &TailPlanner::robotStateCallback, this);
  local_plan_sub_ = nh_.subscribe(local_plan_topic, 1, &TailPlanner::localPlanCallback, this);
  cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic, 1, &TailPlanner::cmdVelCallback, this);

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

  body_plan_ = Eigen::MatrixXd::Zero(N_, 12);

  grf_plan_ = Eigen::MatrixXd::Zero(N_, 12);

  tail_plan_ = Eigen::MatrixXd::Zero(N_, 4);

  tail_torque_plan_ = Eigen::MatrixXd::Zero(N_, 2);

  // Initialize twist input
  cmd_vel_.resize(6);

  // Zero the velocity to start
  std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0);

  // Assume we know the step height
  z_des_ = 0.5;
}

void TailPlanner::robotPlanCallback(const spirit_msgs::RobotPlan::ConstPtr &msg)
{
  body_plan_msg_ = msg;
}

void TailPlanner::robotStateCallback(const spirit_msgs::RobotState::ConstPtr &msg)
{
  // Make sure the data is actually populated
  if (msg->feet.feet.empty() || msg->joints.position.empty())
    return;

  robot_state_msg_ = msg;
}

void TailPlanner::localPlanCallback(const spirit_msgs::RobotPlan::ConstPtr &msg)
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

void TailPlanner::computeTailPlan()
{
  if (last_local_plan_msg_ == NULL || (body_plan_msg_ == NULL && !use_twist_input_) || robot_state_msg_ == NULL)
    return;

  current_state_ = spirit_utils::odomMsgToEigen(robot_state_msg_->body);

  tail_current_state_ = spirit_utils::odomMsgToEigenForTail(*robot_state_msg_);
  ref_tail_plan_ = Eigen::MatrixXd::Zero(N_ + 1, 4);

  int current_plan_index = last_local_plan_msg_->plan_indices[0];

  // current_state_ = spirit_utils::odomMsgToEigen(last_local_plan_msg_->states[0].body).transpose();

  if (use_twist_input_)
  {
    ref_body_plan_.setZero();

    // Check that we have recent twist data, otherwise set cmd_vel to zero
    ros::Duration time_elapsed_since_msg = ros::Time::now() - last_cmd_vel_msg_time_;
    if (time_elapsed_since_msg.toSec() > last_cmd_vel_msg_time_max_)
    {
      std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0);
      ROS_WARN_THROTTLE(1.0, "No cmd_vel data, setting twist cmd_vel to zero");
    }

    // Adaptive body height, assume we know step height
    if (abs(current_foot_positions_world_(2) - current_state_(2)) >= 0.4 ||
    abs(current_foot_positions_world_(5) - current_state_(2)) >= 0.4 || 
    abs(current_foot_positions_world_(8) - current_state_(2)) >= 0.4 || 
    abs(current_foot_positions_world_(11) - current_state_(2)) >= 0.4)
    {
      z_des_ = 0.3;
    }

    // Integrate to get full body plan
    ref_body_plan_(0, 0) = current_state_[0];
    ref_body_plan_(0, 1) = current_state_[1];
    ref_body_plan_(0, 2) = z_des_;
    ref_body_plan_(0, 3) = 0;
    ref_body_plan_(0, 4) = 0;
    ref_body_plan_(0, 5) = current_state_[5];
    ref_body_plan_(0, 6) = cmd_vel_[0];
    ref_body_plan_(0, 7) = cmd_vel_[1];
    ref_body_plan_(0, 8) = cmd_vel_[2];
    ref_body_plan_(0, 9) = cmd_vel_[3];
    ref_body_plan_(0, 10) = cmd_vel_[4];
    ref_body_plan_(0, 11) = cmd_vel_[5];

    for (size_t i = 1; i < N_ + 1; i++)
    {
      std::vector<double> current_cmd_vel = cmd_vel_;

      double yaw = ref_body_plan_(i - 1, 5);
      current_cmd_vel[0] = cmd_vel_[0] * cos(yaw) - cmd_vel_[1] * sin(yaw);
      current_cmd_vel[1] = cmd_vel_[0] * sin(yaw) + cmd_vel_[1] * cos(yaw);

      for (int j = 0; j < 6; j++)
      {
        ref_body_plan_(i, j) = ref_body_plan_(i - 1, j) + current_cmd_vel[j] * dt_;
        ref_body_plan_(i, j + 6) = (current_cmd_vel[j]);
      }
    }
  }
  else
  {
    for (size_t i = 0; i < N_ + 1; i++)
    {
      if (i + current_plan_index > body_plan_msg_->plan_indices.back())
      {
        ref_body_plan_.row(i) = spirit_utils::odomMsgToEigen(body_plan_msg_->states.back().body);
      }
      else
      {
        ref_body_plan_.row(i) = spirit_utils::odomMsgToEigen(body_plan_msg_->states[i + current_plan_index].body);
      }
    }
  }

  for (size_t i = 0; i < N_; i++)
  {
    body_plan_.row(i) = spirit_utils::odomMsgToEigen(last_local_plan_msg_->states[i].body).transpose();
    grf_plan_.row(i) = spirit_utils::grfArrayMsgToEigen(last_local_plan_msg_->grfs[i]).transpose();

    Eigen::VectorXd foot_positions(12);
    spirit_utils::multiFootStateMsgToEigen(last_local_plan_msg_->states[i].feet, foot_positions);
    for (size_t j = 0; j < 4; j++)
    {
      contact_schedule_[i][j] = last_local_plan_msg_->grfs[i].contact_states[j];

      foot_positions_body_.block(i, j * 3, 1, 3) = foot_positions.segment(j * 3, 3).transpose() -
                                                   body_plan_.block(i, 0, 1, 3);
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
                                                 tail_plan_,
                                                 tail_torque_plan_))
    return;

  spirit_msgs::LegCommandArray tail_plan_msg;
  ros::Time timestamp = ros::Time::now();
  tail_plan_msg.header.stamp = timestamp;

  for (size_t i = 0; i < N_; i++)
  {
    spirit_msgs::LegCommand tail_msg;
    tail_msg.motor_commands.resize(2);

    tail_msg.motor_commands.at(0).pos_setpoint = tail_plan_(i, 0);
    tail_msg.motor_commands.at(0).vel_setpoint = tail_plan_(i, 2);
    tail_msg.motor_commands.at(0).torque_ff = tail_torque_plan_(i, 0);

    tail_msg.motor_commands.at(1).pos_setpoint = tail_plan_(i, 1);
    tail_msg.motor_commands.at(1).vel_setpoint = tail_plan_(i, 3);
    tail_msg.motor_commands.at(1).torque_ff = tail_torque_plan_(i, 1);

    tail_plan_msg.leg_commands.push_back(tail_msg);
  }

  tail_plan_pub_.publish(tail_plan_msg);
}

void TailPlanner::spin()
{
  double start_time = ros::Time::now().toSec();
  ros::Rate r(update_rate_);
  while (ros::ok())
  {
    this->computeTailPlan();
    ros::spinOnce();
    r.sleep();
  }
}