#include "tail_controller/tail_planner.h"

TailPlanner::TailPlanner(ros::NodeHandle nh)
{
  nh_ = nh;

  // Get rosparams
  std::string body_plan_topic, robot_state_topic, local_plan_topic, tail_plan_topic;
  spirit_utils::loadROSParam(nh_, "/topics/control/tail_plan", tail_plan_topic);
  spirit_utils::loadROSParam(nh_, "/topics/global_plan", body_plan_topic);
  spirit_utils::loadROSParam(nh_, "/topics/state/ground_truth", robot_state_topic);
  spirit_utils::loadROSParam(nh_, "/topics/local_plan", local_plan_topic);
  spirit_utils::loadROSParam(nh_, "/tail_controller/planner_update_rate", update_rate_);

  // Setup pubs and subs
  tail_plan_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>(tail_plan_topic, 1);
  body_plan_sub_ = nh_.subscribe(body_plan_topic, 1, &TailPlanner::robotPlanCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic, 1, &TailPlanner::robotStateCallback, this);
  local_plan_sub_ = nh_.subscribe(local_plan_topic, 1, &TailPlanner::localPlanCallback, this);

  nh.param<int>("/tail_controller/tail_type", tail_type_, 0);

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

  contact_schedule_.resize(N_);
  for (size_t i = 0; i < N_; i++)
  {
    contact_schedule_[i].resize(4);
  }

  ref_body_plan_ = Eigen::MatrixXd::Zero(N_, 12);

  foot_positions_body_ = Eigen::MatrixXd::Zero(N_, 12);

  body_plan_ = Eigen::MatrixXd::Zero(N_, 12);

  grf_plan_ = Eigen::MatrixXd::Zero(N_, 12);

  tail_plan_ = Eigen::MatrixXd::Zero(N_, 4);

  tail_torque_plan_ = Eigen::MatrixXd::Zero(N_, 2);
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

void TailPlanner::computeTailPlan()
{
  if (last_local_plan_msg_ == NULL || body_plan_msg_ == NULL || robot_state_msg_ == NULL)
    return;

  current_state_ = spirit_utils::odomMsgToEigen(robot_state_msg_->body);

  tail_current_state_ = spirit_utils::odomMsgToEigenForTail(*robot_state_msg_);
  ref_tail_plan_ = Eigen::MatrixXd::Zero(N_, 4);

  int current_plan_index = last_local_plan_msg_->plan_indices[0];

  for (size_t i = 0; i < N_; i++)
  {
    if (i + current_plan_index > body_plan_msg_->plan_indices.back())
    {
      ref_body_plan_.row(i) = spirit_utils::odomMsgToEigen(body_plan_msg_->states.back().body);
    }
    else
    {
      ref_body_plan_.row(i) = spirit_utils::odomMsgToEigen(body_plan_msg_->states[i + current_plan_index].body);
    }

    Eigen::VectorXd foot_positions(12);
    spirit_utils::multiFootStateMsgToEigen(last_local_plan_msg_->states[i].feet, foot_positions);
    for (size_t j = 0; j < 4; j++)
    {
      contact_schedule_[i][j] = last_local_plan_msg_->grfs[i].contact_states[j];

      foot_positions_body_.block(i, j * 3, 1, 3) = foot_positions.segment(j * 3, 3).transpose() -
                                                   ref_body_plan_.block(i, 0, 1, 3);
    }

    body_plan_.row(i) = spirit_utils::odomMsgToEigen(last_local_plan_msg_->states[i].body).transpose();
    grf_plan_.row(i) = spirit_utils::grfArrayMsgToEigen(last_local_plan_msg_->grfs[i]).transpose();
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