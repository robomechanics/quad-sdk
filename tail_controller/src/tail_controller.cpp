#include "tail_controller/tail_controller.h"

TailController::TailController(ros::NodeHandle nh)
{
  nh_ = nh;

  nh.param<int>("/tail_controller/tail_type", tail_type_, 0);

  switch (tail_type_)
  {
  case NONE:
    // Leg controller
    param_ns_ = "leg";
    break;
  case CENTRALIZED:
    // Centralized tail controller
    param_ns_ = "centralized_tail";
    break;
  case DISTRIBUTED:
    // Distributed tail controller
    param_ns_ = "distributed_tail";
    break;
  case DECENTRALIZED:
    // Decentralized tail controller
    param_ns_ = "decentralized_tail";
    break;
  default:
    param_ns_ = "leg";
    break;
  }

  ros::param::get("/nmpc_controller/" + param_ns_ + "/step_length", dt_);

  // Get rosparams
  std::string tail_plan_topic, tail_control_topic, robot_state_topic;
  spirit_utils::loadROSParam(nh_, "/topics/control/tail_plan", tail_plan_topic);
  spirit_utils::loadROSParam(nh_, "/topics/control/tail_command", tail_control_topic);
  spirit_utils::loadROSParam(nh_, "/topics/state/ground_truth", robot_state_topic);
  spirit_utils::loadROSParam(nh_, "tail_controller/controller_update_rate", update_rate_);
  spirit_utils::loadROSParam(nh_, "tail_controller/roll_kp", roll_kp_);
  spirit_utils::loadROSParam(nh_, "tail_controller/roll_kd", roll_kd_);
  spirit_utils::loadROSParam(nh_, "tail_controller/pitch_kp", pitch_kp_);
  spirit_utils::loadROSParam(nh_, "tail_controller/pitch_kd", pitch_kd_);

  // Setup pubs and subs
  tail_control_pub_ = nh_.advertise<spirit_msgs::LegCommand>(tail_control_topic, 1);
  tail_plan_sub_ = nh_.subscribe(tail_plan_topic, 1, &TailController::tailPlanCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic, 1, &TailController::robotStateCallback, this);
}

void TailController::tailPlanCallback(const spirit_msgs::LegCommandArray::ConstPtr &msg)
{
  last_tail_plan_msg_ = msg;
}

void TailController::robotStateCallback(const spirit_msgs::RobotState::ConstPtr &msg)
{
  // Make sure the data is actually populated
  if (msg->feet.feet.empty() || msg->joints.position.empty())
    return;

  robot_state_msg_ = msg;
}

void TailController::publishTailCommand()
{
  if (robot_state_msg_ == NULL)
  {
    return;
  }

  current_state_ = spirit_utils::bodyStateMsgToEigen(robot_state_msg_->body);
  tail_current_state_ = spirit_utils::odomMsgToEigenForTail(*robot_state_msg_);

  spirit_msgs::LegCommand msg;
  msg.motor_commands.resize(2);

  if (robot_state_msg_ != NULL && param_ns_ == "decentralized_tail")
  {
    msg.motor_commands.at(0).pos_setpoint = 5 * current_state_(3);
    msg.motor_commands.at(0).vel_setpoint = 5 * current_state_(9);
    msg.motor_commands.at(0).torque_ff = 0;
    msg.motor_commands.at(0).kp = roll_kp_;
    msg.motor_commands.at(0).kd = roll_kd_;

    msg.motor_commands.at(1).pos_setpoint = 5 * current_state_(4);
    msg.motor_commands.at(1).vel_setpoint = 5 * current_state_(10);
    msg.motor_commands.at(1).torque_ff = 0;
    msg.motor_commands.at(1).kp = pitch_kp_;
    msg.motor_commands.at(1).kd = pitch_kd_;

    if (abs(msg.motor_commands.at(0).pos_setpoint) > 1.5)
    {
      if (msg.motor_commands.at(0).pos_setpoint > 0)
      {
        msg.motor_commands.at(0).vel_setpoint = std::min(msg.motor_commands.at(0).vel_setpoint, 0.);
        msg.motor_commands.at(0).torque_ff = std::min(msg.motor_commands.at(0).torque_ff, 0.);
      }
      else
      {
        msg.motor_commands.at(0).vel_setpoint = std::max(msg.motor_commands.at(0).vel_setpoint, 0.);
        msg.motor_commands.at(0).torque_ff = std::max(msg.motor_commands.at(0).torque_ff, 0.);
      }
      msg.motor_commands.at(0).pos_setpoint = std::min(std::max(-1.5, msg.motor_commands.at(0).pos_setpoint), 1.5);
    }

    if (abs(msg.motor_commands.at(1).pos_setpoint) > 1.5)
    {
      if (msg.motor_commands.at(1).pos_setpoint > 0)
      {
        msg.motor_commands.at(1).vel_setpoint = std::min(msg.motor_commands.at(1).vel_setpoint, 0.);
        msg.motor_commands.at(1).torque_ff = std::min(msg.motor_commands.at(1).torque_ff, 0.);
      }
      else
      {
        msg.motor_commands.at(1).vel_setpoint = std::max(msg.motor_commands.at(1).vel_setpoint, 0.);
        msg.motor_commands.at(1).torque_ff = std::max(msg.motor_commands.at(1).torque_ff, 0.);
      }
      msg.motor_commands.at(1).pos_setpoint = std::min(std::max(-1.5, msg.motor_commands.at(1).pos_setpoint), 1.5);
    }
  }
  else if (last_tail_plan_msg_ == NULL || robot_state_msg_ == NULL)
  {
    msg.motor_commands.at(0).pos_setpoint = 0;
    msg.motor_commands.at(0).vel_setpoint = 0;
    msg.motor_commands.at(0).torque_ff = 0;
    msg.motor_commands.at(0).kp = roll_kp_;
    msg.motor_commands.at(0).kd = roll_kd_;

    msg.motor_commands.at(1).pos_setpoint = 0;
    msg.motor_commands.at(1).vel_setpoint = 0;
    msg.motor_commands.at(1).torque_ff = 0;
    msg.motor_commands.at(1).kp = pitch_kp_;
    msg.motor_commands.at(1).kd = pitch_kd_;
  }
  else
  {
    double t_interp;
    int current_plan_index;
    double t_now = ros::Time::now().toSec();

    //  Tail plan starts now
    double current_time = t_now - last_tail_plan_msg_->header.stamp.toSec();
    current_plan_index = std::floor((current_time - last_tail_plan_msg_->dt_first_step) / dt_) + 1;
    if (current_plan_index == 0)
    {
      t_interp = current_time / last_tail_plan_msg_->dt_first_step;
    }
    else
    {
      t_interp = std::fmod((current_time - last_tail_plan_msg_->dt_first_step), dt_) / dt_;
    }

    // //  Tail plan starts at the same point as local plan
    // if ((t_now < last_tail_plan_msg_->leg_commands.front().header.stamp.toSec()) ||
    //     (t_now > last_tail_plan_msg_->leg_commands.back().header.stamp.toSec()))
    // {
    //   ROS_ERROR("Tail controller node couldn't find the correct ref state!");
    // }

    // // Interpolate the local plan to get the reference state and ff GRF
    // for (size_t i = 0; i < last_tail_plan_msg_->leg_commands.size() - 1; i++)
    // {
    //   if ((t_now >= last_tail_plan_msg_->leg_commands[i].header.stamp.toSec()) &&
    //       (t_now < last_tail_plan_msg_->leg_commands[i + 1].header.stamp.toSec()))
    //   {
    //     t_interp = (t_now - last_tail_plan_msg_->leg_commands[i].header.stamp.toSec()) /
    //                (last_tail_plan_msg_->leg_commands[i + 1].header.stamp.toSec() -
    //                 last_tail_plan_msg_->leg_commands[i].header.stamp.toSec());

    //     current_plan_index = i;

    //     break;
    //   }
    // }

    if (current_plan_index + 1 > last_tail_plan_msg_->leg_commands.size() - 1)
    {
      msg.motor_commands.at(0).pos_setpoint = last_tail_plan_msg_->leg_commands.back().motor_commands[0].pos_setpoint;
      msg.motor_commands.at(0).vel_setpoint = last_tail_plan_msg_->leg_commands.back().motor_commands[0].vel_setpoint;
      msg.motor_commands.at(0).torque_ff = last_tail_plan_msg_->leg_commands.back().motor_commands[0].torque_ff;
      msg.motor_commands.at(0).kp = roll_kp_;
      msg.motor_commands.at(0).kd = roll_kd_;

      msg.motor_commands.at(1).pos_setpoint = last_tail_plan_msg_->leg_commands.back().motor_commands[1].pos_setpoint;
      msg.motor_commands.at(1).vel_setpoint = last_tail_plan_msg_->leg_commands.back().motor_commands[1].vel_setpoint;
      msg.motor_commands.at(1).torque_ff = last_tail_plan_msg_->leg_commands.back().motor_commands[1].torque_ff;
      msg.motor_commands.at(1).kp = pitch_kp_;
      msg.motor_commands.at(1).kd = pitch_kd_;
    }
    else
    {
      msg.motor_commands.at(0).pos_setpoint = math_utils::lerp(last_tail_plan_msg_->leg_commands[current_plan_index].motor_commands[0].pos_setpoint,
                                                               last_tail_plan_msg_->leg_commands[current_plan_index + 1].motor_commands[0].pos_setpoint,
                                                               t_interp);
      msg.motor_commands.at(0).vel_setpoint = math_utils::lerp(last_tail_plan_msg_->leg_commands[current_plan_index].motor_commands[0].vel_setpoint,
                                                               last_tail_plan_msg_->leg_commands[current_plan_index + 1].motor_commands[0].vel_setpoint,
                                                               t_interp);
      msg.motor_commands.at(0).torque_ff = last_tail_plan_msg_->leg_commands[current_plan_index].motor_commands[0].torque_ff;
      msg.motor_commands.at(0).kp = roll_kp_;
      msg.motor_commands.at(0).kd = roll_kd_;

      msg.motor_commands.at(1).pos_setpoint = math_utils::lerp(last_tail_plan_msg_->leg_commands[current_plan_index].motor_commands[1].pos_setpoint,
                                                               last_tail_plan_msg_->leg_commands[current_plan_index + 1].motor_commands[1].pos_setpoint,
                                                               t_interp);
      msg.motor_commands.at(1).vel_setpoint = math_utils::lerp(last_tail_plan_msg_->leg_commands[current_plan_index].motor_commands[1].vel_setpoint,
                                                               last_tail_plan_msg_->leg_commands[current_plan_index + 1].motor_commands[1].vel_setpoint,
                                                               t_interp);
      msg.motor_commands.at(1).torque_ff = last_tail_plan_msg_->leg_commands[current_plan_index].motor_commands[1].torque_ff;
      msg.motor_commands.at(1).kp = pitch_kp_;
      msg.motor_commands.at(1).kd = pitch_kd_;

      // ROS_INFO_STREAM_THROTTLE(0.1, "current_plan_index: " << current_plan_index << "pos: " << msg.motor_commands.at(0).pos_setpoint << ", " << msg.motor_commands.at(1).pos_setpoint);

      if (abs(msg.motor_commands.at(0).pos_setpoint) > 1.5)
      {
        if (msg.motor_commands.at(0).pos_setpoint > 0)
        {
          msg.motor_commands.at(0).vel_setpoint = std::min(msg.motor_commands.at(0).vel_setpoint, 0.);
          msg.motor_commands.at(0).torque_ff = std::min(msg.motor_commands.at(0).torque_ff, 0.);
        }
        else
        {
          msg.motor_commands.at(0).vel_setpoint = std::max(msg.motor_commands.at(0).vel_setpoint, 0.);
          msg.motor_commands.at(0).torque_ff = std::max(msg.motor_commands.at(0).torque_ff, 0.);
        }
        msg.motor_commands.at(0).pos_setpoint = std::min(std::max(-1.5, msg.motor_commands.at(0).pos_setpoint), 1.5);
      }

      if (abs(msg.motor_commands.at(1).pos_setpoint) > 1.5)
      {
        if (msg.motor_commands.at(1).pos_setpoint > 0)
        {
          msg.motor_commands.at(1).vel_setpoint = std::min(msg.motor_commands.at(1).vel_setpoint, 0.);
          msg.motor_commands.at(1).torque_ff = std::min(msg.motor_commands.at(1).torque_ff, 0.);
        }
        else
        {
          msg.motor_commands.at(1).vel_setpoint = std::max(msg.motor_commands.at(1).vel_setpoint, 0.);
          msg.motor_commands.at(1).torque_ff = std::max(msg.motor_commands.at(1).torque_ff, 0.);
        }
        msg.motor_commands.at(1).pos_setpoint = std::min(std::max(-1.5, msg.motor_commands.at(1).pos_setpoint), 1.5);
      }
    }
  }

  for (size_t i = 0; i < 2; i++)
  {
    double pos_component = msg.motor_commands.at(i).kp * (msg.motor_commands.at(i).pos_setpoint - tail_current_state_(i));
    double vel_component = msg.motor_commands.at(i).kd * (msg.motor_commands.at(i).vel_setpoint - tail_current_state_(2 + i));
    double fb_component = pos_component + vel_component;
    double effort = fb_component + msg.motor_commands.at(i).torque_ff;
    double fb_ratio = abs(fb_component) / (abs(fb_component) + abs(msg.motor_commands.at(i).torque_ff));

    msg.motor_commands.at(i).pos_component = pos_component;
    msg.motor_commands.at(i).vel_component = vel_component;
    msg.motor_commands.at(i).fb_component = fb_component;
    msg.motor_commands.at(i).effort = effort;
    msg.motor_commands.at(i).fb_ratio = fb_ratio;
  }

  msg.header.stamp = ros::Time::now();
  tail_control_pub_.publish(msg);
}

void TailController::spin()
{
  double start_time = ros::Time::now().toSec();
  ros::Rate r(update_rate_);
  while (ros::ok())
  {
    ros::spinOnce();
    this->publishTailCommand();
    r.sleep();
  }
}