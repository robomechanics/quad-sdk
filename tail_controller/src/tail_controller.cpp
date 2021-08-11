#include "tail_controller/tail_controller.h"

TailController::TailController(ros::NodeHandle nh)
{
  nh_ = nh;

  nh.param<int>("/tail_controller/tail_type", tail_type_, 0);

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

  ros::param::get("/nmpc_controller/" + param_ns + "/step_length", dt_);

  // Get rosparams
  std::string tail_plan_topic, tail_control_topic;
  spirit_utils::loadROSParam(nh_, "/topics/control/tail_plan", tail_plan_topic);
  spirit_utils::loadROSParam(nh_, "/topics/control/tail_command", tail_control_topic);
  spirit_utils::loadROSParam(nh_, "tail_controller/controller_update_rate", update_rate_);
  spirit_utils::loadROSParam(nh_, "tail_controller/roll_kp", roll_kp_);
  spirit_utils::loadROSParam(nh_, "tail_controller/roll_kd", roll_kd_);
  spirit_utils::loadROSParam(nh_, "tail_controller/pitch_kp", pitch_kp_);
  spirit_utils::loadROSParam(nh_, "tail_controller/pitch_kd", pitch_kd_);

  // Setup pubs and subs
  tail_control_pub_ = nh_.advertise<spirit_msgs::LegCommand>(tail_control_topic, 1);
  tail_plan_sub_ = nh_.subscribe(tail_plan_topic, 1, &TailController::tailPlanCallback, this);
}

void TailController::tailPlanCallback(const spirit_msgs::LegCommandArray::ConstPtr &msg)
{
  last_tail_plan_msg_ = msg;
}

void TailController::publishTailCommand()
{
  spirit_msgs::LegCommand msg;
  msg.motor_commands.resize(2);

  if (last_tail_plan_msg_ == NULL)
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

    msg.header.stamp = ros::Time::now();
    tail_control_pub_.publish(msg);
  }
  else
  {
    double current_time = spirit_utils::getDurationSinceTime(last_tail_plan_msg_->header.stamp);
    int current_plan_index = spirit_utils::getPlanIndex(last_tail_plan_msg_->header.stamp, dt_);
    double t_interp = std::fmod(current_time, dt_) / dt_;

    if (current_plan_index == 0)
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

      msg.header.stamp = ros::Time::now();
      tail_control_pub_.publish(msg);
    }
  }
}

void TailController::spin()
{
  double start_time = ros::Time::now().toSec();
  ros::Rate r(update_rate_);
  while (ros::ok())
  {
    this->publishTailCommand();
    ros::spinOnce();
    r.sleep();
  }
}