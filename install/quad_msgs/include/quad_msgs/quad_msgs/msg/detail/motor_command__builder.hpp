// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/motor_command.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/motor_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorCommand_fb_ratio
{
public:
  explicit Init_MotorCommand_fb_ratio(::quad_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::MotorCommand fb_ratio(::quad_msgs::msg::MotorCommand::_fb_ratio_type arg)
  {
    msg_.fb_ratio = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_effort
{
public:
  explicit Init_MotorCommand_effort(::quad_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_fb_ratio effort(::quad_msgs::msg::MotorCommand::_effort_type arg)
  {
    msg_.effort = std::move(arg);
    return Init_MotorCommand_fb_ratio(msg_);
  }

private:
  ::quad_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_fb_component
{
public:
  explicit Init_MotorCommand_fb_component(::quad_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_effort fb_component(::quad_msgs::msg::MotorCommand::_fb_component_type arg)
  {
    msg_.fb_component = std::move(arg);
    return Init_MotorCommand_effort(msg_);
  }

private:
  ::quad_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_vel_component
{
public:
  explicit Init_MotorCommand_vel_component(::quad_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_fb_component vel_component(::quad_msgs::msg::MotorCommand::_vel_component_type arg)
  {
    msg_.vel_component = std::move(arg);
    return Init_MotorCommand_fb_component(msg_);
  }

private:
  ::quad_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_pos_component
{
public:
  explicit Init_MotorCommand_pos_component(::quad_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_vel_component pos_component(::quad_msgs::msg::MotorCommand::_pos_component_type arg)
  {
    msg_.pos_component = std::move(arg);
    return Init_MotorCommand_vel_component(msg_);
  }

private:
  ::quad_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_torque_ff
{
public:
  explicit Init_MotorCommand_torque_ff(::quad_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_pos_component torque_ff(::quad_msgs::msg::MotorCommand::_torque_ff_type arg)
  {
    msg_.torque_ff = std::move(arg);
    return Init_MotorCommand_pos_component(msg_);
  }

private:
  ::quad_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_kd
{
public:
  explicit Init_MotorCommand_kd(::quad_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_torque_ff kd(::quad_msgs::msg::MotorCommand::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_MotorCommand_torque_ff(msg_);
  }

private:
  ::quad_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_kp
{
public:
  explicit Init_MotorCommand_kp(::quad_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_kd kp(::quad_msgs::msg::MotorCommand::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_MotorCommand_kd(msg_);
  }

private:
  ::quad_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_vel_setpoint
{
public:
  explicit Init_MotorCommand_vel_setpoint(::quad_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_kp vel_setpoint(::quad_msgs::msg::MotorCommand::_vel_setpoint_type arg)
  {
    msg_.vel_setpoint = std::move(arg);
    return Init_MotorCommand_kp(msg_);
  }

private:
  ::quad_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_pos_setpoint
{
public:
  explicit Init_MotorCommand_pos_setpoint(::quad_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_vel_setpoint pos_setpoint(::quad_msgs::msg::MotorCommand::_pos_setpoint_type arg)
  {
    msg_.pos_setpoint = std::move(arg);
    return Init_MotorCommand_vel_setpoint(msg_);
  }

private:
  ::quad_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_header
{
public:
  Init_MotorCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_pos_setpoint header(::quad_msgs::msg::MotorCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotorCommand_pos_setpoint(msg_);
  }

private:
  ::quad_msgs::msg::MotorCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::MotorCommand>()
{
  return quad_msgs::msg::builder::Init_MotorCommand_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
