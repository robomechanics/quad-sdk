// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/LegCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/leg_command.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__LEG_COMMAND__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__LEG_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/leg_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_LegCommand_motor_commands
{
public:
  explicit Init_LegCommand_motor_commands(::quad_msgs::msg::LegCommand & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::LegCommand motor_commands(::quad_msgs::msg::LegCommand::_motor_commands_type arg)
  {
    msg_.motor_commands = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::LegCommand msg_;
};

class Init_LegCommand_header
{
public:
  Init_LegCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LegCommand_motor_commands header(::quad_msgs::msg::LegCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LegCommand_motor_commands(msg_);
  }

private:
  ::quad_msgs::msg::LegCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::LegCommand>()
{
  return quad_msgs::msg::builder::Init_LegCommand_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__LEG_COMMAND__BUILDER_HPP_
