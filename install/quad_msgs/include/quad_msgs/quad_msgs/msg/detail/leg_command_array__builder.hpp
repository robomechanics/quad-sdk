// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/LegCommandArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/leg_command_array.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__LEG_COMMAND_ARRAY__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__LEG_COMMAND_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/leg_command_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_LegCommandArray_leg_commands
{
public:
  explicit Init_LegCommandArray_leg_commands(::quad_msgs::msg::LegCommandArray & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::LegCommandArray leg_commands(::quad_msgs::msg::LegCommandArray::_leg_commands_type arg)
  {
    msg_.leg_commands = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::LegCommandArray msg_;
};

class Init_LegCommandArray_header
{
public:
  Init_LegCommandArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LegCommandArray_leg_commands header(::quad_msgs::msg::LegCommandArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LegCommandArray_leg_commands(msg_);
  }

private:
  ::quad_msgs::msg::LegCommandArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::LegCommandArray>()
{
  return quad_msgs::msg::builder::Init_LegCommandArray_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__LEG_COMMAND_ARRAY__BUILDER_HPP_
