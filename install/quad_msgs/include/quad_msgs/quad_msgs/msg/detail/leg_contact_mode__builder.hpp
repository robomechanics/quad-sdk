// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/LegContactMode.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/leg_contact_mode.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__LEG_CONTACT_MODE__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__LEG_CONTACT_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/leg_contact_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_LegContactMode_contact_forces
{
public:
  explicit Init_LegContactMode_contact_forces(::quad_msgs::msg::LegContactMode & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::LegContactMode contact_forces(::quad_msgs::msg::LegContactMode::_contact_forces_type arg)
  {
    msg_.contact_forces = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::LegContactMode msg_;
};

class Init_LegContactMode_contact_state
{
public:
  explicit Init_LegContactMode_contact_state(::quad_msgs::msg::LegContactMode & msg)
  : msg_(msg)
  {}
  Init_LegContactMode_contact_forces contact_state(::quad_msgs::msg::LegContactMode::_contact_state_type arg)
  {
    msg_.contact_state = std::move(arg);
    return Init_LegContactMode_contact_forces(msg_);
  }

private:
  ::quad_msgs::msg::LegContactMode msg_;
};

class Init_LegContactMode_contact_prob
{
public:
  explicit Init_LegContactMode_contact_prob(::quad_msgs::msg::LegContactMode & msg)
  : msg_(msg)
  {}
  Init_LegContactMode_contact_state contact_prob(::quad_msgs::msg::LegContactMode::_contact_prob_type arg)
  {
    msg_.contact_prob = std::move(arg);
    return Init_LegContactMode_contact_state(msg_);
  }

private:
  ::quad_msgs::msg::LegContactMode msg_;
};

class Init_LegContactMode_header
{
public:
  Init_LegContactMode_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LegContactMode_contact_prob header(::quad_msgs::msg::LegContactMode::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LegContactMode_contact_prob(msg_);
  }

private:
  ::quad_msgs::msg::LegContactMode msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::LegContactMode>()
{
  return quad_msgs::msg::builder::Init_LegContactMode_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__LEG_CONTACT_MODE__BUILDER_HPP_
