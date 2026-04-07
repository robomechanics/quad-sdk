// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/FootState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/foot_state.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__FOOT_STATE__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__FOOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/foot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_FootState_contact
{
public:
  explicit Init_FootState_contact(::quad_msgs::msg::FootState & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::FootState contact(::quad_msgs::msg::FootState::_contact_type arg)
  {
    msg_.contact = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::FootState msg_;
};

class Init_FootState_acceleration
{
public:
  explicit Init_FootState_acceleration(::quad_msgs::msg::FootState & msg)
  : msg_(msg)
  {}
  Init_FootState_contact acceleration(::quad_msgs::msg::FootState::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_FootState_contact(msg_);
  }

private:
  ::quad_msgs::msg::FootState msg_;
};

class Init_FootState_velocity
{
public:
  explicit Init_FootState_velocity(::quad_msgs::msg::FootState & msg)
  : msg_(msg)
  {}
  Init_FootState_acceleration velocity(::quad_msgs::msg::FootState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_FootState_acceleration(msg_);
  }

private:
  ::quad_msgs::msg::FootState msg_;
};

class Init_FootState_position
{
public:
  explicit Init_FootState_position(::quad_msgs::msg::FootState & msg)
  : msg_(msg)
  {}
  Init_FootState_velocity position(::quad_msgs::msg::FootState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_FootState_velocity(msg_);
  }

private:
  ::quad_msgs::msg::FootState msg_;
};

class Init_FootState_traj_index
{
public:
  explicit Init_FootState_traj_index(::quad_msgs::msg::FootState & msg)
  : msg_(msg)
  {}
  Init_FootState_position traj_index(::quad_msgs::msg::FootState::_traj_index_type arg)
  {
    msg_.traj_index = std::move(arg);
    return Init_FootState_position(msg_);
  }

private:
  ::quad_msgs::msg::FootState msg_;
};

class Init_FootState_header
{
public:
  Init_FootState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FootState_traj_index header(::quad_msgs::msg::FootState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FootState_traj_index(msg_);
  }

private:
  ::quad_msgs::msg::FootState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::FootState>()
{
  return quad_msgs::msg::builder::Init_FootState_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__FOOT_STATE__BUILDER_HPP_
