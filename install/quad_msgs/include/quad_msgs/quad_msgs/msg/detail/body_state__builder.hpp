// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/BodyState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/body_state.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__BODY_STATE__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__BODY_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/body_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_BodyState_twist
{
public:
  explicit Init_BodyState_twist(::quad_msgs::msg::BodyState & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::BodyState twist(::quad_msgs::msg::BodyState::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::BodyState msg_;
};

class Init_BodyState_pose
{
public:
  explicit Init_BodyState_pose(::quad_msgs::msg::BodyState & msg)
  : msg_(msg)
  {}
  Init_BodyState_twist pose(::quad_msgs::msg::BodyState::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_BodyState_twist(msg_);
  }

private:
  ::quad_msgs::msg::BodyState msg_;
};

class Init_BodyState_traj_index
{
public:
  explicit Init_BodyState_traj_index(::quad_msgs::msg::BodyState & msg)
  : msg_(msg)
  {}
  Init_BodyState_pose traj_index(::quad_msgs::msg::BodyState::_traj_index_type arg)
  {
    msg_.traj_index = std::move(arg);
    return Init_BodyState_pose(msg_);
  }

private:
  ::quad_msgs::msg::BodyState msg_;
};

class Init_BodyState_header
{
public:
  Init_BodyState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BodyState_traj_index header(::quad_msgs::msg::BodyState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BodyState_traj_index(msg_);
  }

private:
  ::quad_msgs::msg::BodyState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::BodyState>()
{
  return quad_msgs::msg::builder::Init_BodyState_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__BODY_STATE__BUILDER_HPP_
