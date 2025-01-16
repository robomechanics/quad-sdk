// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/MultiFootState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/multi_foot_state.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_STATE__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/multi_foot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_MultiFootState_feet
{
public:
  explicit Init_MultiFootState_feet(::quad_msgs::msg::MultiFootState & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::MultiFootState feet(::quad_msgs::msg::MultiFootState::_feet_type arg)
  {
    msg_.feet = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::MultiFootState msg_;
};

class Init_MultiFootState_traj_index
{
public:
  explicit Init_MultiFootState_traj_index(::quad_msgs::msg::MultiFootState & msg)
  : msg_(msg)
  {}
  Init_MultiFootState_feet traj_index(::quad_msgs::msg::MultiFootState::_traj_index_type arg)
  {
    msg_.traj_index = std::move(arg);
    return Init_MultiFootState_feet(msg_);
  }

private:
  ::quad_msgs::msg::MultiFootState msg_;
};

class Init_MultiFootState_header
{
public:
  Init_MultiFootState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultiFootState_traj_index header(::quad_msgs::msg::MultiFootState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MultiFootState_traj_index(msg_);
  }

private:
  ::quad_msgs::msg::MultiFootState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::MultiFootState>()
{
  return quad_msgs::msg::builder::Init_MultiFootState_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_STATE__BUILDER_HPP_
