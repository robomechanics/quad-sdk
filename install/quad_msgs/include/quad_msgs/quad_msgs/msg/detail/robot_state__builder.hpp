// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/robot_state.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotState_feet
{
public:
  explicit Init_RobotState_feet(::quad_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::RobotState feet(::quad_msgs::msg::RobotState::_feet_type arg)
  {
    msg_.feet = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::RobotState msg_;
};

class Init_RobotState_joints
{
public:
  explicit Init_RobotState_joints(::quad_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_feet joints(::quad_msgs::msg::RobotState::_joints_type arg)
  {
    msg_.joints = std::move(arg);
    return Init_RobotState_feet(msg_);
  }

private:
  ::quad_msgs::msg::RobotState msg_;
};

class Init_RobotState_body
{
public:
  explicit Init_RobotState_body(::quad_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_joints body(::quad_msgs::msg::RobotState::_body_type arg)
  {
    msg_.body = std::move(arg);
    return Init_RobotState_joints(msg_);
  }

private:
  ::quad_msgs::msg::RobotState msg_;
};

class Init_RobotState_traj_index
{
public:
  explicit Init_RobotState_traj_index(::quad_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_body traj_index(::quad_msgs::msg::RobotState::_traj_index_type arg)
  {
    msg_.traj_index = std::move(arg);
    return Init_RobotState_body(msg_);
  }

private:
  ::quad_msgs::msg::RobotState msg_;
};

class Init_RobotState_header
{
public:
  Init_RobotState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_traj_index header(::quad_msgs::msg::RobotState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobotState_traj_index(msg_);
  }

private:
  ::quad_msgs::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::RobotState>()
{
  return quad_msgs::msg::builder::Init_RobotState_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
