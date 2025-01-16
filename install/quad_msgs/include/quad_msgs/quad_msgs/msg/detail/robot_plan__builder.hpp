// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/RobotPlan.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/robot_plan.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/robot_plan__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotPlan_diagnostics
{
public:
  explicit Init_RobotPlan_diagnostics(::quad_msgs::msg::RobotPlan & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::RobotPlan diagnostics(::quad_msgs::msg::RobotPlan::_diagnostics_type arg)
  {
    msg_.diagnostics = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::RobotPlan msg_;
};

class Init_RobotPlan_compute_time
{
public:
  explicit Init_RobotPlan_compute_time(::quad_msgs::msg::RobotPlan & msg)
  : msg_(msg)
  {}
  Init_RobotPlan_diagnostics compute_time(::quad_msgs::msg::RobotPlan::_compute_time_type arg)
  {
    msg_.compute_time = std::move(arg);
    return Init_RobotPlan_diagnostics(msg_);
  }

private:
  ::quad_msgs::msg::RobotPlan msg_;
};

class Init_RobotPlan_primitive_ids
{
public:
  explicit Init_RobotPlan_primitive_ids(::quad_msgs::msg::RobotPlan & msg)
  : msg_(msg)
  {}
  Init_RobotPlan_compute_time primitive_ids(::quad_msgs::msg::RobotPlan::_primitive_ids_type arg)
  {
    msg_.primitive_ids = std::move(arg);
    return Init_RobotPlan_compute_time(msg_);
  }

private:
  ::quad_msgs::msg::RobotPlan msg_;
};

class Init_RobotPlan_plan_indices
{
public:
  explicit Init_RobotPlan_plan_indices(::quad_msgs::msg::RobotPlan & msg)
  : msg_(msg)
  {}
  Init_RobotPlan_primitive_ids plan_indices(::quad_msgs::msg::RobotPlan::_plan_indices_type arg)
  {
    msg_.plan_indices = std::move(arg);
    return Init_RobotPlan_primitive_ids(msg_);
  }

private:
  ::quad_msgs::msg::RobotPlan msg_;
};

class Init_RobotPlan_grfs
{
public:
  explicit Init_RobotPlan_grfs(::quad_msgs::msg::RobotPlan & msg)
  : msg_(msg)
  {}
  Init_RobotPlan_plan_indices grfs(::quad_msgs::msg::RobotPlan::_grfs_type arg)
  {
    msg_.grfs = std::move(arg);
    return Init_RobotPlan_plan_indices(msg_);
  }

private:
  ::quad_msgs::msg::RobotPlan msg_;
};

class Init_RobotPlan_states
{
public:
  explicit Init_RobotPlan_states(::quad_msgs::msg::RobotPlan & msg)
  : msg_(msg)
  {}
  Init_RobotPlan_grfs states(::quad_msgs::msg::RobotPlan::_states_type arg)
  {
    msg_.states = std::move(arg);
    return Init_RobotPlan_grfs(msg_);
  }

private:
  ::quad_msgs::msg::RobotPlan msg_;
};

class Init_RobotPlan_state_timestamp
{
public:
  explicit Init_RobotPlan_state_timestamp(::quad_msgs::msg::RobotPlan & msg)
  : msg_(msg)
  {}
  Init_RobotPlan_states state_timestamp(::quad_msgs::msg::RobotPlan::_state_timestamp_type arg)
  {
    msg_.state_timestamp = std::move(arg);
    return Init_RobotPlan_states(msg_);
  }

private:
  ::quad_msgs::msg::RobotPlan msg_;
};

class Init_RobotPlan_global_plan_timestamp
{
public:
  explicit Init_RobotPlan_global_plan_timestamp(::quad_msgs::msg::RobotPlan & msg)
  : msg_(msg)
  {}
  Init_RobotPlan_state_timestamp global_plan_timestamp(::quad_msgs::msg::RobotPlan::_global_plan_timestamp_type arg)
  {
    msg_.global_plan_timestamp = std::move(arg);
    return Init_RobotPlan_state_timestamp(msg_);
  }

private:
  ::quad_msgs::msg::RobotPlan msg_;
};

class Init_RobotPlan_header
{
public:
  Init_RobotPlan_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotPlan_global_plan_timestamp header(::quad_msgs::msg::RobotPlan::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobotPlan_global_plan_timestamp(msg_);
  }

private:
  ::quad_msgs::msg::RobotPlan msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::RobotPlan>()
{
  return quad_msgs::msg::builder::Init_RobotPlan_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN__BUILDER_HPP_
