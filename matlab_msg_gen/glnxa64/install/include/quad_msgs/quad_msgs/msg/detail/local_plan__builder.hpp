// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/LocalPlan.idl
// generated code does not contain a copyright notice

#ifndef QUAD_MSGS__MSG__DETAIL__LOCAL_PLAN__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__LOCAL_PLAN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/local_plan__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_LocalPlan_global_plan_timestamp
{
public:
  explicit Init_LocalPlan_global_plan_timestamp(::quad_msgs::msg::LocalPlan & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::LocalPlan global_plan_timestamp(::quad_msgs::msg::LocalPlan::_global_plan_timestamp_type arg)
  {
    msg_.global_plan_timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::LocalPlan msg_;
};

class Init_LocalPlan_plan_indices
{
public:
  explicit Init_LocalPlan_plan_indices(::quad_msgs::msg::LocalPlan & msg)
  : msg_(msg)
  {}
  Init_LocalPlan_global_plan_timestamp plan_indices(::quad_msgs::msg::LocalPlan::_plan_indices_type arg)
  {
    msg_.plan_indices = std::move(arg);
    return Init_LocalPlan_global_plan_timestamp(msg_);
  }

private:
  ::quad_msgs::msg::LocalPlan msg_;
};

class Init_LocalPlan_grfs
{
public:
  explicit Init_LocalPlan_grfs(::quad_msgs::msg::LocalPlan & msg)
  : msg_(msg)
  {}
  Init_LocalPlan_plan_indices grfs(::quad_msgs::msg::LocalPlan::_grfs_type arg)
  {
    msg_.grfs = std::move(arg);
    return Init_LocalPlan_plan_indices(msg_);
  }

private:
  ::quad_msgs::msg::LocalPlan msg_;
};

class Init_LocalPlan_states
{
public:
  explicit Init_LocalPlan_states(::quad_msgs::msg::LocalPlan & msg)
  : msg_(msg)
  {}
  Init_LocalPlan_grfs states(::quad_msgs::msg::LocalPlan::_states_type arg)
  {
    msg_.states = std::move(arg);
    return Init_LocalPlan_grfs(msg_);
  }

private:
  ::quad_msgs::msg::LocalPlan msg_;
};

class Init_LocalPlan_header
{
public:
  Init_LocalPlan_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocalPlan_states header(::quad_msgs::msg::LocalPlan::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LocalPlan_states(msg_);
  }

private:
  ::quad_msgs::msg::LocalPlan msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::LocalPlan>()
{
  return quad_msgs::msg::builder::Init_LocalPlan_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__LOCAL_PLAN__BUILDER_HPP_
