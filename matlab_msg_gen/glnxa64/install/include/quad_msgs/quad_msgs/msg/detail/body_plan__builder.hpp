// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/BodyPlan.idl
// generated code does not contain a copyright notice

#ifndef QUAD_MSGS__MSG__DETAIL__BODY_PLAN__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__BODY_PLAN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/body_plan__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_BodyPlan_grfs
{
public:
  explicit Init_BodyPlan_grfs(::quad_msgs::msg::BodyPlan & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::BodyPlan grfs(::quad_msgs::msg::BodyPlan::_grfs_type arg)
  {
    msg_.grfs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::BodyPlan msg_;
};

class Init_BodyPlan_states
{
public:
  explicit Init_BodyPlan_states(::quad_msgs::msg::BodyPlan & msg)
  : msg_(msg)
  {}
  Init_BodyPlan_grfs states(::quad_msgs::msg::BodyPlan::_states_type arg)
  {
    msg_.states = std::move(arg);
    return Init_BodyPlan_grfs(msg_);
  }

private:
  ::quad_msgs::msg::BodyPlan msg_;
};

class Init_BodyPlan_primitive_ids
{
public:
  explicit Init_BodyPlan_primitive_ids(::quad_msgs::msg::BodyPlan & msg)
  : msg_(msg)
  {}
  Init_BodyPlan_states primitive_ids(::quad_msgs::msg::BodyPlan::_primitive_ids_type arg)
  {
    msg_.primitive_ids = std::move(arg);
    return Init_BodyPlan_states(msg_);
  }

private:
  ::quad_msgs::msg::BodyPlan msg_;
};

class Init_BodyPlan_plan_indices
{
public:
  explicit Init_BodyPlan_plan_indices(::quad_msgs::msg::BodyPlan & msg)
  : msg_(msg)
  {}
  Init_BodyPlan_primitive_ids plan_indices(::quad_msgs::msg::BodyPlan::_plan_indices_type arg)
  {
    msg_.plan_indices = std::move(arg);
    return Init_BodyPlan_primitive_ids(msg_);
  }

private:
  ::quad_msgs::msg::BodyPlan msg_;
};

class Init_BodyPlan_header
{
public:
  Init_BodyPlan_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BodyPlan_plan_indices header(::quad_msgs::msg::BodyPlan::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BodyPlan_plan_indices(msg_);
  }

private:
  ::quad_msgs::msg::BodyPlan msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::BodyPlan>()
{
  return quad_msgs::msg::builder::Init_BodyPlan_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__BODY_PLAN__BUILDER_HPP_
