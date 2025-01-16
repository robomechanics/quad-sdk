// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/MultiFootPlanContinuous.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/multi_foot_plan_continuous.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_PLAN_CONTINUOUS__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_PLAN_CONTINUOUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/multi_foot_plan_continuous__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_MultiFootPlanContinuous_states
{
public:
  explicit Init_MultiFootPlanContinuous_states(::quad_msgs::msg::MultiFootPlanContinuous & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::MultiFootPlanContinuous states(::quad_msgs::msg::MultiFootPlanContinuous::_states_type arg)
  {
    msg_.states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::MultiFootPlanContinuous msg_;
};

class Init_MultiFootPlanContinuous_header
{
public:
  Init_MultiFootPlanContinuous_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultiFootPlanContinuous_states header(::quad_msgs::msg::MultiFootPlanContinuous::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MultiFootPlanContinuous_states(msg_);
  }

private:
  ::quad_msgs::msg::MultiFootPlanContinuous msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::MultiFootPlanContinuous>()
{
  return quad_msgs::msg::builder::Init_MultiFootPlanContinuous_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_PLAN_CONTINUOUS__BUILDER_HPP_
