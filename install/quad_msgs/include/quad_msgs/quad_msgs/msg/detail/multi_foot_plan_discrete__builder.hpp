// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/MultiFootPlanDiscrete.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/multi_foot_plan_discrete.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_PLAN_DISCRETE__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_PLAN_DISCRETE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/multi_foot_plan_discrete__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_MultiFootPlanDiscrete_feet
{
public:
  explicit Init_MultiFootPlanDiscrete_feet(::quad_msgs::msg::MultiFootPlanDiscrete & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::MultiFootPlanDiscrete feet(::quad_msgs::msg::MultiFootPlanDiscrete::_feet_type arg)
  {
    msg_.feet = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::MultiFootPlanDiscrete msg_;
};

class Init_MultiFootPlanDiscrete_header
{
public:
  Init_MultiFootPlanDiscrete_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultiFootPlanDiscrete_feet header(::quad_msgs::msg::MultiFootPlanDiscrete::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MultiFootPlanDiscrete_feet(msg_);
  }

private:
  ::quad_msgs::msg::MultiFootPlanDiscrete msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::MultiFootPlanDiscrete>()
{
  return quad_msgs::msg::builder::Init_MultiFootPlanDiscrete_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_PLAN_DISCRETE__BUILDER_HPP_
