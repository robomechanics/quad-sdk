// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/FootPlanDiscrete.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/foot_plan_discrete.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__FOOT_PLAN_DISCRETE__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__FOOT_PLAN_DISCRETE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/foot_plan_discrete__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_FootPlanDiscrete_footholds
{
public:
  explicit Init_FootPlanDiscrete_footholds(::quad_msgs::msg::FootPlanDiscrete & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::FootPlanDiscrete footholds(::quad_msgs::msg::FootPlanDiscrete::_footholds_type arg)
  {
    msg_.footholds = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::FootPlanDiscrete msg_;
};

class Init_FootPlanDiscrete_header
{
public:
  Init_FootPlanDiscrete_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FootPlanDiscrete_footholds header(::quad_msgs::msg::FootPlanDiscrete::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FootPlanDiscrete_footholds(msg_);
  }

private:
  ::quad_msgs::msg::FootPlanDiscrete msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::FootPlanDiscrete>()
{
  return quad_msgs::msg::builder::Init_FootPlanDiscrete_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__FOOT_PLAN_DISCRETE__BUILDER_HPP_
