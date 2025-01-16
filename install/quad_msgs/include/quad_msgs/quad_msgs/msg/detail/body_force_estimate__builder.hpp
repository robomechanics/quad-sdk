// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/BodyForceEstimate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/body_force_estimate.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__BODY_FORCE_ESTIMATE__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__BODY_FORCE_ESTIMATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/body_force_estimate__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_BodyForceEstimate_joint_torques
{
public:
  explicit Init_BodyForceEstimate_joint_torques(::quad_msgs::msg::BodyForceEstimate & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::BodyForceEstimate joint_torques(::quad_msgs::msg::BodyForceEstimate::_joint_torques_type arg)
  {
    msg_.joint_torques = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::BodyForceEstimate msg_;
};

class Init_BodyForceEstimate_header
{
public:
  Init_BodyForceEstimate_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BodyForceEstimate_joint_torques header(::quad_msgs::msg::BodyForceEstimate::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BodyForceEstimate_joint_torques(msg_);
  }

private:
  ::quad_msgs::msg::BodyForceEstimate msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::BodyForceEstimate>()
{
  return quad_msgs::msg::builder::Init_BodyForceEstimate_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__BODY_FORCE_ESTIMATE__BUILDER_HPP_
