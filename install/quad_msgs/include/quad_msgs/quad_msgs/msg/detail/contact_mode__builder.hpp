// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/ContactMode.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/contact_mode.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__CONTACT_MODE__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__CONTACT_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/contact_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_ContactMode_leg_contacts
{
public:
  explicit Init_ContactMode_leg_contacts(::quad_msgs::msg::ContactMode & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::ContactMode leg_contacts(::quad_msgs::msg::ContactMode::_leg_contacts_type arg)
  {
    msg_.leg_contacts = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::ContactMode msg_;
};

class Init_ContactMode_header
{
public:
  Init_ContactMode_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ContactMode_leg_contacts header(::quad_msgs::msg::ContactMode::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ContactMode_leg_contacts(msg_);
  }

private:
  ::quad_msgs::msg::ContactMode msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::ContactMode>()
{
  return quad_msgs::msg::builder::Init_ContactMode_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__CONTACT_MODE__BUILDER_HPP_
