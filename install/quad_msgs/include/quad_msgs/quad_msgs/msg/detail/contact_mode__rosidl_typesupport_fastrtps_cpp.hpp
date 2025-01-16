// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from quad_msgs:msg/ContactMode.idl
// generated code does not contain a copyright notice

#ifndef QUAD_MSGS__MSG__DETAIL__CONTACT_MODE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define QUAD_MSGS__MSG__DETAIL__CONTACT_MODE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "quad_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "quad_msgs/msg/detail/contact_mode__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace quad_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_quad_msgs
cdr_serialize(
  const quad_msgs::msg::ContactMode & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_quad_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  quad_msgs::msg::ContactMode & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_quad_msgs
get_serialized_size(
  const quad_msgs::msg::ContactMode & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_quad_msgs
max_serialized_size_ContactMode(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_quad_msgs
cdr_serialize_key(
  const quad_msgs::msg::ContactMode & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_quad_msgs
get_serialized_size_key(
  const quad_msgs::msg::ContactMode & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_quad_msgs
max_serialized_size_key_ContactMode(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace quad_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_quad_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, quad_msgs, msg, ContactMode)();

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__CONTACT_MODE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
