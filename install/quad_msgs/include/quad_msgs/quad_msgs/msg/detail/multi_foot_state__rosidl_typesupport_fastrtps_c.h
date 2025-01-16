// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from quad_msgs:msg/MultiFootState.idl
// generated code does not contain a copyright notice
#ifndef QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "quad_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "quad_msgs/msg/detail/multi_foot_state__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quad_msgs
bool cdr_serialize_quad_msgs__msg__MultiFootState(
  const quad_msgs__msg__MultiFootState * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quad_msgs
bool cdr_deserialize_quad_msgs__msg__MultiFootState(
  eprosima::fastcdr::Cdr &,
  quad_msgs__msg__MultiFootState * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quad_msgs
size_t get_serialized_size_quad_msgs__msg__MultiFootState(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quad_msgs
size_t max_serialized_size_quad_msgs__msg__MultiFootState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quad_msgs
bool cdr_serialize_key_quad_msgs__msg__MultiFootState(
  const quad_msgs__msg__MultiFootState * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quad_msgs
size_t get_serialized_size_key_quad_msgs__msg__MultiFootState(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quad_msgs
size_t max_serialized_size_key_quad_msgs__msg__MultiFootState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quad_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, quad_msgs, msg, MultiFootState)();

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
