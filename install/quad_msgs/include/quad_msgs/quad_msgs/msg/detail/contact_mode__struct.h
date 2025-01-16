// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/ContactMode.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/contact_mode.h"


#ifndef QUAD_MSGS__MSG__DETAIL__CONTACT_MODE__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__CONTACT_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'leg_contacts'
#include "quad_msgs/msg/detail/leg_contact_mode__struct.h"

/// Struct defined in msg/ContactMode in the package quad_msgs.
/**
  * This is a message to hold contact states of the robot
 */
typedef struct quad_msgs__msg__ContactMode
{
  std_msgs__msg__Header header;
  /// FL Bl FR BR
  quad_msgs__msg__LegContactMode__Sequence leg_contacts;
} quad_msgs__msg__ContactMode;

// Struct for a sequence of quad_msgs__msg__ContactMode.
typedef struct quad_msgs__msg__ContactMode__Sequence
{
  quad_msgs__msg__ContactMode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__ContactMode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__CONTACT_MODE__STRUCT_H_
