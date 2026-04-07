// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/LegCommandArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/leg_command_array.h"


#ifndef QUAD_MSGS__MSG__DETAIL__LEG_COMMAND_ARRAY__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__LEG_COMMAND_ARRAY__STRUCT_H_

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
// Member 'leg_commands'
#include "quad_msgs/msg/detail/leg_command__struct.h"

/// Struct defined in msg/LegCommandArray in the package quad_msgs.
/**
  * This is a message of leg commands for each leg on quad. The order is in FL, BL, FR, BR.
  *
  * Accurate timing information is stored in the header
 */
typedef struct quad_msgs__msg__LegCommandArray
{
  std_msgs__msg__Header header;
  /// FL, BL, FR, BR
  quad_msgs__msg__LegCommand__Sequence leg_commands;
} quad_msgs__msg__LegCommandArray;

// Struct for a sequence of quad_msgs__msg__LegCommandArray.
typedef struct quad_msgs__msg__LegCommandArray__Sequence
{
  quad_msgs__msg__LegCommandArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__LegCommandArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__LEG_COMMAND_ARRAY__STRUCT_H_
