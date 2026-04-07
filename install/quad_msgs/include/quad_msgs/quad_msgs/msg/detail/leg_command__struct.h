// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/LegCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/leg_command.h"


#ifndef QUAD_MSGS__MSG__DETAIL__LEG_COMMAND__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__LEG_COMMAND__STRUCT_H_

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
// Member 'motor_commands'
#include "quad_msgs/msg/detail/motor_command__struct.h"

/// Struct defined in msg/LegCommand in the package quad_msgs.
/**
  * This is a message of motor commands for each joint on a quad leg.
  *
  * Accurate timing information is stored in the header
 */
typedef struct quad_msgs__msg__LegCommand
{
  std_msgs__msg__Header header;
  /// Stored as Abd, Hip, Knee
  quad_msgs__msg__MotorCommand__Sequence motor_commands;
} quad_msgs__msg__LegCommand;

// Struct for a sequence of quad_msgs__msg__LegCommand.
typedef struct quad_msgs__msg__LegCommand__Sequence
{
  quad_msgs__msg__LegCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__LegCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__LEG_COMMAND__STRUCT_H_
