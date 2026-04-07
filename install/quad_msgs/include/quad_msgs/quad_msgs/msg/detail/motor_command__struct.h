// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/motor_command.h"


#ifndef QUAD_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_

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

/// Struct defined in msg/MotorCommand in the package quad_msgs.
/**
  * This is a message to hold the desired position, desired velocity, feedforward torques and control gains for a single joint on Quad
  *
  * Accurate timing information is stored in the header
 */
typedef struct quad_msgs__msg__MotorCommand
{
  std_msgs__msg__Header header;
  /// Commands
  /// Position command
  double pos_setpoint;
  /// Velocity command
  double vel_setpoint;
  /// Position setpoint gain
  float kp;
  /// Derivative setpoint gain
  float kd;
  /// Feedforward torque
  double torque_ff;
  /// Diagnostics
  /// Feedback position component
  double pos_component;
  /// Feedback velocity component
  double vel_component;
  /// Feedback total component
  double fb_component;
  /// Total effort
  double effort;
  /// Feedback to total ratio
  double fb_ratio;
} quad_msgs__msg__MotorCommand;

// Struct for a sequence of quad_msgs__msg__MotorCommand.
typedef struct quad_msgs__msg__MotorCommand__Sequence
{
  quad_msgs__msg__MotorCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__MotorCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_
