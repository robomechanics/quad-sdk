// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from quad_msgs:msg/LegCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/leg_command.h"


#ifndef QUAD_MSGS__MSG__DETAIL__LEG_COMMAND__FUNCTIONS_H_
#define QUAD_MSGS__MSG__DETAIL__LEG_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "quad_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "quad_msgs/msg/detail/leg_command__struct.h"

/// Initialize msg/LegCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * quad_msgs__msg__LegCommand
 * )) before or use
 * quad_msgs__msg__LegCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
bool
quad_msgs__msg__LegCommand__init(quad_msgs__msg__LegCommand * msg);

/// Finalize msg/LegCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
void
quad_msgs__msg__LegCommand__fini(quad_msgs__msg__LegCommand * msg);

/// Create msg/LegCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * quad_msgs__msg__LegCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
quad_msgs__msg__LegCommand *
quad_msgs__msg__LegCommand__create(void);

/// Destroy msg/LegCommand message.
/**
 * It calls
 * quad_msgs__msg__LegCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
void
quad_msgs__msg__LegCommand__destroy(quad_msgs__msg__LegCommand * msg);

/// Check for msg/LegCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
bool
quad_msgs__msg__LegCommand__are_equal(const quad_msgs__msg__LegCommand * lhs, const quad_msgs__msg__LegCommand * rhs);

/// Copy a msg/LegCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
bool
quad_msgs__msg__LegCommand__copy(
  const quad_msgs__msg__LegCommand * input,
  quad_msgs__msg__LegCommand * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
const rosidl_type_hash_t *
quad_msgs__msg__LegCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
const rosidl_runtime_c__type_description__TypeDescription *
quad_msgs__msg__LegCommand__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
const rosidl_runtime_c__type_description__TypeSource *
quad_msgs__msg__LegCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
quad_msgs__msg__LegCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/LegCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * quad_msgs__msg__LegCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
bool
quad_msgs__msg__LegCommand__Sequence__init(quad_msgs__msg__LegCommand__Sequence * array, size_t size);

/// Finalize array of msg/LegCommand messages.
/**
 * It calls
 * quad_msgs__msg__LegCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
void
quad_msgs__msg__LegCommand__Sequence__fini(quad_msgs__msg__LegCommand__Sequence * array);

/// Create array of msg/LegCommand messages.
/**
 * It allocates the memory for the array and calls
 * quad_msgs__msg__LegCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
quad_msgs__msg__LegCommand__Sequence *
quad_msgs__msg__LegCommand__Sequence__create(size_t size);

/// Destroy array of msg/LegCommand messages.
/**
 * It calls
 * quad_msgs__msg__LegCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
void
quad_msgs__msg__LegCommand__Sequence__destroy(quad_msgs__msg__LegCommand__Sequence * array);

/// Check for msg/LegCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
bool
quad_msgs__msg__LegCommand__Sequence__are_equal(const quad_msgs__msg__LegCommand__Sequence * lhs, const quad_msgs__msg__LegCommand__Sequence * rhs);

/// Copy an array of msg/LegCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
bool
quad_msgs__msg__LegCommand__Sequence__copy(
  const quad_msgs__msg__LegCommand__Sequence * input,
  quad_msgs__msg__LegCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__LEG_COMMAND__FUNCTIONS_H_
