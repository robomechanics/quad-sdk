// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from quad_msgs:msg/MultiFootState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/multi_foot_state.h"


#ifndef QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_STATE__FUNCTIONS_H_
#define QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_STATE__FUNCTIONS_H_

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

#include "quad_msgs/msg/detail/multi_foot_state__struct.h"

/// Initialize msg/MultiFootState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * quad_msgs__msg__MultiFootState
 * )) before or use
 * quad_msgs__msg__MultiFootState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
bool
quad_msgs__msg__MultiFootState__init(quad_msgs__msg__MultiFootState * msg);

/// Finalize msg/MultiFootState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
void
quad_msgs__msg__MultiFootState__fini(quad_msgs__msg__MultiFootState * msg);

/// Create msg/MultiFootState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * quad_msgs__msg__MultiFootState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
quad_msgs__msg__MultiFootState *
quad_msgs__msg__MultiFootState__create(void);

/// Destroy msg/MultiFootState message.
/**
 * It calls
 * quad_msgs__msg__MultiFootState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
void
quad_msgs__msg__MultiFootState__destroy(quad_msgs__msg__MultiFootState * msg);

/// Check for msg/MultiFootState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
bool
quad_msgs__msg__MultiFootState__are_equal(const quad_msgs__msg__MultiFootState * lhs, const quad_msgs__msg__MultiFootState * rhs);

/// Copy a msg/MultiFootState message.
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
quad_msgs__msg__MultiFootState__copy(
  const quad_msgs__msg__MultiFootState * input,
  quad_msgs__msg__MultiFootState * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
const rosidl_type_hash_t *
quad_msgs__msg__MultiFootState__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
const rosidl_runtime_c__type_description__TypeDescription *
quad_msgs__msg__MultiFootState__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
const rosidl_runtime_c__type_description__TypeSource *
quad_msgs__msg__MultiFootState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
quad_msgs__msg__MultiFootState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/MultiFootState messages.
/**
 * It allocates the memory for the number of elements and calls
 * quad_msgs__msg__MultiFootState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
bool
quad_msgs__msg__MultiFootState__Sequence__init(quad_msgs__msg__MultiFootState__Sequence * array, size_t size);

/// Finalize array of msg/MultiFootState messages.
/**
 * It calls
 * quad_msgs__msg__MultiFootState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
void
quad_msgs__msg__MultiFootState__Sequence__fini(quad_msgs__msg__MultiFootState__Sequence * array);

/// Create array of msg/MultiFootState messages.
/**
 * It allocates the memory for the array and calls
 * quad_msgs__msg__MultiFootState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
quad_msgs__msg__MultiFootState__Sequence *
quad_msgs__msg__MultiFootState__Sequence__create(size_t size);

/// Destroy array of msg/MultiFootState messages.
/**
 * It calls
 * quad_msgs__msg__MultiFootState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
void
quad_msgs__msg__MultiFootState__Sequence__destroy(quad_msgs__msg__MultiFootState__Sequence * array);

/// Check for msg/MultiFootState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
bool
quad_msgs__msg__MultiFootState__Sequence__are_equal(const quad_msgs__msg__MultiFootState__Sequence * lhs, const quad_msgs__msg__MultiFootState__Sequence * rhs);

/// Copy an array of msg/MultiFootState messages.
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
quad_msgs__msg__MultiFootState__Sequence__copy(
  const quad_msgs__msg__MultiFootState__Sequence * input,
  quad_msgs__msg__MultiFootState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_STATE__FUNCTIONS_H_
