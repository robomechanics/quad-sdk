// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from quad_msgs:msg/RobotPlanDiagnostics.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "quad_msgs/msg/detail/robot_plan_diagnostics__rosidl_typesupport_introspection_c.h"
#include "quad_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "quad_msgs/msg/detail/robot_plan_diagnostics__functions.h"
#include "quad_msgs/msg/detail/robot_plan_diagnostics__struct.h"


// Include directives for member types
// Member `complexity_schedule`
// Member `element_times`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__RobotPlanDiagnostics_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  quad_msgs__msg__RobotPlanDiagnostics__init(message_memory);
}

void quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__RobotPlanDiagnostics_fini_function(void * message_memory)
{
  quad_msgs__msg__RobotPlanDiagnostics__fini(message_memory);
}

size_t quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__size_function__RobotPlanDiagnostics__complexity_schedule(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return member->size;
}

const void * quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__get_const_function__RobotPlanDiagnostics__complexity_schedule(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__get_function__RobotPlanDiagnostics__complexity_schedule(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__fetch_function__RobotPlanDiagnostics__complexity_schedule(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__get_const_function__RobotPlanDiagnostics__complexity_schedule(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__assign_function__RobotPlanDiagnostics__complexity_schedule(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__get_function__RobotPlanDiagnostics__complexity_schedule(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

bool quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__resize_function__RobotPlanDiagnostics__complexity_schedule(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  rosidl_runtime_c__uint32__Sequence__fini(member);
  return rosidl_runtime_c__uint32__Sequence__init(member, size);
}

size_t quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__size_function__RobotPlanDiagnostics__element_times(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__get_const_function__RobotPlanDiagnostics__element_times(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__get_function__RobotPlanDiagnostics__element_times(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__fetch_function__RobotPlanDiagnostics__element_times(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__get_const_function__RobotPlanDiagnostics__element_times(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__assign_function__RobotPlanDiagnostics__element_times(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__get_function__RobotPlanDiagnostics__element_times(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__resize_function__RobotPlanDiagnostics__element_times(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__RobotPlanDiagnostics_message_member_array[6] = {
  {
    "compute_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__RobotPlanDiagnostics, compute_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cost",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__RobotPlanDiagnostics, cost),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "iterations",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__RobotPlanDiagnostics, iterations),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "horizon_length",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__RobotPlanDiagnostics, horizon_length),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "complexity_schedule",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__RobotPlanDiagnostics, complexity_schedule),  // bytes offset in struct
    NULL,  // default value
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__size_function__RobotPlanDiagnostics__complexity_schedule,  // size() function pointer
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__get_const_function__RobotPlanDiagnostics__complexity_schedule,  // get_const(index) function pointer
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__get_function__RobotPlanDiagnostics__complexity_schedule,  // get(index) function pointer
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__fetch_function__RobotPlanDiagnostics__complexity_schedule,  // fetch(index, &value) function pointer
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__assign_function__RobotPlanDiagnostics__complexity_schedule,  // assign(index, value) function pointer
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__resize_function__RobotPlanDiagnostics__complexity_schedule  // resize(index) function pointer
  },
  {
    "element_times",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__RobotPlanDiagnostics, element_times),  // bytes offset in struct
    NULL,  // default value
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__size_function__RobotPlanDiagnostics__element_times,  // size() function pointer
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__get_const_function__RobotPlanDiagnostics__element_times,  // get_const(index) function pointer
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__get_function__RobotPlanDiagnostics__element_times,  // get(index) function pointer
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__fetch_function__RobotPlanDiagnostics__element_times,  // fetch(index, &value) function pointer
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__assign_function__RobotPlanDiagnostics__element_times,  // assign(index, value) function pointer
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__resize_function__RobotPlanDiagnostics__element_times  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__RobotPlanDiagnostics_message_members = {
  "quad_msgs__msg",  // message namespace
  "RobotPlanDiagnostics",  // message name
  6,  // number of fields
  sizeof(quad_msgs__msg__RobotPlanDiagnostics),
  false,  // has_any_key_member_
  quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__RobotPlanDiagnostics_message_member_array,  // message members
  quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__RobotPlanDiagnostics_init_function,  // function to initialize message memory (memory has to be allocated)
  quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__RobotPlanDiagnostics_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__RobotPlanDiagnostics_message_type_support_handle = {
  0,
  &quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__RobotPlanDiagnostics_message_members,
  get_message_typesupport_handle_function,
  &quad_msgs__msg__RobotPlanDiagnostics__get_type_hash,
  &quad_msgs__msg__RobotPlanDiagnostics__get_type_description,
  &quad_msgs__msg__RobotPlanDiagnostics__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_quad_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, quad_msgs, msg, RobotPlanDiagnostics)() {
  if (!quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__RobotPlanDiagnostics_message_type_support_handle.typesupport_identifier) {
    quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__RobotPlanDiagnostics_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &quad_msgs__msg__RobotPlanDiagnostics__rosidl_typesupport_introspection_c__RobotPlanDiagnostics_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
