// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quad_msgs:msg/RobotPlan.idl
// generated code does not contain a copyright notice
#include "quad_msgs/msg/detail/robot_plan__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `global_plan_timestamp`
// Member `state_timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `states`
#include "quad_msgs/msg/detail/robot_state__functions.h"
// Member `grfs`
#include "quad_msgs/msg/detail/grf_array__functions.h"
// Member `plan_indices`
// Member `primitive_ids`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `diagnostics`
#include "quad_msgs/msg/detail/robot_plan_diagnostics__functions.h"

bool
quad_msgs__msg__RobotPlan__init(quad_msgs__msg__RobotPlan * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    quad_msgs__msg__RobotPlan__fini(msg);
    return false;
  }
  // global_plan_timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->global_plan_timestamp)) {
    quad_msgs__msg__RobotPlan__fini(msg);
    return false;
  }
  // state_timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->state_timestamp)) {
    quad_msgs__msg__RobotPlan__fini(msg);
    return false;
  }
  // states
  if (!quad_msgs__msg__RobotState__Sequence__init(&msg->states, 0)) {
    quad_msgs__msg__RobotPlan__fini(msg);
    return false;
  }
  // grfs
  if (!quad_msgs__msg__GRFArray__Sequence__init(&msg->grfs, 0)) {
    quad_msgs__msg__RobotPlan__fini(msg);
    return false;
  }
  // plan_indices
  if (!rosidl_runtime_c__uint32__Sequence__init(&msg->plan_indices, 0)) {
    quad_msgs__msg__RobotPlan__fini(msg);
    return false;
  }
  // primitive_ids
  if (!rosidl_runtime_c__uint32__Sequence__init(&msg->primitive_ids, 0)) {
    quad_msgs__msg__RobotPlan__fini(msg);
    return false;
  }
  // compute_time
  // diagnostics
  if (!quad_msgs__msg__RobotPlanDiagnostics__init(&msg->diagnostics)) {
    quad_msgs__msg__RobotPlan__fini(msg);
    return false;
  }
  return true;
}

void
quad_msgs__msg__RobotPlan__fini(quad_msgs__msg__RobotPlan * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // global_plan_timestamp
  builtin_interfaces__msg__Time__fini(&msg->global_plan_timestamp);
  // state_timestamp
  builtin_interfaces__msg__Time__fini(&msg->state_timestamp);
  // states
  quad_msgs__msg__RobotState__Sequence__fini(&msg->states);
  // grfs
  quad_msgs__msg__GRFArray__Sequence__fini(&msg->grfs);
  // plan_indices
  rosidl_runtime_c__uint32__Sequence__fini(&msg->plan_indices);
  // primitive_ids
  rosidl_runtime_c__uint32__Sequence__fini(&msg->primitive_ids);
  // compute_time
  // diagnostics
  quad_msgs__msg__RobotPlanDiagnostics__fini(&msg->diagnostics);
}

bool
quad_msgs__msg__RobotPlan__are_equal(const quad_msgs__msg__RobotPlan * lhs, const quad_msgs__msg__RobotPlan * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // global_plan_timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->global_plan_timestamp), &(rhs->global_plan_timestamp)))
  {
    return false;
  }
  // state_timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->state_timestamp), &(rhs->state_timestamp)))
  {
    return false;
  }
  // states
  if (!quad_msgs__msg__RobotState__Sequence__are_equal(
      &(lhs->states), &(rhs->states)))
  {
    return false;
  }
  // grfs
  if (!quad_msgs__msg__GRFArray__Sequence__are_equal(
      &(lhs->grfs), &(rhs->grfs)))
  {
    return false;
  }
  // plan_indices
  if (!rosidl_runtime_c__uint32__Sequence__are_equal(
      &(lhs->plan_indices), &(rhs->plan_indices)))
  {
    return false;
  }
  // primitive_ids
  if (!rosidl_runtime_c__uint32__Sequence__are_equal(
      &(lhs->primitive_ids), &(rhs->primitive_ids)))
  {
    return false;
  }
  // compute_time
  if (lhs->compute_time != rhs->compute_time) {
    return false;
  }
  // diagnostics
  if (!quad_msgs__msg__RobotPlanDiagnostics__are_equal(
      &(lhs->diagnostics), &(rhs->diagnostics)))
  {
    return false;
  }
  return true;
}

bool
quad_msgs__msg__RobotPlan__copy(
  const quad_msgs__msg__RobotPlan * input,
  quad_msgs__msg__RobotPlan * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // global_plan_timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->global_plan_timestamp), &(output->global_plan_timestamp)))
  {
    return false;
  }
  // state_timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->state_timestamp), &(output->state_timestamp)))
  {
    return false;
  }
  // states
  if (!quad_msgs__msg__RobotState__Sequence__copy(
      &(input->states), &(output->states)))
  {
    return false;
  }
  // grfs
  if (!quad_msgs__msg__GRFArray__Sequence__copy(
      &(input->grfs), &(output->grfs)))
  {
    return false;
  }
  // plan_indices
  if (!rosidl_runtime_c__uint32__Sequence__copy(
      &(input->plan_indices), &(output->plan_indices)))
  {
    return false;
  }
  // primitive_ids
  if (!rosidl_runtime_c__uint32__Sequence__copy(
      &(input->primitive_ids), &(output->primitive_ids)))
  {
    return false;
  }
  // compute_time
  output->compute_time = input->compute_time;
  // diagnostics
  if (!quad_msgs__msg__RobotPlanDiagnostics__copy(
      &(input->diagnostics), &(output->diagnostics)))
  {
    return false;
  }
  return true;
}

quad_msgs__msg__RobotPlan *
quad_msgs__msg__RobotPlan__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__RobotPlan * msg = (quad_msgs__msg__RobotPlan *)allocator.allocate(sizeof(quad_msgs__msg__RobotPlan), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quad_msgs__msg__RobotPlan));
  bool success = quad_msgs__msg__RobotPlan__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quad_msgs__msg__RobotPlan__destroy(quad_msgs__msg__RobotPlan * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quad_msgs__msg__RobotPlan__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quad_msgs__msg__RobotPlan__Sequence__init(quad_msgs__msg__RobotPlan__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__RobotPlan * data = NULL;

  if (size) {
    data = (quad_msgs__msg__RobotPlan *)allocator.zero_allocate(size, sizeof(quad_msgs__msg__RobotPlan), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quad_msgs__msg__RobotPlan__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quad_msgs__msg__RobotPlan__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
quad_msgs__msg__RobotPlan__Sequence__fini(quad_msgs__msg__RobotPlan__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      quad_msgs__msg__RobotPlan__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

quad_msgs__msg__RobotPlan__Sequence *
quad_msgs__msg__RobotPlan__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__RobotPlan__Sequence * array = (quad_msgs__msg__RobotPlan__Sequence *)allocator.allocate(sizeof(quad_msgs__msg__RobotPlan__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quad_msgs__msg__RobotPlan__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quad_msgs__msg__RobotPlan__Sequence__destroy(quad_msgs__msg__RobotPlan__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quad_msgs__msg__RobotPlan__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quad_msgs__msg__RobotPlan__Sequence__are_equal(const quad_msgs__msg__RobotPlan__Sequence * lhs, const quad_msgs__msg__RobotPlan__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quad_msgs__msg__RobotPlan__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quad_msgs__msg__RobotPlan__Sequence__copy(
  const quad_msgs__msg__RobotPlan__Sequence * input,
  quad_msgs__msg__RobotPlan__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quad_msgs__msg__RobotPlan);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quad_msgs__msg__RobotPlan * data =
      (quad_msgs__msg__RobotPlan *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quad_msgs__msg__RobotPlan__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quad_msgs__msg__RobotPlan__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quad_msgs__msg__RobotPlan__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
