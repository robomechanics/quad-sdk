// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quad_msgs:msg/RobotPlanDiagnostics.idl
// generated code does not contain a copyright notice
#include "quad_msgs/msg/detail/robot_plan_diagnostics__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `complexity_schedule`
// Member `element_times`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
quad_msgs__msg__RobotPlanDiagnostics__init(quad_msgs__msg__RobotPlanDiagnostics * msg)
{
  if (!msg) {
    return false;
  }
  // compute_time
  // cost
  // iterations
  // horizon_length
  // complexity_schedule
  if (!rosidl_runtime_c__uint32__Sequence__init(&msg->complexity_schedule, 0)) {
    quad_msgs__msg__RobotPlanDiagnostics__fini(msg);
    return false;
  }
  // element_times
  if (!rosidl_runtime_c__double__Sequence__init(&msg->element_times, 0)) {
    quad_msgs__msg__RobotPlanDiagnostics__fini(msg);
    return false;
  }
  return true;
}

void
quad_msgs__msg__RobotPlanDiagnostics__fini(quad_msgs__msg__RobotPlanDiagnostics * msg)
{
  if (!msg) {
    return;
  }
  // compute_time
  // cost
  // iterations
  // horizon_length
  // complexity_schedule
  rosidl_runtime_c__uint32__Sequence__fini(&msg->complexity_schedule);
  // element_times
  rosidl_runtime_c__double__Sequence__fini(&msg->element_times);
}

bool
quad_msgs__msg__RobotPlanDiagnostics__are_equal(const quad_msgs__msg__RobotPlanDiagnostics * lhs, const quad_msgs__msg__RobotPlanDiagnostics * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // compute_time
  if (lhs->compute_time != rhs->compute_time) {
    return false;
  }
  // cost
  if (lhs->cost != rhs->cost) {
    return false;
  }
  // iterations
  if (lhs->iterations != rhs->iterations) {
    return false;
  }
  // horizon_length
  if (lhs->horizon_length != rhs->horizon_length) {
    return false;
  }
  // complexity_schedule
  if (!rosidl_runtime_c__uint32__Sequence__are_equal(
      &(lhs->complexity_schedule), &(rhs->complexity_schedule)))
  {
    return false;
  }
  // element_times
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->element_times), &(rhs->element_times)))
  {
    return false;
  }
  return true;
}

bool
quad_msgs__msg__RobotPlanDiagnostics__copy(
  const quad_msgs__msg__RobotPlanDiagnostics * input,
  quad_msgs__msg__RobotPlanDiagnostics * output)
{
  if (!input || !output) {
    return false;
  }
  // compute_time
  output->compute_time = input->compute_time;
  // cost
  output->cost = input->cost;
  // iterations
  output->iterations = input->iterations;
  // horizon_length
  output->horizon_length = input->horizon_length;
  // complexity_schedule
  if (!rosidl_runtime_c__uint32__Sequence__copy(
      &(input->complexity_schedule), &(output->complexity_schedule)))
  {
    return false;
  }
  // element_times
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->element_times), &(output->element_times)))
  {
    return false;
  }
  return true;
}

quad_msgs__msg__RobotPlanDiagnostics *
quad_msgs__msg__RobotPlanDiagnostics__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__RobotPlanDiagnostics * msg = (quad_msgs__msg__RobotPlanDiagnostics *)allocator.allocate(sizeof(quad_msgs__msg__RobotPlanDiagnostics), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quad_msgs__msg__RobotPlanDiagnostics));
  bool success = quad_msgs__msg__RobotPlanDiagnostics__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quad_msgs__msg__RobotPlanDiagnostics__destroy(quad_msgs__msg__RobotPlanDiagnostics * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quad_msgs__msg__RobotPlanDiagnostics__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quad_msgs__msg__RobotPlanDiagnostics__Sequence__init(quad_msgs__msg__RobotPlanDiagnostics__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__RobotPlanDiagnostics * data = NULL;

  if (size) {
    data = (quad_msgs__msg__RobotPlanDiagnostics *)allocator.zero_allocate(size, sizeof(quad_msgs__msg__RobotPlanDiagnostics), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quad_msgs__msg__RobotPlanDiagnostics__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quad_msgs__msg__RobotPlanDiagnostics__fini(&data[i - 1]);
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
quad_msgs__msg__RobotPlanDiagnostics__Sequence__fini(quad_msgs__msg__RobotPlanDiagnostics__Sequence * array)
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
      quad_msgs__msg__RobotPlanDiagnostics__fini(&array->data[i]);
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

quad_msgs__msg__RobotPlanDiagnostics__Sequence *
quad_msgs__msg__RobotPlanDiagnostics__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__RobotPlanDiagnostics__Sequence * array = (quad_msgs__msg__RobotPlanDiagnostics__Sequence *)allocator.allocate(sizeof(quad_msgs__msg__RobotPlanDiagnostics__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quad_msgs__msg__RobotPlanDiagnostics__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quad_msgs__msg__RobotPlanDiagnostics__Sequence__destroy(quad_msgs__msg__RobotPlanDiagnostics__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quad_msgs__msg__RobotPlanDiagnostics__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quad_msgs__msg__RobotPlanDiagnostics__Sequence__are_equal(const quad_msgs__msg__RobotPlanDiagnostics__Sequence * lhs, const quad_msgs__msg__RobotPlanDiagnostics__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quad_msgs__msg__RobotPlanDiagnostics__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quad_msgs__msg__RobotPlanDiagnostics__Sequence__copy(
  const quad_msgs__msg__RobotPlanDiagnostics__Sequence * input,
  quad_msgs__msg__RobotPlanDiagnostics__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quad_msgs__msg__RobotPlanDiagnostics);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quad_msgs__msg__RobotPlanDiagnostics * data =
      (quad_msgs__msg__RobotPlanDiagnostics *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quad_msgs__msg__RobotPlanDiagnostics__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quad_msgs__msg__RobotPlanDiagnostics__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quad_msgs__msg__RobotPlanDiagnostics__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
