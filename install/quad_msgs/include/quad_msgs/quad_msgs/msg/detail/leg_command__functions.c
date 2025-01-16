// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quad_msgs:msg/LegCommand.idl
// generated code does not contain a copyright notice
#include "quad_msgs/msg/detail/leg_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `motor_commands`
#include "quad_msgs/msg/detail/motor_command__functions.h"

bool
quad_msgs__msg__LegCommand__init(quad_msgs__msg__LegCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    quad_msgs__msg__LegCommand__fini(msg);
    return false;
  }
  // motor_commands
  if (!quad_msgs__msg__MotorCommand__Sequence__init(&msg->motor_commands, 0)) {
    quad_msgs__msg__LegCommand__fini(msg);
    return false;
  }
  return true;
}

void
quad_msgs__msg__LegCommand__fini(quad_msgs__msg__LegCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // motor_commands
  quad_msgs__msg__MotorCommand__Sequence__fini(&msg->motor_commands);
}

bool
quad_msgs__msg__LegCommand__are_equal(const quad_msgs__msg__LegCommand * lhs, const quad_msgs__msg__LegCommand * rhs)
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
  // motor_commands
  if (!quad_msgs__msg__MotorCommand__Sequence__are_equal(
      &(lhs->motor_commands), &(rhs->motor_commands)))
  {
    return false;
  }
  return true;
}

bool
quad_msgs__msg__LegCommand__copy(
  const quad_msgs__msg__LegCommand * input,
  quad_msgs__msg__LegCommand * output)
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
  // motor_commands
  if (!quad_msgs__msg__MotorCommand__Sequence__copy(
      &(input->motor_commands), &(output->motor_commands)))
  {
    return false;
  }
  return true;
}

quad_msgs__msg__LegCommand *
quad_msgs__msg__LegCommand__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__LegCommand * msg = (quad_msgs__msg__LegCommand *)allocator.allocate(sizeof(quad_msgs__msg__LegCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quad_msgs__msg__LegCommand));
  bool success = quad_msgs__msg__LegCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quad_msgs__msg__LegCommand__destroy(quad_msgs__msg__LegCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quad_msgs__msg__LegCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quad_msgs__msg__LegCommand__Sequence__init(quad_msgs__msg__LegCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__LegCommand * data = NULL;

  if (size) {
    data = (quad_msgs__msg__LegCommand *)allocator.zero_allocate(size, sizeof(quad_msgs__msg__LegCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quad_msgs__msg__LegCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quad_msgs__msg__LegCommand__fini(&data[i - 1]);
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
quad_msgs__msg__LegCommand__Sequence__fini(quad_msgs__msg__LegCommand__Sequence * array)
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
      quad_msgs__msg__LegCommand__fini(&array->data[i]);
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

quad_msgs__msg__LegCommand__Sequence *
quad_msgs__msg__LegCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__LegCommand__Sequence * array = (quad_msgs__msg__LegCommand__Sequence *)allocator.allocate(sizeof(quad_msgs__msg__LegCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quad_msgs__msg__LegCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quad_msgs__msg__LegCommand__Sequence__destroy(quad_msgs__msg__LegCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quad_msgs__msg__LegCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quad_msgs__msg__LegCommand__Sequence__are_equal(const quad_msgs__msg__LegCommand__Sequence * lhs, const quad_msgs__msg__LegCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quad_msgs__msg__LegCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quad_msgs__msg__LegCommand__Sequence__copy(
  const quad_msgs__msg__LegCommand__Sequence * input,
  quad_msgs__msg__LegCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quad_msgs__msg__LegCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quad_msgs__msg__LegCommand * data =
      (quad_msgs__msg__LegCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quad_msgs__msg__LegCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quad_msgs__msg__LegCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quad_msgs__msg__LegCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
