// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quad_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice
#include "quad_msgs/msg/detail/motor_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
quad_msgs__msg__MotorCommand__init(quad_msgs__msg__MotorCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    quad_msgs__msg__MotorCommand__fini(msg);
    return false;
  }
  // pos_setpoint
  // vel_setpoint
  // kp
  // kd
  // torque_ff
  // pos_component
  // vel_component
  // fb_component
  // effort
  // fb_ratio
  return true;
}

void
quad_msgs__msg__MotorCommand__fini(quad_msgs__msg__MotorCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // pos_setpoint
  // vel_setpoint
  // kp
  // kd
  // torque_ff
  // pos_component
  // vel_component
  // fb_component
  // effort
  // fb_ratio
}

bool
quad_msgs__msg__MotorCommand__are_equal(const quad_msgs__msg__MotorCommand * lhs, const quad_msgs__msg__MotorCommand * rhs)
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
  // pos_setpoint
  if (lhs->pos_setpoint != rhs->pos_setpoint) {
    return false;
  }
  // vel_setpoint
  if (lhs->vel_setpoint != rhs->vel_setpoint) {
    return false;
  }
  // kp
  if (lhs->kp != rhs->kp) {
    return false;
  }
  // kd
  if (lhs->kd != rhs->kd) {
    return false;
  }
  // torque_ff
  if (lhs->torque_ff != rhs->torque_ff) {
    return false;
  }
  // pos_component
  if (lhs->pos_component != rhs->pos_component) {
    return false;
  }
  // vel_component
  if (lhs->vel_component != rhs->vel_component) {
    return false;
  }
  // fb_component
  if (lhs->fb_component != rhs->fb_component) {
    return false;
  }
  // effort
  if (lhs->effort != rhs->effort) {
    return false;
  }
  // fb_ratio
  if (lhs->fb_ratio != rhs->fb_ratio) {
    return false;
  }
  return true;
}

bool
quad_msgs__msg__MotorCommand__copy(
  const quad_msgs__msg__MotorCommand * input,
  quad_msgs__msg__MotorCommand * output)
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
  // pos_setpoint
  output->pos_setpoint = input->pos_setpoint;
  // vel_setpoint
  output->vel_setpoint = input->vel_setpoint;
  // kp
  output->kp = input->kp;
  // kd
  output->kd = input->kd;
  // torque_ff
  output->torque_ff = input->torque_ff;
  // pos_component
  output->pos_component = input->pos_component;
  // vel_component
  output->vel_component = input->vel_component;
  // fb_component
  output->fb_component = input->fb_component;
  // effort
  output->effort = input->effort;
  // fb_ratio
  output->fb_ratio = input->fb_ratio;
  return true;
}

quad_msgs__msg__MotorCommand *
quad_msgs__msg__MotorCommand__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__MotorCommand * msg = (quad_msgs__msg__MotorCommand *)allocator.allocate(sizeof(quad_msgs__msg__MotorCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quad_msgs__msg__MotorCommand));
  bool success = quad_msgs__msg__MotorCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quad_msgs__msg__MotorCommand__destroy(quad_msgs__msg__MotorCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quad_msgs__msg__MotorCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quad_msgs__msg__MotorCommand__Sequence__init(quad_msgs__msg__MotorCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__MotorCommand * data = NULL;

  if (size) {
    data = (quad_msgs__msg__MotorCommand *)allocator.zero_allocate(size, sizeof(quad_msgs__msg__MotorCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quad_msgs__msg__MotorCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quad_msgs__msg__MotorCommand__fini(&data[i - 1]);
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
quad_msgs__msg__MotorCommand__Sequence__fini(quad_msgs__msg__MotorCommand__Sequence * array)
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
      quad_msgs__msg__MotorCommand__fini(&array->data[i]);
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

quad_msgs__msg__MotorCommand__Sequence *
quad_msgs__msg__MotorCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__MotorCommand__Sequence * array = (quad_msgs__msg__MotorCommand__Sequence *)allocator.allocate(sizeof(quad_msgs__msg__MotorCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quad_msgs__msg__MotorCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quad_msgs__msg__MotorCommand__Sequence__destroy(quad_msgs__msg__MotorCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quad_msgs__msg__MotorCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quad_msgs__msg__MotorCommand__Sequence__are_equal(const quad_msgs__msg__MotorCommand__Sequence * lhs, const quad_msgs__msg__MotorCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quad_msgs__msg__MotorCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quad_msgs__msg__MotorCommand__Sequence__copy(
  const quad_msgs__msg__MotorCommand__Sequence * input,
  quad_msgs__msg__MotorCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quad_msgs__msg__MotorCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quad_msgs__msg__MotorCommand * data =
      (quad_msgs__msg__MotorCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quad_msgs__msg__MotorCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quad_msgs__msg__MotorCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quad_msgs__msg__MotorCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
