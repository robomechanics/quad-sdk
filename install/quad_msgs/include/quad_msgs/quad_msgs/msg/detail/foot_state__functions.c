// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quad_msgs:msg/FootState.idl
// generated code does not contain a copyright notice
#include "quad_msgs/msg/detail/foot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
// Member `velocity`
// Member `acceleration`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
quad_msgs__msg__FootState__init(quad_msgs__msg__FootState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    quad_msgs__msg__FootState__fini(msg);
    return false;
  }
  // traj_index
  // position
  if (!geometry_msgs__msg__Vector3__init(&msg->position)) {
    quad_msgs__msg__FootState__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    quad_msgs__msg__FootState__fini(msg);
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__init(&msg->acceleration)) {
    quad_msgs__msg__FootState__fini(msg);
    return false;
  }
  // contact
  return true;
}

void
quad_msgs__msg__FootState__fini(quad_msgs__msg__FootState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // traj_index
  // position
  geometry_msgs__msg__Vector3__fini(&msg->position);
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
  // acceleration
  geometry_msgs__msg__Vector3__fini(&msg->acceleration);
  // contact
}

bool
quad_msgs__msg__FootState__are_equal(const quad_msgs__msg__FootState * lhs, const quad_msgs__msg__FootState * rhs)
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
  // traj_index
  if (lhs->traj_index != rhs->traj_index) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->acceleration), &(rhs->acceleration)))
  {
    return false;
  }
  // contact
  if (lhs->contact != rhs->contact) {
    return false;
  }
  return true;
}

bool
quad_msgs__msg__FootState__copy(
  const quad_msgs__msg__FootState * input,
  quad_msgs__msg__FootState * output)
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
  // traj_index
  output->traj_index = input->traj_index;
  // position
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->acceleration), &(output->acceleration)))
  {
    return false;
  }
  // contact
  output->contact = input->contact;
  return true;
}

quad_msgs__msg__FootState *
quad_msgs__msg__FootState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__FootState * msg = (quad_msgs__msg__FootState *)allocator.allocate(sizeof(quad_msgs__msg__FootState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quad_msgs__msg__FootState));
  bool success = quad_msgs__msg__FootState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quad_msgs__msg__FootState__destroy(quad_msgs__msg__FootState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quad_msgs__msg__FootState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quad_msgs__msg__FootState__Sequence__init(quad_msgs__msg__FootState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__FootState * data = NULL;

  if (size) {
    data = (quad_msgs__msg__FootState *)allocator.zero_allocate(size, sizeof(quad_msgs__msg__FootState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quad_msgs__msg__FootState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quad_msgs__msg__FootState__fini(&data[i - 1]);
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
quad_msgs__msg__FootState__Sequence__fini(quad_msgs__msg__FootState__Sequence * array)
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
      quad_msgs__msg__FootState__fini(&array->data[i]);
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

quad_msgs__msg__FootState__Sequence *
quad_msgs__msg__FootState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__FootState__Sequence * array = (quad_msgs__msg__FootState__Sequence *)allocator.allocate(sizeof(quad_msgs__msg__FootState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quad_msgs__msg__FootState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quad_msgs__msg__FootState__Sequence__destroy(quad_msgs__msg__FootState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quad_msgs__msg__FootState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quad_msgs__msg__FootState__Sequence__are_equal(const quad_msgs__msg__FootState__Sequence * lhs, const quad_msgs__msg__FootState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quad_msgs__msg__FootState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quad_msgs__msg__FootState__Sequence__copy(
  const quad_msgs__msg__FootState__Sequence * input,
  quad_msgs__msg__FootState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quad_msgs__msg__FootState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quad_msgs__msg__FootState * data =
      (quad_msgs__msg__FootState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quad_msgs__msg__FootState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quad_msgs__msg__FootState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quad_msgs__msg__FootState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
