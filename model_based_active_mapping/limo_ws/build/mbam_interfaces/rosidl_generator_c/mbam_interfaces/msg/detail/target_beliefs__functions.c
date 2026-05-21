// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mbam_interfaces:msg/TargetBeliefs.idl
// generated code does not contain a copyright notice
#include "mbam_interfaces/msg/detail/target_beliefs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `mean_x`
// Member `mean_y`
// Member `active_mask`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
mbam_interfaces__msg__TargetBeliefs__init(mbam_interfaces__msg__TargetBeliefs * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mbam_interfaces__msg__TargetBeliefs__fini(msg);
    return false;
  }
  // max_num_targets
  // mean_x
  if (!rosidl_runtime_c__double__Sequence__init(&msg->mean_x, 0)) {
    mbam_interfaces__msg__TargetBeliefs__fini(msg);
    return false;
  }
  // mean_y
  if (!rosidl_runtime_c__double__Sequence__init(&msg->mean_y, 0)) {
    mbam_interfaces__msg__TargetBeliefs__fini(msg);
    return false;
  }
  // active_mask
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->active_mask, 0)) {
    mbam_interfaces__msg__TargetBeliefs__fini(msg);
    return false;
  }
  return true;
}

void
mbam_interfaces__msg__TargetBeliefs__fini(mbam_interfaces__msg__TargetBeliefs * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // max_num_targets
  // mean_x
  rosidl_runtime_c__double__Sequence__fini(&msg->mean_x);
  // mean_y
  rosidl_runtime_c__double__Sequence__fini(&msg->mean_y);
  // active_mask
  rosidl_runtime_c__boolean__Sequence__fini(&msg->active_mask);
}

bool
mbam_interfaces__msg__TargetBeliefs__are_equal(const mbam_interfaces__msg__TargetBeliefs * lhs, const mbam_interfaces__msg__TargetBeliefs * rhs)
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
  // max_num_targets
  if (lhs->max_num_targets != rhs->max_num_targets) {
    return false;
  }
  // mean_x
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->mean_x), &(rhs->mean_x)))
  {
    return false;
  }
  // mean_y
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->mean_y), &(rhs->mean_y)))
  {
    return false;
  }
  // active_mask
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->active_mask), &(rhs->active_mask)))
  {
    return false;
  }
  return true;
}

bool
mbam_interfaces__msg__TargetBeliefs__copy(
  const mbam_interfaces__msg__TargetBeliefs * input,
  mbam_interfaces__msg__TargetBeliefs * output)
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
  // max_num_targets
  output->max_num_targets = input->max_num_targets;
  // mean_x
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->mean_x), &(output->mean_x)))
  {
    return false;
  }
  // mean_y
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->mean_y), &(output->mean_y)))
  {
    return false;
  }
  // active_mask
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->active_mask), &(output->active_mask)))
  {
    return false;
  }
  return true;
}

mbam_interfaces__msg__TargetBeliefs *
mbam_interfaces__msg__TargetBeliefs__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbam_interfaces__msg__TargetBeliefs * msg = (mbam_interfaces__msg__TargetBeliefs *)allocator.allocate(sizeof(mbam_interfaces__msg__TargetBeliefs), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbam_interfaces__msg__TargetBeliefs));
  bool success = mbam_interfaces__msg__TargetBeliefs__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbam_interfaces__msg__TargetBeliefs__destroy(mbam_interfaces__msg__TargetBeliefs * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbam_interfaces__msg__TargetBeliefs__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbam_interfaces__msg__TargetBeliefs__Sequence__init(mbam_interfaces__msg__TargetBeliefs__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbam_interfaces__msg__TargetBeliefs * data = NULL;

  if (size) {
    data = (mbam_interfaces__msg__TargetBeliefs *)allocator.zero_allocate(size, sizeof(mbam_interfaces__msg__TargetBeliefs), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbam_interfaces__msg__TargetBeliefs__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbam_interfaces__msg__TargetBeliefs__fini(&data[i - 1]);
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
mbam_interfaces__msg__TargetBeliefs__Sequence__fini(mbam_interfaces__msg__TargetBeliefs__Sequence * array)
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
      mbam_interfaces__msg__TargetBeliefs__fini(&array->data[i]);
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

mbam_interfaces__msg__TargetBeliefs__Sequence *
mbam_interfaces__msg__TargetBeliefs__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbam_interfaces__msg__TargetBeliefs__Sequence * array = (mbam_interfaces__msg__TargetBeliefs__Sequence *)allocator.allocate(sizeof(mbam_interfaces__msg__TargetBeliefs__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbam_interfaces__msg__TargetBeliefs__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbam_interfaces__msg__TargetBeliefs__Sequence__destroy(mbam_interfaces__msg__TargetBeliefs__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbam_interfaces__msg__TargetBeliefs__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbam_interfaces__msg__TargetBeliefs__Sequence__are_equal(const mbam_interfaces__msg__TargetBeliefs__Sequence * lhs, const mbam_interfaces__msg__TargetBeliefs__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbam_interfaces__msg__TargetBeliefs__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbam_interfaces__msg__TargetBeliefs__Sequence__copy(
  const mbam_interfaces__msg__TargetBeliefs__Sequence * input,
  mbam_interfaces__msg__TargetBeliefs__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbam_interfaces__msg__TargetBeliefs);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbam_interfaces__msg__TargetBeliefs * data =
      (mbam_interfaces__msg__TargetBeliefs *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbam_interfaces__msg__TargetBeliefs__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbam_interfaces__msg__TargetBeliefs__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbam_interfaces__msg__TargetBeliefs__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
