// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mbam_interfaces:msg/AgentObservation.idl
// generated code does not contain a copyright notice
#include "mbam_interfaces/msg/detail/agent_observation__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `z_world_x`
// Member `z_world_y`
// Member `visible`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
mbam_interfaces__msg__AgentObservation__init(mbam_interfaces__msg__AgentObservation * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mbam_interfaces__msg__AgentObservation__fini(msg);
    return false;
  }
  // robot_id
  // pose_x
  // pose_y
  // pose_yaw
  // max_num_targets
  // z_world_x
  if (!rosidl_runtime_c__double__Sequence__init(&msg->z_world_x, 0)) {
    mbam_interfaces__msg__AgentObservation__fini(msg);
    return false;
  }
  // z_world_y
  if (!rosidl_runtime_c__double__Sequence__init(&msg->z_world_y, 0)) {
    mbam_interfaces__msg__AgentObservation__fini(msg);
    return false;
  }
  // visible
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->visible, 0)) {
    mbam_interfaces__msg__AgentObservation__fini(msg);
    return false;
  }
  return true;
}

void
mbam_interfaces__msg__AgentObservation__fini(mbam_interfaces__msg__AgentObservation * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // robot_id
  // pose_x
  // pose_y
  // pose_yaw
  // max_num_targets
  // z_world_x
  rosidl_runtime_c__double__Sequence__fini(&msg->z_world_x);
  // z_world_y
  rosidl_runtime_c__double__Sequence__fini(&msg->z_world_y);
  // visible
  rosidl_runtime_c__boolean__Sequence__fini(&msg->visible);
}

bool
mbam_interfaces__msg__AgentObservation__are_equal(const mbam_interfaces__msg__AgentObservation * lhs, const mbam_interfaces__msg__AgentObservation * rhs)
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
  // robot_id
  if (lhs->robot_id != rhs->robot_id) {
    return false;
  }
  // pose_x
  if (lhs->pose_x != rhs->pose_x) {
    return false;
  }
  // pose_y
  if (lhs->pose_y != rhs->pose_y) {
    return false;
  }
  // pose_yaw
  if (lhs->pose_yaw != rhs->pose_yaw) {
    return false;
  }
  // max_num_targets
  if (lhs->max_num_targets != rhs->max_num_targets) {
    return false;
  }
  // z_world_x
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->z_world_x), &(rhs->z_world_x)))
  {
    return false;
  }
  // z_world_y
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->z_world_y), &(rhs->z_world_y)))
  {
    return false;
  }
  // visible
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->visible), &(rhs->visible)))
  {
    return false;
  }
  return true;
}

bool
mbam_interfaces__msg__AgentObservation__copy(
  const mbam_interfaces__msg__AgentObservation * input,
  mbam_interfaces__msg__AgentObservation * output)
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
  // robot_id
  output->robot_id = input->robot_id;
  // pose_x
  output->pose_x = input->pose_x;
  // pose_y
  output->pose_y = input->pose_y;
  // pose_yaw
  output->pose_yaw = input->pose_yaw;
  // max_num_targets
  output->max_num_targets = input->max_num_targets;
  // z_world_x
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->z_world_x), &(output->z_world_x)))
  {
    return false;
  }
  // z_world_y
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->z_world_y), &(output->z_world_y)))
  {
    return false;
  }
  // visible
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->visible), &(output->visible)))
  {
    return false;
  }
  return true;
}

mbam_interfaces__msg__AgentObservation *
mbam_interfaces__msg__AgentObservation__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbam_interfaces__msg__AgentObservation * msg = (mbam_interfaces__msg__AgentObservation *)allocator.allocate(sizeof(mbam_interfaces__msg__AgentObservation), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbam_interfaces__msg__AgentObservation));
  bool success = mbam_interfaces__msg__AgentObservation__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbam_interfaces__msg__AgentObservation__destroy(mbam_interfaces__msg__AgentObservation * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbam_interfaces__msg__AgentObservation__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbam_interfaces__msg__AgentObservation__Sequence__init(mbam_interfaces__msg__AgentObservation__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbam_interfaces__msg__AgentObservation * data = NULL;

  if (size) {
    data = (mbam_interfaces__msg__AgentObservation *)allocator.zero_allocate(size, sizeof(mbam_interfaces__msg__AgentObservation), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbam_interfaces__msg__AgentObservation__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbam_interfaces__msg__AgentObservation__fini(&data[i - 1]);
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
mbam_interfaces__msg__AgentObservation__Sequence__fini(mbam_interfaces__msg__AgentObservation__Sequence * array)
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
      mbam_interfaces__msg__AgentObservation__fini(&array->data[i]);
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

mbam_interfaces__msg__AgentObservation__Sequence *
mbam_interfaces__msg__AgentObservation__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbam_interfaces__msg__AgentObservation__Sequence * array = (mbam_interfaces__msg__AgentObservation__Sequence *)allocator.allocate(sizeof(mbam_interfaces__msg__AgentObservation__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbam_interfaces__msg__AgentObservation__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbam_interfaces__msg__AgentObservation__Sequence__destroy(mbam_interfaces__msg__AgentObservation__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbam_interfaces__msg__AgentObservation__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbam_interfaces__msg__AgentObservation__Sequence__are_equal(const mbam_interfaces__msg__AgentObservation__Sequence * lhs, const mbam_interfaces__msg__AgentObservation__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbam_interfaces__msg__AgentObservation__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbam_interfaces__msg__AgentObservation__Sequence__copy(
  const mbam_interfaces__msg__AgentObservation__Sequence * input,
  mbam_interfaces__msg__AgentObservation__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbam_interfaces__msg__AgentObservation);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbam_interfaces__msg__AgentObservation * data =
      (mbam_interfaces__msg__AgentObservation *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbam_interfaces__msg__AgentObservation__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbam_interfaces__msg__AgentObservation__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbam_interfaces__msg__AgentObservation__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
