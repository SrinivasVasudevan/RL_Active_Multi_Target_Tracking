// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbam_interfaces:msg/TargetBeliefs.idl
// generated code does not contain a copyright notice

#ifndef MBAM_INTERFACES__MSG__DETAIL__TARGET_BELIEFS__STRUCT_H_
#define MBAM_INTERFACES__MSG__DETAIL__TARGET_BELIEFS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'mean_x'
// Member 'mean_y'
// Member 'active_mask'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/TargetBeliefs in the package mbam_interfaces.
typedef struct mbam_interfaces__msg__TargetBeliefs
{
  std_msgs__msg__Header header;
  int32_t max_num_targets;
  rosidl_runtime_c__double__Sequence mean_x;
  rosidl_runtime_c__double__Sequence mean_y;
  rosidl_runtime_c__boolean__Sequence active_mask;
} mbam_interfaces__msg__TargetBeliefs;

// Struct for a sequence of mbam_interfaces__msg__TargetBeliefs.
typedef struct mbam_interfaces__msg__TargetBeliefs__Sequence
{
  mbam_interfaces__msg__TargetBeliefs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbam_interfaces__msg__TargetBeliefs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBAM_INTERFACES__MSG__DETAIL__TARGET_BELIEFS__STRUCT_H_
