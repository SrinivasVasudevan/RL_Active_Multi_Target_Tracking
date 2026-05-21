// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbam_interfaces:msg/AgentObservation.idl
// generated code does not contain a copyright notice

#ifndef MBAM_INTERFACES__MSG__DETAIL__AGENT_OBSERVATION__STRUCT_H_
#define MBAM_INTERFACES__MSG__DETAIL__AGENT_OBSERVATION__STRUCT_H_

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
// Member 'z_world_x'
// Member 'z_world_y'
// Member 'visible'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/AgentObservation in the package mbam_interfaces.
typedef struct mbam_interfaces__msg__AgentObservation
{
  std_msgs__msg__Header header;
  int32_t robot_id;
  double pose_x;
  double pose_y;
  double pose_yaw;
  int32_t max_num_targets;
  rosidl_runtime_c__double__Sequence z_world_x;
  rosidl_runtime_c__double__Sequence z_world_y;
  rosidl_runtime_c__boolean__Sequence visible;
} mbam_interfaces__msg__AgentObservation;

// Struct for a sequence of mbam_interfaces__msg__AgentObservation.
typedef struct mbam_interfaces__msg__AgentObservation__Sequence
{
  mbam_interfaces__msg__AgentObservation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbam_interfaces__msg__AgentObservation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBAM_INTERFACES__MSG__DETAIL__AGENT_OBSERVATION__STRUCT_H_
