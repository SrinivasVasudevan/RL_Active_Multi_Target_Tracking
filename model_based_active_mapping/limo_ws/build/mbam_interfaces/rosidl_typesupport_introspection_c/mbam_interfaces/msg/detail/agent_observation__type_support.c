// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mbam_interfaces:msg/AgentObservation.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mbam_interfaces/msg/detail/agent_observation__rosidl_typesupport_introspection_c.h"
#include "mbam_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mbam_interfaces/msg/detail/agent_observation__functions.h"
#include "mbam_interfaces/msg/detail/agent_observation__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `z_world_x`
// Member `z_world_y`
// Member `visible`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__AgentObservation_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbam_interfaces__msg__AgentObservation__init(message_memory);
}

void mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__AgentObservation_fini_function(void * message_memory)
{
  mbam_interfaces__msg__AgentObservation__fini(message_memory);
}

size_t mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__size_function__AgentObservation__z_world_x(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_const_function__AgentObservation__z_world_x(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_function__AgentObservation__z_world_x(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__fetch_function__AgentObservation__z_world_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_const_function__AgentObservation__z_world_x(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__assign_function__AgentObservation__z_world_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_function__AgentObservation__z_world_x(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__resize_function__AgentObservation__z_world_x(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__size_function__AgentObservation__z_world_y(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_const_function__AgentObservation__z_world_y(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_function__AgentObservation__z_world_y(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__fetch_function__AgentObservation__z_world_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_const_function__AgentObservation__z_world_y(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__assign_function__AgentObservation__z_world_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_function__AgentObservation__z_world_y(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__resize_function__AgentObservation__z_world_y(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__size_function__AgentObservation__visible(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_const_function__AgentObservation__visible(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_function__AgentObservation__visible(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__fetch_function__AgentObservation__visible(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_const_function__AgentObservation__visible(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__assign_function__AgentObservation__visible(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_function__AgentObservation__visible(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__resize_function__AgentObservation__visible(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__AgentObservation_message_member_array[9] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces__msg__AgentObservation, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces__msg__AgentObservation, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces__msg__AgentObservation, pose_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces__msg__AgentObservation, pose_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose_yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces__msg__AgentObservation, pose_yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_num_targets",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces__msg__AgentObservation, max_num_targets),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z_world_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces__msg__AgentObservation, z_world_x),  // bytes offset in struct
    NULL,  // default value
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__size_function__AgentObservation__z_world_x,  // size() function pointer
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_const_function__AgentObservation__z_world_x,  // get_const(index) function pointer
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_function__AgentObservation__z_world_x,  // get(index) function pointer
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__fetch_function__AgentObservation__z_world_x,  // fetch(index, &value) function pointer
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__assign_function__AgentObservation__z_world_x,  // assign(index, value) function pointer
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__resize_function__AgentObservation__z_world_x  // resize(index) function pointer
  },
  {
    "z_world_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces__msg__AgentObservation, z_world_y),  // bytes offset in struct
    NULL,  // default value
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__size_function__AgentObservation__z_world_y,  // size() function pointer
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_const_function__AgentObservation__z_world_y,  // get_const(index) function pointer
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_function__AgentObservation__z_world_y,  // get(index) function pointer
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__fetch_function__AgentObservation__z_world_y,  // fetch(index, &value) function pointer
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__assign_function__AgentObservation__z_world_y,  // assign(index, value) function pointer
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__resize_function__AgentObservation__z_world_y  // resize(index) function pointer
  },
  {
    "visible",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces__msg__AgentObservation, visible),  // bytes offset in struct
    NULL,  // default value
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__size_function__AgentObservation__visible,  // size() function pointer
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_const_function__AgentObservation__visible,  // get_const(index) function pointer
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__get_function__AgentObservation__visible,  // get(index) function pointer
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__fetch_function__AgentObservation__visible,  // fetch(index, &value) function pointer
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__assign_function__AgentObservation__visible,  // assign(index, value) function pointer
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__resize_function__AgentObservation__visible  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__AgentObservation_message_members = {
  "mbam_interfaces__msg",  // message namespace
  "AgentObservation",  // message name
  9,  // number of fields
  sizeof(mbam_interfaces__msg__AgentObservation),
  mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__AgentObservation_message_member_array,  // message members
  mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__AgentObservation_init_function,  // function to initialize message memory (memory has to be allocated)
  mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__AgentObservation_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__AgentObservation_message_type_support_handle = {
  0,
  &mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__AgentObservation_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbam_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbam_interfaces, msg, AgentObservation)() {
  mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__AgentObservation_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__AgentObservation_message_type_support_handle.typesupport_identifier) {
    mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__AgentObservation_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbam_interfaces__msg__AgentObservation__rosidl_typesupport_introspection_c__AgentObservation_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
