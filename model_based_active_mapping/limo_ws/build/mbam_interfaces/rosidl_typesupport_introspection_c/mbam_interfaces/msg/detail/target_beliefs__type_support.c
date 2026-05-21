// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mbam_interfaces:msg/TargetBeliefs.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mbam_interfaces/msg/detail/target_beliefs__rosidl_typesupport_introspection_c.h"
#include "mbam_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mbam_interfaces/msg/detail/target_beliefs__functions.h"
#include "mbam_interfaces/msg/detail/target_beliefs__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `mean_x`
// Member `mean_y`
// Member `active_mask`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__TargetBeliefs_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbam_interfaces__msg__TargetBeliefs__init(message_memory);
}

void mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__TargetBeliefs_fini_function(void * message_memory)
{
  mbam_interfaces__msg__TargetBeliefs__fini(message_memory);
}

size_t mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__size_function__TargetBeliefs__mean_x(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_const_function__TargetBeliefs__mean_x(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_function__TargetBeliefs__mean_x(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__fetch_function__TargetBeliefs__mean_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_const_function__TargetBeliefs__mean_x(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__assign_function__TargetBeliefs__mean_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_function__TargetBeliefs__mean_x(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__resize_function__TargetBeliefs__mean_x(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__size_function__TargetBeliefs__mean_y(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_const_function__TargetBeliefs__mean_y(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_function__TargetBeliefs__mean_y(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__fetch_function__TargetBeliefs__mean_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_const_function__TargetBeliefs__mean_y(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__assign_function__TargetBeliefs__mean_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_function__TargetBeliefs__mean_y(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__resize_function__TargetBeliefs__mean_y(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__size_function__TargetBeliefs__active_mask(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_const_function__TargetBeliefs__active_mask(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_function__TargetBeliefs__active_mask(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__fetch_function__TargetBeliefs__active_mask(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_const_function__TargetBeliefs__active_mask(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__assign_function__TargetBeliefs__active_mask(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_function__TargetBeliefs__active_mask(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__resize_function__TargetBeliefs__active_mask(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__TargetBeliefs_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces__msg__TargetBeliefs, header),  // bytes offset in struct
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
    offsetof(mbam_interfaces__msg__TargetBeliefs, max_num_targets),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mean_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces__msg__TargetBeliefs, mean_x),  // bytes offset in struct
    NULL,  // default value
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__size_function__TargetBeliefs__mean_x,  // size() function pointer
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_const_function__TargetBeliefs__mean_x,  // get_const(index) function pointer
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_function__TargetBeliefs__mean_x,  // get(index) function pointer
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__fetch_function__TargetBeliefs__mean_x,  // fetch(index, &value) function pointer
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__assign_function__TargetBeliefs__mean_x,  // assign(index, value) function pointer
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__resize_function__TargetBeliefs__mean_x  // resize(index) function pointer
  },
  {
    "mean_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces__msg__TargetBeliefs, mean_y),  // bytes offset in struct
    NULL,  // default value
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__size_function__TargetBeliefs__mean_y,  // size() function pointer
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_const_function__TargetBeliefs__mean_y,  // get_const(index) function pointer
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_function__TargetBeliefs__mean_y,  // get(index) function pointer
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__fetch_function__TargetBeliefs__mean_y,  // fetch(index, &value) function pointer
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__assign_function__TargetBeliefs__mean_y,  // assign(index, value) function pointer
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__resize_function__TargetBeliefs__mean_y  // resize(index) function pointer
  },
  {
    "active_mask",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces__msg__TargetBeliefs, active_mask),  // bytes offset in struct
    NULL,  // default value
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__size_function__TargetBeliefs__active_mask,  // size() function pointer
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_const_function__TargetBeliefs__active_mask,  // get_const(index) function pointer
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__get_function__TargetBeliefs__active_mask,  // get(index) function pointer
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__fetch_function__TargetBeliefs__active_mask,  // fetch(index, &value) function pointer
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__assign_function__TargetBeliefs__active_mask,  // assign(index, value) function pointer
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__resize_function__TargetBeliefs__active_mask  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__TargetBeliefs_message_members = {
  "mbam_interfaces__msg",  // message namespace
  "TargetBeliefs",  // message name
  5,  // number of fields
  sizeof(mbam_interfaces__msg__TargetBeliefs),
  mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__TargetBeliefs_message_member_array,  // message members
  mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__TargetBeliefs_init_function,  // function to initialize message memory (memory has to be allocated)
  mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__TargetBeliefs_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__TargetBeliefs_message_type_support_handle = {
  0,
  &mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__TargetBeliefs_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbam_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbam_interfaces, msg, TargetBeliefs)() {
  mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__TargetBeliefs_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__TargetBeliefs_message_type_support_handle.typesupport_identifier) {
    mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__TargetBeliefs_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbam_interfaces__msg__TargetBeliefs__rosidl_typesupport_introspection_c__TargetBeliefs_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
