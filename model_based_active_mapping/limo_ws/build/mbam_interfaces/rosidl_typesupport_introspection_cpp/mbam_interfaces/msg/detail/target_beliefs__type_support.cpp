// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mbam_interfaces:msg/TargetBeliefs.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mbam_interfaces/msg/detail/target_beliefs__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mbam_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TargetBeliefs_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mbam_interfaces::msg::TargetBeliefs(_init);
}

void TargetBeliefs_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mbam_interfaces::msg::TargetBeliefs *>(message_memory);
  typed_message->~TargetBeliefs();
}

size_t size_function__TargetBeliefs__mean_x(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TargetBeliefs__mean_x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__TargetBeliefs__mean_x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__TargetBeliefs__mean_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TargetBeliefs__mean_x(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TargetBeliefs__mean_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TargetBeliefs__mean_x(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__TargetBeliefs__mean_x(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__TargetBeliefs__mean_y(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TargetBeliefs__mean_y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__TargetBeliefs__mean_y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__TargetBeliefs__mean_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TargetBeliefs__mean_y(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TargetBeliefs__mean_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TargetBeliefs__mean_y(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__TargetBeliefs__mean_y(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__TargetBeliefs__active_mask(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__TargetBeliefs__active_mask(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__TargetBeliefs__active_mask(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__TargetBeliefs__active_mask(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TargetBeliefs_message_member_array[5] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces::msg::TargetBeliefs, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "max_num_targets",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces::msg::TargetBeliefs, max_num_targets),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "mean_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces::msg::TargetBeliefs, mean_x),  // bytes offset in struct
    nullptr,  // default value
    size_function__TargetBeliefs__mean_x,  // size() function pointer
    get_const_function__TargetBeliefs__mean_x,  // get_const(index) function pointer
    get_function__TargetBeliefs__mean_x,  // get(index) function pointer
    fetch_function__TargetBeliefs__mean_x,  // fetch(index, &value) function pointer
    assign_function__TargetBeliefs__mean_x,  // assign(index, value) function pointer
    resize_function__TargetBeliefs__mean_x  // resize(index) function pointer
  },
  {
    "mean_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces::msg::TargetBeliefs, mean_y),  // bytes offset in struct
    nullptr,  // default value
    size_function__TargetBeliefs__mean_y,  // size() function pointer
    get_const_function__TargetBeliefs__mean_y,  // get_const(index) function pointer
    get_function__TargetBeliefs__mean_y,  // get(index) function pointer
    fetch_function__TargetBeliefs__mean_y,  // fetch(index, &value) function pointer
    assign_function__TargetBeliefs__mean_y,  // assign(index, value) function pointer
    resize_function__TargetBeliefs__mean_y  // resize(index) function pointer
  },
  {
    "active_mask",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbam_interfaces::msg::TargetBeliefs, active_mask),  // bytes offset in struct
    nullptr,  // default value
    size_function__TargetBeliefs__active_mask,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__TargetBeliefs__active_mask,  // fetch(index, &value) function pointer
    assign_function__TargetBeliefs__active_mask,  // assign(index, value) function pointer
    resize_function__TargetBeliefs__active_mask  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TargetBeliefs_message_members = {
  "mbam_interfaces::msg",  // message namespace
  "TargetBeliefs",  // message name
  5,  // number of fields
  sizeof(mbam_interfaces::msg::TargetBeliefs),
  TargetBeliefs_message_member_array,  // message members
  TargetBeliefs_init_function,  // function to initialize message memory (memory has to be allocated)
  TargetBeliefs_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TargetBeliefs_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TargetBeliefs_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mbam_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbam_interfaces::msg::TargetBeliefs>()
{
  return &::mbam_interfaces::msg::rosidl_typesupport_introspection_cpp::TargetBeliefs_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbam_interfaces, msg, TargetBeliefs)() {
  return &::mbam_interfaces::msg::rosidl_typesupport_introspection_cpp::TargetBeliefs_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
