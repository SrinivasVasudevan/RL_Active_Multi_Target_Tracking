// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mbam_interfaces:msg/AgentObservation.idl
// generated code does not contain a copyright notice

#ifndef MBAM_INTERFACES__MSG__DETAIL__AGENT_OBSERVATION__STRUCT_HPP_
#define MBAM_INTERFACES__MSG__DETAIL__AGENT_OBSERVATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbam_interfaces__msg__AgentObservation __attribute__((deprecated))
#else
# define DEPRECATED__mbam_interfaces__msg__AgentObservation __declspec(deprecated)
#endif

namespace mbam_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AgentObservation_
{
  using Type = AgentObservation_<ContainerAllocator>;

  explicit AgentObservation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->pose_x = 0.0;
      this->pose_y = 0.0;
      this->pose_yaw = 0.0;
      this->max_num_targets = 0l;
    }
  }

  explicit AgentObservation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->pose_x = 0.0;
      this->pose_y = 0.0;
      this->pose_yaw = 0.0;
      this->max_num_targets = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _pose_x_type =
    double;
  _pose_x_type pose_x;
  using _pose_y_type =
    double;
  _pose_y_type pose_y;
  using _pose_yaw_type =
    double;
  _pose_yaw_type pose_yaw;
  using _max_num_targets_type =
    int32_t;
  _max_num_targets_type max_num_targets;
  using _z_world_x_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _z_world_x_type z_world_x;
  using _z_world_y_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _z_world_y_type z_world_y;
  using _visible_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _visible_type visible;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__pose_x(
    const double & _arg)
  {
    this->pose_x = _arg;
    return *this;
  }
  Type & set__pose_y(
    const double & _arg)
  {
    this->pose_y = _arg;
    return *this;
  }
  Type & set__pose_yaw(
    const double & _arg)
  {
    this->pose_yaw = _arg;
    return *this;
  }
  Type & set__max_num_targets(
    const int32_t & _arg)
  {
    this->max_num_targets = _arg;
    return *this;
  }
  Type & set__z_world_x(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->z_world_x = _arg;
    return *this;
  }
  Type & set__z_world_y(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->z_world_y = _arg;
    return *this;
  }
  Type & set__visible(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->visible = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbam_interfaces::msg::AgentObservation_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbam_interfaces::msg::AgentObservation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbam_interfaces::msg::AgentObservation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbam_interfaces::msg::AgentObservation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbam_interfaces::msg::AgentObservation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbam_interfaces::msg::AgentObservation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbam_interfaces::msg::AgentObservation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbam_interfaces::msg::AgentObservation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbam_interfaces::msg::AgentObservation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbam_interfaces::msg::AgentObservation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbam_interfaces__msg__AgentObservation
    std::shared_ptr<mbam_interfaces::msg::AgentObservation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbam_interfaces__msg__AgentObservation
    std::shared_ptr<mbam_interfaces::msg::AgentObservation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AgentObservation_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->pose_x != other.pose_x) {
      return false;
    }
    if (this->pose_y != other.pose_y) {
      return false;
    }
    if (this->pose_yaw != other.pose_yaw) {
      return false;
    }
    if (this->max_num_targets != other.max_num_targets) {
      return false;
    }
    if (this->z_world_x != other.z_world_x) {
      return false;
    }
    if (this->z_world_y != other.z_world_y) {
      return false;
    }
    if (this->visible != other.visible) {
      return false;
    }
    return true;
  }
  bool operator!=(const AgentObservation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AgentObservation_

// alias to use template instance with default allocator
using AgentObservation =
  mbam_interfaces::msg::AgentObservation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mbam_interfaces

#endif  // MBAM_INTERFACES__MSG__DETAIL__AGENT_OBSERVATION__STRUCT_HPP_
