// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mbam_interfaces:msg/TargetBeliefs.idl
// generated code does not contain a copyright notice

#ifndef MBAM_INTERFACES__MSG__DETAIL__TARGET_BELIEFS__STRUCT_HPP_
#define MBAM_INTERFACES__MSG__DETAIL__TARGET_BELIEFS__STRUCT_HPP_

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
# define DEPRECATED__mbam_interfaces__msg__TargetBeliefs __attribute__((deprecated))
#else
# define DEPRECATED__mbam_interfaces__msg__TargetBeliefs __declspec(deprecated)
#endif

namespace mbam_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TargetBeliefs_
{
  using Type = TargetBeliefs_<ContainerAllocator>;

  explicit TargetBeliefs_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->max_num_targets = 0l;
    }
  }

  explicit TargetBeliefs_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->max_num_targets = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _max_num_targets_type =
    int32_t;
  _max_num_targets_type max_num_targets;
  using _mean_x_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _mean_x_type mean_x;
  using _mean_y_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _mean_y_type mean_y;
  using _active_mask_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _active_mask_type active_mask;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__max_num_targets(
    const int32_t & _arg)
  {
    this->max_num_targets = _arg;
    return *this;
  }
  Type & set__mean_x(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->mean_x = _arg;
    return *this;
  }
  Type & set__mean_y(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->mean_y = _arg;
    return *this;
  }
  Type & set__active_mask(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->active_mask = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbam_interfaces::msg::TargetBeliefs_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbam_interfaces::msg::TargetBeliefs_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbam_interfaces::msg::TargetBeliefs_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbam_interfaces::msg::TargetBeliefs_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbam_interfaces::msg::TargetBeliefs_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbam_interfaces::msg::TargetBeliefs_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbam_interfaces::msg::TargetBeliefs_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbam_interfaces::msg::TargetBeliefs_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbam_interfaces::msg::TargetBeliefs_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbam_interfaces::msg::TargetBeliefs_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbam_interfaces__msg__TargetBeliefs
    std::shared_ptr<mbam_interfaces::msg::TargetBeliefs_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbam_interfaces__msg__TargetBeliefs
    std::shared_ptr<mbam_interfaces::msg::TargetBeliefs_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TargetBeliefs_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->max_num_targets != other.max_num_targets) {
      return false;
    }
    if (this->mean_x != other.mean_x) {
      return false;
    }
    if (this->mean_y != other.mean_y) {
      return false;
    }
    if (this->active_mask != other.active_mask) {
      return false;
    }
    return true;
  }
  bool operator!=(const TargetBeliefs_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TargetBeliefs_

// alias to use template instance with default allocator
using TargetBeliefs =
  mbam_interfaces::msg::TargetBeliefs_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mbam_interfaces

#endif  // MBAM_INTERFACES__MSG__DETAIL__TARGET_BELIEFS__STRUCT_HPP_
