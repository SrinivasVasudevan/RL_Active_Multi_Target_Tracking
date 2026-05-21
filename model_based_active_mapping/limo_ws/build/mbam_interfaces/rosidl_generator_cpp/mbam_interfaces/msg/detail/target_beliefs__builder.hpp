// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mbam_interfaces:msg/TargetBeliefs.idl
// generated code does not contain a copyright notice

#ifndef MBAM_INTERFACES__MSG__DETAIL__TARGET_BELIEFS__BUILDER_HPP_
#define MBAM_INTERFACES__MSG__DETAIL__TARGET_BELIEFS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mbam_interfaces/msg/detail/target_beliefs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mbam_interfaces
{

namespace msg
{

namespace builder
{

class Init_TargetBeliefs_active_mask
{
public:
  explicit Init_TargetBeliefs_active_mask(::mbam_interfaces::msg::TargetBeliefs & msg)
  : msg_(msg)
  {}
  ::mbam_interfaces::msg::TargetBeliefs active_mask(::mbam_interfaces::msg::TargetBeliefs::_active_mask_type arg)
  {
    msg_.active_mask = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbam_interfaces::msg::TargetBeliefs msg_;
};

class Init_TargetBeliefs_mean_y
{
public:
  explicit Init_TargetBeliefs_mean_y(::mbam_interfaces::msg::TargetBeliefs & msg)
  : msg_(msg)
  {}
  Init_TargetBeliefs_active_mask mean_y(::mbam_interfaces::msg::TargetBeliefs::_mean_y_type arg)
  {
    msg_.mean_y = std::move(arg);
    return Init_TargetBeliefs_active_mask(msg_);
  }

private:
  ::mbam_interfaces::msg::TargetBeliefs msg_;
};

class Init_TargetBeliefs_mean_x
{
public:
  explicit Init_TargetBeliefs_mean_x(::mbam_interfaces::msg::TargetBeliefs & msg)
  : msg_(msg)
  {}
  Init_TargetBeliefs_mean_y mean_x(::mbam_interfaces::msg::TargetBeliefs::_mean_x_type arg)
  {
    msg_.mean_x = std::move(arg);
    return Init_TargetBeliefs_mean_y(msg_);
  }

private:
  ::mbam_interfaces::msg::TargetBeliefs msg_;
};

class Init_TargetBeliefs_max_num_targets
{
public:
  explicit Init_TargetBeliefs_max_num_targets(::mbam_interfaces::msg::TargetBeliefs & msg)
  : msg_(msg)
  {}
  Init_TargetBeliefs_mean_x max_num_targets(::mbam_interfaces::msg::TargetBeliefs::_max_num_targets_type arg)
  {
    msg_.max_num_targets = std::move(arg);
    return Init_TargetBeliefs_mean_x(msg_);
  }

private:
  ::mbam_interfaces::msg::TargetBeliefs msg_;
};

class Init_TargetBeliefs_header
{
public:
  Init_TargetBeliefs_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetBeliefs_max_num_targets header(::mbam_interfaces::msg::TargetBeliefs::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TargetBeliefs_max_num_targets(msg_);
  }

private:
  ::mbam_interfaces::msg::TargetBeliefs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbam_interfaces::msg::TargetBeliefs>()
{
  return mbam_interfaces::msg::builder::Init_TargetBeliefs_header();
}

}  // namespace mbam_interfaces

#endif  // MBAM_INTERFACES__MSG__DETAIL__TARGET_BELIEFS__BUILDER_HPP_
