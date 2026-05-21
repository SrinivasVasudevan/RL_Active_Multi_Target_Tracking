// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mbam_interfaces:msg/AgentObservation.idl
// generated code does not contain a copyright notice

#ifndef MBAM_INTERFACES__MSG__DETAIL__AGENT_OBSERVATION__BUILDER_HPP_
#define MBAM_INTERFACES__MSG__DETAIL__AGENT_OBSERVATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mbam_interfaces/msg/detail/agent_observation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mbam_interfaces
{

namespace msg
{

namespace builder
{

class Init_AgentObservation_visible
{
public:
  explicit Init_AgentObservation_visible(::mbam_interfaces::msg::AgentObservation & msg)
  : msg_(msg)
  {}
  ::mbam_interfaces::msg::AgentObservation visible(::mbam_interfaces::msg::AgentObservation::_visible_type arg)
  {
    msg_.visible = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbam_interfaces::msg::AgentObservation msg_;
};

class Init_AgentObservation_z_world_y
{
public:
  explicit Init_AgentObservation_z_world_y(::mbam_interfaces::msg::AgentObservation & msg)
  : msg_(msg)
  {}
  Init_AgentObservation_visible z_world_y(::mbam_interfaces::msg::AgentObservation::_z_world_y_type arg)
  {
    msg_.z_world_y = std::move(arg);
    return Init_AgentObservation_visible(msg_);
  }

private:
  ::mbam_interfaces::msg::AgentObservation msg_;
};

class Init_AgentObservation_z_world_x
{
public:
  explicit Init_AgentObservation_z_world_x(::mbam_interfaces::msg::AgentObservation & msg)
  : msg_(msg)
  {}
  Init_AgentObservation_z_world_y z_world_x(::mbam_interfaces::msg::AgentObservation::_z_world_x_type arg)
  {
    msg_.z_world_x = std::move(arg);
    return Init_AgentObservation_z_world_y(msg_);
  }

private:
  ::mbam_interfaces::msg::AgentObservation msg_;
};

class Init_AgentObservation_max_num_targets
{
public:
  explicit Init_AgentObservation_max_num_targets(::mbam_interfaces::msg::AgentObservation & msg)
  : msg_(msg)
  {}
  Init_AgentObservation_z_world_x max_num_targets(::mbam_interfaces::msg::AgentObservation::_max_num_targets_type arg)
  {
    msg_.max_num_targets = std::move(arg);
    return Init_AgentObservation_z_world_x(msg_);
  }

private:
  ::mbam_interfaces::msg::AgentObservation msg_;
};

class Init_AgentObservation_pose_yaw
{
public:
  explicit Init_AgentObservation_pose_yaw(::mbam_interfaces::msg::AgentObservation & msg)
  : msg_(msg)
  {}
  Init_AgentObservation_max_num_targets pose_yaw(::mbam_interfaces::msg::AgentObservation::_pose_yaw_type arg)
  {
    msg_.pose_yaw = std::move(arg);
    return Init_AgentObservation_max_num_targets(msg_);
  }

private:
  ::mbam_interfaces::msg::AgentObservation msg_;
};

class Init_AgentObservation_pose_y
{
public:
  explicit Init_AgentObservation_pose_y(::mbam_interfaces::msg::AgentObservation & msg)
  : msg_(msg)
  {}
  Init_AgentObservation_pose_yaw pose_y(::mbam_interfaces::msg::AgentObservation::_pose_y_type arg)
  {
    msg_.pose_y = std::move(arg);
    return Init_AgentObservation_pose_yaw(msg_);
  }

private:
  ::mbam_interfaces::msg::AgentObservation msg_;
};

class Init_AgentObservation_pose_x
{
public:
  explicit Init_AgentObservation_pose_x(::mbam_interfaces::msg::AgentObservation & msg)
  : msg_(msg)
  {}
  Init_AgentObservation_pose_y pose_x(::mbam_interfaces::msg::AgentObservation::_pose_x_type arg)
  {
    msg_.pose_x = std::move(arg);
    return Init_AgentObservation_pose_y(msg_);
  }

private:
  ::mbam_interfaces::msg::AgentObservation msg_;
};

class Init_AgentObservation_robot_id
{
public:
  explicit Init_AgentObservation_robot_id(::mbam_interfaces::msg::AgentObservation & msg)
  : msg_(msg)
  {}
  Init_AgentObservation_pose_x robot_id(::mbam_interfaces::msg::AgentObservation::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_AgentObservation_pose_x(msg_);
  }

private:
  ::mbam_interfaces::msg::AgentObservation msg_;
};

class Init_AgentObservation_header
{
public:
  Init_AgentObservation_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AgentObservation_robot_id header(::mbam_interfaces::msg::AgentObservation::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AgentObservation_robot_id(msg_);
  }

private:
  ::mbam_interfaces::msg::AgentObservation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbam_interfaces::msg::AgentObservation>()
{
  return mbam_interfaces::msg::builder::Init_AgentObservation_header();
}

}  // namespace mbam_interfaces

#endif  // MBAM_INTERFACES__MSG__DETAIL__AGENT_OBSERVATION__BUILDER_HPP_
