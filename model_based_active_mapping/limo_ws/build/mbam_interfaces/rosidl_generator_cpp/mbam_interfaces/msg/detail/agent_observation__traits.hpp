// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mbam_interfaces:msg/AgentObservation.idl
// generated code does not contain a copyright notice

#ifndef MBAM_INTERFACES__MSG__DETAIL__AGENT_OBSERVATION__TRAITS_HPP_
#define MBAM_INTERFACES__MSG__DETAIL__AGENT_OBSERVATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mbam_interfaces/msg/detail/agent_observation__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mbam_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const AgentObservation & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: pose_x
  {
    out << "pose_x: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_x, out);
    out << ", ";
  }

  // member: pose_y
  {
    out << "pose_y: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_y, out);
    out << ", ";
  }

  // member: pose_yaw
  {
    out << "pose_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_yaw, out);
    out << ", ";
  }

  // member: max_num_targets
  {
    out << "max_num_targets: ";
    rosidl_generator_traits::value_to_yaml(msg.max_num_targets, out);
    out << ", ";
  }

  // member: z_world_x
  {
    if (msg.z_world_x.size() == 0) {
      out << "z_world_x: []";
    } else {
      out << "z_world_x: [";
      size_t pending_items = msg.z_world_x.size();
      for (auto item : msg.z_world_x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: z_world_y
  {
    if (msg.z_world_y.size() == 0) {
      out << "z_world_y: []";
    } else {
      out << "z_world_y: [";
      size_t pending_items = msg.z_world_y.size();
      for (auto item : msg.z_world_y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: visible
  {
    if (msg.visible.size() == 0) {
      out << "visible: []";
    } else {
      out << "visible: [";
      size_t pending_items = msg.visible.size();
      for (auto item : msg.visible) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AgentObservation & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: pose_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_x: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_x, out);
    out << "\n";
  }

  // member: pose_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_y: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_y, out);
    out << "\n";
  }

  // member: pose_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_yaw, out);
    out << "\n";
  }

  // member: max_num_targets
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_num_targets: ";
    rosidl_generator_traits::value_to_yaml(msg.max_num_targets, out);
    out << "\n";
  }

  // member: z_world_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.z_world_x.size() == 0) {
      out << "z_world_x: []\n";
    } else {
      out << "z_world_x:\n";
      for (auto item : msg.z_world_x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: z_world_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.z_world_y.size() == 0) {
      out << "z_world_y: []\n";
    } else {
      out << "z_world_y:\n";
      for (auto item : msg.z_world_y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: visible
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.visible.size() == 0) {
      out << "visible: []\n";
    } else {
      out << "visible:\n";
      for (auto item : msg.visible) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AgentObservation & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mbam_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use mbam_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mbam_interfaces::msg::AgentObservation & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbam_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbam_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const mbam_interfaces::msg::AgentObservation & msg)
{
  return mbam_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mbam_interfaces::msg::AgentObservation>()
{
  return "mbam_interfaces::msg::AgentObservation";
}

template<>
inline const char * name<mbam_interfaces::msg::AgentObservation>()
{
  return "mbam_interfaces/msg/AgentObservation";
}

template<>
struct has_fixed_size<mbam_interfaces::msg::AgentObservation>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mbam_interfaces::msg::AgentObservation>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mbam_interfaces::msg::AgentObservation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MBAM_INTERFACES__MSG__DETAIL__AGENT_OBSERVATION__TRAITS_HPP_
