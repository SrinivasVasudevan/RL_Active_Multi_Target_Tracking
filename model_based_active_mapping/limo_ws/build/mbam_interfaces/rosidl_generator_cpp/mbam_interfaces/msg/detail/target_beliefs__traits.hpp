// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mbam_interfaces:msg/TargetBeliefs.idl
// generated code does not contain a copyright notice

#ifndef MBAM_INTERFACES__MSG__DETAIL__TARGET_BELIEFS__TRAITS_HPP_
#define MBAM_INTERFACES__MSG__DETAIL__TARGET_BELIEFS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mbam_interfaces/msg/detail/target_beliefs__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mbam_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TargetBeliefs & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: max_num_targets
  {
    out << "max_num_targets: ";
    rosidl_generator_traits::value_to_yaml(msg.max_num_targets, out);
    out << ", ";
  }

  // member: mean_x
  {
    if (msg.mean_x.size() == 0) {
      out << "mean_x: []";
    } else {
      out << "mean_x: [";
      size_t pending_items = msg.mean_x.size();
      for (auto item : msg.mean_x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: mean_y
  {
    if (msg.mean_y.size() == 0) {
      out << "mean_y: []";
    } else {
      out << "mean_y: [";
      size_t pending_items = msg.mean_y.size();
      for (auto item : msg.mean_y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: active_mask
  {
    if (msg.active_mask.size() == 0) {
      out << "active_mask: []";
    } else {
      out << "active_mask: [";
      size_t pending_items = msg.active_mask.size();
      for (auto item : msg.active_mask) {
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
  const TargetBeliefs & msg,
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

  // member: max_num_targets
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_num_targets: ";
    rosidl_generator_traits::value_to_yaml(msg.max_num_targets, out);
    out << "\n";
  }

  // member: mean_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.mean_x.size() == 0) {
      out << "mean_x: []\n";
    } else {
      out << "mean_x:\n";
      for (auto item : msg.mean_x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: mean_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.mean_y.size() == 0) {
      out << "mean_y: []\n";
    } else {
      out << "mean_y:\n";
      for (auto item : msg.mean_y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: active_mask
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.active_mask.size() == 0) {
      out << "active_mask: []\n";
    } else {
      out << "active_mask:\n";
      for (auto item : msg.active_mask) {
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

inline std::string to_yaml(const TargetBeliefs & msg, bool use_flow_style = false)
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
  const mbam_interfaces::msg::TargetBeliefs & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbam_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbam_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const mbam_interfaces::msg::TargetBeliefs & msg)
{
  return mbam_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mbam_interfaces::msg::TargetBeliefs>()
{
  return "mbam_interfaces::msg::TargetBeliefs";
}

template<>
inline const char * name<mbam_interfaces::msg::TargetBeliefs>()
{
  return "mbam_interfaces/msg/TargetBeliefs";
}

template<>
struct has_fixed_size<mbam_interfaces::msg::TargetBeliefs>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mbam_interfaces::msg::TargetBeliefs>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mbam_interfaces::msg::TargetBeliefs>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MBAM_INTERFACES__MSG__DETAIL__TARGET_BELIEFS__TRAITS_HPP_
