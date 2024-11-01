// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from test_package_msgs:msg/ArucoMarker.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE_MSGS__MSG__DETAIL__ARUCO_MARKER__TRAITS_HPP_
#define TEST_PACKAGE_MSGS__MSG__DETAIL__ARUCO_MARKER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "test_package_msgs/msg/detail/aruco_marker__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace test_package_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ArucoMarker & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArucoMarker & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArucoMarker & msg, bool use_flow_style = false)
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

}  // namespace test_package_msgs

namespace rosidl_generator_traits
{

[[deprecated("use test_package_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const test_package_msgs::msg::ArucoMarker & msg,
  std::ostream & out, size_t indentation = 0)
{
  test_package_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use test_package_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const test_package_msgs::msg::ArucoMarker & msg)
{
  return test_package_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<test_package_msgs::msg::ArucoMarker>()
{
  return "test_package_msgs::msg::ArucoMarker";
}

template<>
inline const char * name<test_package_msgs::msg::ArucoMarker>()
{
  return "test_package_msgs/msg/ArucoMarker";
}

template<>
struct has_fixed_size<test_package_msgs::msg::ArucoMarker>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<test_package_msgs::msg::ArucoMarker>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<test_package_msgs::msg::ArucoMarker>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TEST_PACKAGE_MSGS__MSG__DETAIL__ARUCO_MARKER__TRAITS_HPP_
