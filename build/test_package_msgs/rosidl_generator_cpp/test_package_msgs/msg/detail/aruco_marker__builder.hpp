// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from test_package_msgs:msg/ArucoMarker.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE_MSGS__MSG__DETAIL__ARUCO_MARKER__BUILDER_HPP_
#define TEST_PACKAGE_MSGS__MSG__DETAIL__ARUCO_MARKER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "test_package_msgs/msg/detail/aruco_marker__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace test_package_msgs
{

namespace msg
{

namespace builder
{

class Init_ArucoMarker_roll
{
public:
  explicit Init_ArucoMarker_roll(::test_package_msgs::msg::ArucoMarker & msg)
  : msg_(msg)
  {}
  ::test_package_msgs::msg::ArucoMarker roll(::test_package_msgs::msg::ArucoMarker::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package_msgs::msg::ArucoMarker msg_;
};

class Init_ArucoMarker_pitch
{
public:
  explicit Init_ArucoMarker_pitch(::test_package_msgs::msg::ArucoMarker & msg)
  : msg_(msg)
  {}
  Init_ArucoMarker_roll pitch(::test_package_msgs::msg::ArucoMarker::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_ArucoMarker_roll(msg_);
  }

private:
  ::test_package_msgs::msg::ArucoMarker msg_;
};

class Init_ArucoMarker_yaw
{
public:
  explicit Init_ArucoMarker_yaw(::test_package_msgs::msg::ArucoMarker & msg)
  : msg_(msg)
  {}
  Init_ArucoMarker_pitch yaw(::test_package_msgs::msg::ArucoMarker::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_ArucoMarker_pitch(msg_);
  }

private:
  ::test_package_msgs::msg::ArucoMarker msg_;
};

class Init_ArucoMarker_z
{
public:
  explicit Init_ArucoMarker_z(::test_package_msgs::msg::ArucoMarker & msg)
  : msg_(msg)
  {}
  Init_ArucoMarker_yaw z(::test_package_msgs::msg::ArucoMarker::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_ArucoMarker_yaw(msg_);
  }

private:
  ::test_package_msgs::msg::ArucoMarker msg_;
};

class Init_ArucoMarker_y
{
public:
  explicit Init_ArucoMarker_y(::test_package_msgs::msg::ArucoMarker & msg)
  : msg_(msg)
  {}
  Init_ArucoMarker_z y(::test_package_msgs::msg::ArucoMarker::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_ArucoMarker_z(msg_);
  }

private:
  ::test_package_msgs::msg::ArucoMarker msg_;
};

class Init_ArucoMarker_x
{
public:
  explicit Init_ArucoMarker_x(::test_package_msgs::msg::ArucoMarker & msg)
  : msg_(msg)
  {}
  Init_ArucoMarker_y x(::test_package_msgs::msg::ArucoMarker::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_ArucoMarker_y(msg_);
  }

private:
  ::test_package_msgs::msg::ArucoMarker msg_;
};

class Init_ArucoMarker_distance
{
public:
  explicit Init_ArucoMarker_distance(::test_package_msgs::msg::ArucoMarker & msg)
  : msg_(msg)
  {}
  Init_ArucoMarker_x distance(::test_package_msgs::msg::ArucoMarker::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_ArucoMarker_x(msg_);
  }

private:
  ::test_package_msgs::msg::ArucoMarker msg_;
};

class Init_ArucoMarker_id
{
public:
  Init_ArucoMarker_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArucoMarker_distance id(::test_package_msgs::msg::ArucoMarker::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_ArucoMarker_distance(msg_);
  }

private:
  ::test_package_msgs::msg::ArucoMarker msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package_msgs::msg::ArucoMarker>()
{
  return test_package_msgs::msg::builder::Init_ArucoMarker_id();
}

}  // namespace test_package_msgs

#endif  // TEST_PACKAGE_MSGS__MSG__DETAIL__ARUCO_MARKER__BUILDER_HPP_
