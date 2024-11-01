// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from test_package_msgs:msg/ArucoMarker.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE_MSGS__MSG__DETAIL__ARUCO_MARKER__STRUCT_HPP_
#define TEST_PACKAGE_MSGS__MSG__DETAIL__ARUCO_MARKER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__test_package_msgs__msg__ArucoMarker __attribute__((deprecated))
#else
# define DEPRECATED__test_package_msgs__msg__ArucoMarker __declspec(deprecated)
#endif

namespace test_package_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ArucoMarker_
{
  using Type = ArucoMarker_<ContainerAllocator>;

  explicit ArucoMarker_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->distance = 0.0f;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->yaw = 0.0f;
      this->pitch = 0.0f;
      this->roll = 0.0f;
    }
  }

  explicit ArucoMarker_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->distance = 0.0f;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->yaw = 0.0f;
      this->pitch = 0.0f;
      this->roll = 0.0f;
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _distance_type =
    float;
  _distance_type distance;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _pitch_type =
    float;
  _pitch_type pitch;
  using _roll_type =
    float;
  _roll_type roll;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__pitch(
    const float & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__roll(
    const float & _arg)
  {
    this->roll = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    test_package_msgs::msg::ArucoMarker_<ContainerAllocator> *;
  using ConstRawPtr =
    const test_package_msgs::msg::ArucoMarker_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<test_package_msgs::msg::ArucoMarker_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<test_package_msgs::msg::ArucoMarker_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      test_package_msgs::msg::ArucoMarker_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<test_package_msgs::msg::ArucoMarker_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      test_package_msgs::msg::ArucoMarker_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<test_package_msgs::msg::ArucoMarker_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<test_package_msgs::msg::ArucoMarker_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<test_package_msgs::msg::ArucoMarker_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__test_package_msgs__msg__ArucoMarker
    std::shared_ptr<test_package_msgs::msg::ArucoMarker_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__test_package_msgs__msg__ArucoMarker
    std::shared_ptr<test_package_msgs::msg::ArucoMarker_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArucoMarker_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArucoMarker_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArucoMarker_

// alias to use template instance with default allocator
using ArucoMarker =
  test_package_msgs::msg::ArucoMarker_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace test_package_msgs

#endif  // TEST_PACKAGE_MSGS__MSG__DETAIL__ARUCO_MARKER__STRUCT_HPP_
