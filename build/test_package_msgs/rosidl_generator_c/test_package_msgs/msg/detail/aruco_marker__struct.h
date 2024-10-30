// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from test_package_msgs:msg/ArucoMarker.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE_MSGS__MSG__DETAIL__ARUCO_MARKER__STRUCT_H_
#define TEST_PACKAGE_MSGS__MSG__DETAIL__ARUCO_MARKER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ArucoMarker in the package test_package_msgs.
typedef struct test_package_msgs__msg__ArucoMarker
{
  int32_t id;
  float distance;
  float x;
  float y;
  float z;
} test_package_msgs__msg__ArucoMarker;

// Struct for a sequence of test_package_msgs__msg__ArucoMarker.
typedef struct test_package_msgs__msg__ArucoMarker__Sequence
{
  test_package_msgs__msg__ArucoMarker * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} test_package_msgs__msg__ArucoMarker__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TEST_PACKAGE_MSGS__MSG__DETAIL__ARUCO_MARKER__STRUCT_H_
