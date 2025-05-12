// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from qbo_msgs:msg/ListenResult.idl
// generated code does not contain a copyright notice

#ifndef QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__STRUCT_H_
#define QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'sentence'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ListenResult in the package qbo_msgs.
typedef struct qbo_msgs__msg__ListenResult
{
  /// transcription complète
  rosidl_runtime_c__String sentence;
  /// 0.0 – 1.0  (≈ probabilité moyenne par token)
  float confidence;
} qbo_msgs__msg__ListenResult;

// Struct for a sequence of qbo_msgs__msg__ListenResult.
typedef struct qbo_msgs__msg__ListenResult__Sequence
{
  qbo_msgs__msg__ListenResult * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} qbo_msgs__msg__ListenResult__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__STRUCT_H_
