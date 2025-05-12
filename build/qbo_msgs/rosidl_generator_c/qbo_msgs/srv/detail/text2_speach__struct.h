// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from qbo_msgs:srv/Text2Speach.idl
// generated code does not contain a copyright notice

#ifndef QBO_MSGS__SRV__DETAIL__TEXT2_SPEACH__STRUCT_H_
#define QBO_MSGS__SRV__DETAIL__TEXT2_SPEACH__STRUCT_H_

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

/// Struct defined in srv/Text2Speach in the package qbo_msgs.
typedef struct qbo_msgs__srv__Text2Speach_Request
{
  rosidl_runtime_c__String sentence;
} qbo_msgs__srv__Text2Speach_Request;

// Struct for a sequence of qbo_msgs__srv__Text2Speach_Request.
typedef struct qbo_msgs__srv__Text2Speach_Request__Sequence
{
  qbo_msgs__srv__Text2Speach_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} qbo_msgs__srv__Text2Speach_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Text2Speach in the package qbo_msgs.
typedef struct qbo_msgs__srv__Text2Speach_Response
{
  bool success;
} qbo_msgs__srv__Text2Speach_Response;

// Struct for a sequence of qbo_msgs__srv__Text2Speach_Response.
typedef struct qbo_msgs__srv__Text2Speach_Response__Sequence
{
  qbo_msgs__srv__Text2Speach_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} qbo_msgs__srv__Text2Speach_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QBO_MSGS__SRV__DETAIL__TEXT2_SPEACH__STRUCT_H_
