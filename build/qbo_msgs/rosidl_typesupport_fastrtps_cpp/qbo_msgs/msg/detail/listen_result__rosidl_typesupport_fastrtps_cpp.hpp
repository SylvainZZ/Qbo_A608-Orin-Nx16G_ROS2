// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from qbo_msgs:msg/ListenResult.idl
// generated code does not contain a copyright notice

#ifndef QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "qbo_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "qbo_msgs/msg/detail/listen_result__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace qbo_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_qbo_msgs
cdr_serialize(
  const qbo_msgs::msg::ListenResult & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_qbo_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  qbo_msgs::msg::ListenResult & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_qbo_msgs
get_serialized_size(
  const qbo_msgs::msg::ListenResult & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_qbo_msgs
max_serialized_size_ListenResult(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace qbo_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_qbo_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, qbo_msgs, msg, ListenResult)();

#ifdef __cplusplus
}
#endif

#endif  // QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
