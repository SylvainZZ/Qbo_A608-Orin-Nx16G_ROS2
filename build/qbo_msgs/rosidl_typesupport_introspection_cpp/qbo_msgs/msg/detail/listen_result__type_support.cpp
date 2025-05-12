// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from qbo_msgs:msg/ListenResult.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "qbo_msgs/msg/detail/listen_result__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace qbo_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ListenResult_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) qbo_msgs::msg::ListenResult(_init);
}

void ListenResult_fini_function(void * message_memory)
{
  auto typed_message = static_cast<qbo_msgs::msg::ListenResult *>(message_memory);
  typed_message->~ListenResult();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ListenResult_message_member_array[2] = {
  {
    "sentence",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(qbo_msgs::msg::ListenResult, sentence),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "confidence",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(qbo_msgs::msg::ListenResult, confidence),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ListenResult_message_members = {
  "qbo_msgs::msg",  // message namespace
  "ListenResult",  // message name
  2,  // number of fields
  sizeof(qbo_msgs::msg::ListenResult),
  ListenResult_message_member_array,  // message members
  ListenResult_init_function,  // function to initialize message memory (memory has to be allocated)
  ListenResult_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ListenResult_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ListenResult_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace qbo_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<qbo_msgs::msg::ListenResult>()
{
  return &::qbo_msgs::msg::rosidl_typesupport_introspection_cpp::ListenResult_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, qbo_msgs, msg, ListenResult)() {
  return &::qbo_msgs::msg::rosidl_typesupport_introspection_cpp::ListenResult_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
