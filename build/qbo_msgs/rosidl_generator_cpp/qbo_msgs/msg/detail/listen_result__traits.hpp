// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from qbo_msgs:msg/ListenResult.idl
// generated code does not contain a copyright notice

#ifndef QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__TRAITS_HPP_
#define QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "qbo_msgs/msg/detail/listen_result__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace qbo_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ListenResult & msg,
  std::ostream & out)
{
  out << "{";
  // member: sentence
  {
    out << "sentence: ";
    rosidl_generator_traits::value_to_yaml(msg.sentence, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ListenResult & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sentence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sentence: ";
    rosidl_generator_traits::value_to_yaml(msg.sentence, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ListenResult & msg, bool use_flow_style = false)
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

}  // namespace qbo_msgs

namespace rosidl_generator_traits
{

[[deprecated("use qbo_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const qbo_msgs::msg::ListenResult & msg,
  std::ostream & out, size_t indentation = 0)
{
  qbo_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use qbo_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const qbo_msgs::msg::ListenResult & msg)
{
  return qbo_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<qbo_msgs::msg::ListenResult>()
{
  return "qbo_msgs::msg::ListenResult";
}

template<>
inline const char * name<qbo_msgs::msg::ListenResult>()
{
  return "qbo_msgs/msg/ListenResult";
}

template<>
struct has_fixed_size<qbo_msgs::msg::ListenResult>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<qbo_msgs::msg::ListenResult>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<qbo_msgs::msg::ListenResult>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__TRAITS_HPP_
