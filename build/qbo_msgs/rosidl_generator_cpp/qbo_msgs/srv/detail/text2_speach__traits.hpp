// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from qbo_msgs:srv/Text2Speach.idl
// generated code does not contain a copyright notice

#ifndef QBO_MSGS__SRV__DETAIL__TEXT2_SPEACH__TRAITS_HPP_
#define QBO_MSGS__SRV__DETAIL__TEXT2_SPEACH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "qbo_msgs/srv/detail/text2_speach__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace qbo_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Text2Speach_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: sentence
  {
    out << "sentence: ";
    rosidl_generator_traits::value_to_yaml(msg.sentence, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Text2Speach_Request & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Text2Speach_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace qbo_msgs

namespace rosidl_generator_traits
{

[[deprecated("use qbo_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const qbo_msgs::srv::Text2Speach_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  qbo_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use qbo_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const qbo_msgs::srv::Text2Speach_Request & msg)
{
  return qbo_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<qbo_msgs::srv::Text2Speach_Request>()
{
  return "qbo_msgs::srv::Text2Speach_Request";
}

template<>
inline const char * name<qbo_msgs::srv::Text2Speach_Request>()
{
  return "qbo_msgs/srv/Text2Speach_Request";
}

template<>
struct has_fixed_size<qbo_msgs::srv::Text2Speach_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<qbo_msgs::srv::Text2Speach_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<qbo_msgs::srv::Text2Speach_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace qbo_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Text2Speach_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Text2Speach_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Text2Speach_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace qbo_msgs

namespace rosidl_generator_traits
{

[[deprecated("use qbo_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const qbo_msgs::srv::Text2Speach_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  qbo_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use qbo_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const qbo_msgs::srv::Text2Speach_Response & msg)
{
  return qbo_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<qbo_msgs::srv::Text2Speach_Response>()
{
  return "qbo_msgs::srv::Text2Speach_Response";
}

template<>
inline const char * name<qbo_msgs::srv::Text2Speach_Response>()
{
  return "qbo_msgs/srv/Text2Speach_Response";
}

template<>
struct has_fixed_size<qbo_msgs::srv::Text2Speach_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<qbo_msgs::srv::Text2Speach_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<qbo_msgs::srv::Text2Speach_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<qbo_msgs::srv::Text2Speach>()
{
  return "qbo_msgs::srv::Text2Speach";
}

template<>
inline const char * name<qbo_msgs::srv::Text2Speach>()
{
  return "qbo_msgs/srv/Text2Speach";
}

template<>
struct has_fixed_size<qbo_msgs::srv::Text2Speach>
  : std::integral_constant<
    bool,
    has_fixed_size<qbo_msgs::srv::Text2Speach_Request>::value &&
    has_fixed_size<qbo_msgs::srv::Text2Speach_Response>::value
  >
{
};

template<>
struct has_bounded_size<qbo_msgs::srv::Text2Speach>
  : std::integral_constant<
    bool,
    has_bounded_size<qbo_msgs::srv::Text2Speach_Request>::value &&
    has_bounded_size<qbo_msgs::srv::Text2Speach_Response>::value
  >
{
};

template<>
struct is_service<qbo_msgs::srv::Text2Speach>
  : std::true_type
{
};

template<>
struct is_service_request<qbo_msgs::srv::Text2Speach_Request>
  : std::true_type
{
};

template<>
struct is_service_response<qbo_msgs::srv::Text2Speach_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // QBO_MSGS__SRV__DETAIL__TEXT2_SPEACH__TRAITS_HPP_
