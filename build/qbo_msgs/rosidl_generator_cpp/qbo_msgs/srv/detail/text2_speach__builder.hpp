// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from qbo_msgs:srv/Text2Speach.idl
// generated code does not contain a copyright notice

#ifndef QBO_MSGS__SRV__DETAIL__TEXT2_SPEACH__BUILDER_HPP_
#define QBO_MSGS__SRV__DETAIL__TEXT2_SPEACH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "qbo_msgs/srv/detail/text2_speach__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace qbo_msgs
{

namespace srv
{

namespace builder
{

class Init_Text2Speach_Request_sentence
{
public:
  Init_Text2Speach_Request_sentence()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::qbo_msgs::srv::Text2Speach_Request sentence(::qbo_msgs::srv::Text2Speach_Request::_sentence_type arg)
  {
    msg_.sentence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::qbo_msgs::srv::Text2Speach_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::qbo_msgs::srv::Text2Speach_Request>()
{
  return qbo_msgs::srv::builder::Init_Text2Speach_Request_sentence();
}

}  // namespace qbo_msgs


namespace qbo_msgs
{

namespace srv
{

namespace builder
{

class Init_Text2Speach_Response_success
{
public:
  Init_Text2Speach_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::qbo_msgs::srv::Text2Speach_Response success(::qbo_msgs::srv::Text2Speach_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::qbo_msgs::srv::Text2Speach_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::qbo_msgs::srv::Text2Speach_Response>()
{
  return qbo_msgs::srv::builder::Init_Text2Speach_Response_success();
}

}  // namespace qbo_msgs

#endif  // QBO_MSGS__SRV__DETAIL__TEXT2_SPEACH__BUILDER_HPP_
