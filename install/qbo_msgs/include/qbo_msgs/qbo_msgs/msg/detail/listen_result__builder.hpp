// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from qbo_msgs:msg/ListenResult.idl
// generated code does not contain a copyright notice

#ifndef QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__BUILDER_HPP_
#define QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "qbo_msgs/msg/detail/listen_result__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace qbo_msgs
{

namespace msg
{

namespace builder
{

class Init_ListenResult_confidence
{
public:
  explicit Init_ListenResult_confidence(::qbo_msgs::msg::ListenResult & msg)
  : msg_(msg)
  {}
  ::qbo_msgs::msg::ListenResult confidence(::qbo_msgs::msg::ListenResult::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::qbo_msgs::msg::ListenResult msg_;
};

class Init_ListenResult_sentence
{
public:
  Init_ListenResult_sentence()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ListenResult_confidence sentence(::qbo_msgs::msg::ListenResult::_sentence_type arg)
  {
    msg_.sentence = std::move(arg);
    return Init_ListenResult_confidence(msg_);
  }

private:
  ::qbo_msgs::msg::ListenResult msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::qbo_msgs::msg::ListenResult>()
{
  return qbo_msgs::msg::builder::Init_ListenResult_sentence();
}

}  // namespace qbo_msgs

#endif  // QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__BUILDER_HPP_
