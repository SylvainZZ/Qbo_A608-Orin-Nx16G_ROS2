// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from qbo_msgs:msg/ListenResult.idl
// generated code does not contain a copyright notice

#ifndef QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__STRUCT_HPP_
#define QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__qbo_msgs__msg__ListenResult __attribute__((deprecated))
#else
# define DEPRECATED__qbo_msgs__msg__ListenResult __declspec(deprecated)
#endif

namespace qbo_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ListenResult_
{
  using Type = ListenResult_<ContainerAllocator>;

  explicit ListenResult_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sentence = "";
      this->confidence = 0.0f;
    }
  }

  explicit ListenResult_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sentence(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sentence = "";
      this->confidence = 0.0f;
    }
  }

  // field types and members
  using _sentence_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sentence_type sentence;
  using _confidence_type =
    float;
  _confidence_type confidence;

  // setters for named parameter idiom
  Type & set__sentence(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sentence = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    qbo_msgs::msg::ListenResult_<ContainerAllocator> *;
  using ConstRawPtr =
    const qbo_msgs::msg::ListenResult_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<qbo_msgs::msg::ListenResult_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<qbo_msgs::msg::ListenResult_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      qbo_msgs::msg::ListenResult_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<qbo_msgs::msg::ListenResult_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      qbo_msgs::msg::ListenResult_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<qbo_msgs::msg::ListenResult_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<qbo_msgs::msg::ListenResult_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<qbo_msgs::msg::ListenResult_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__qbo_msgs__msg__ListenResult
    std::shared_ptr<qbo_msgs::msg::ListenResult_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__qbo_msgs__msg__ListenResult
    std::shared_ptr<qbo_msgs::msg::ListenResult_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ListenResult_ & other) const
  {
    if (this->sentence != other.sentence) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const ListenResult_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ListenResult_

// alias to use template instance with default allocator
using ListenResult =
  qbo_msgs::msg::ListenResult_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace qbo_msgs

#endif  // QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__STRUCT_HPP_
