// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from qbo_msgs:srv/Text2Speach.idl
// generated code does not contain a copyright notice

#ifndef QBO_MSGS__SRV__DETAIL__TEXT2_SPEACH__STRUCT_HPP_
#define QBO_MSGS__SRV__DETAIL__TEXT2_SPEACH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__qbo_msgs__srv__Text2Speach_Request __attribute__((deprecated))
#else
# define DEPRECATED__qbo_msgs__srv__Text2Speach_Request __declspec(deprecated)
#endif

namespace qbo_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Text2Speach_Request_
{
  using Type = Text2Speach_Request_<ContainerAllocator>;

  explicit Text2Speach_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sentence = "";
    }
  }

  explicit Text2Speach_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sentence(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sentence = "";
    }
  }

  // field types and members
  using _sentence_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sentence_type sentence;

  // setters for named parameter idiom
  Type & set__sentence(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sentence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    qbo_msgs::srv::Text2Speach_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const qbo_msgs::srv::Text2Speach_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<qbo_msgs::srv::Text2Speach_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<qbo_msgs::srv::Text2Speach_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      qbo_msgs::srv::Text2Speach_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<qbo_msgs::srv::Text2Speach_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      qbo_msgs::srv::Text2Speach_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<qbo_msgs::srv::Text2Speach_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<qbo_msgs::srv::Text2Speach_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<qbo_msgs::srv::Text2Speach_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__qbo_msgs__srv__Text2Speach_Request
    std::shared_ptr<qbo_msgs::srv::Text2Speach_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__qbo_msgs__srv__Text2Speach_Request
    std::shared_ptr<qbo_msgs::srv::Text2Speach_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Text2Speach_Request_ & other) const
  {
    if (this->sentence != other.sentence) {
      return false;
    }
    return true;
  }
  bool operator!=(const Text2Speach_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Text2Speach_Request_

// alias to use template instance with default allocator
using Text2Speach_Request =
  qbo_msgs::srv::Text2Speach_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace qbo_msgs


#ifndef _WIN32
# define DEPRECATED__qbo_msgs__srv__Text2Speach_Response __attribute__((deprecated))
#else
# define DEPRECATED__qbo_msgs__srv__Text2Speach_Response __declspec(deprecated)
#endif

namespace qbo_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Text2Speach_Response_
{
  using Type = Text2Speach_Response_<ContainerAllocator>;

  explicit Text2Speach_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit Text2Speach_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    qbo_msgs::srv::Text2Speach_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const qbo_msgs::srv::Text2Speach_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<qbo_msgs::srv::Text2Speach_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<qbo_msgs::srv::Text2Speach_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      qbo_msgs::srv::Text2Speach_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<qbo_msgs::srv::Text2Speach_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      qbo_msgs::srv::Text2Speach_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<qbo_msgs::srv::Text2Speach_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<qbo_msgs::srv::Text2Speach_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<qbo_msgs::srv::Text2Speach_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__qbo_msgs__srv__Text2Speach_Response
    std::shared_ptr<qbo_msgs::srv::Text2Speach_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__qbo_msgs__srv__Text2Speach_Response
    std::shared_ptr<qbo_msgs::srv::Text2Speach_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Text2Speach_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const Text2Speach_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Text2Speach_Response_

// alias to use template instance with default allocator
using Text2Speach_Response =
  qbo_msgs::srv::Text2Speach_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace qbo_msgs

namespace qbo_msgs
{

namespace srv
{

struct Text2Speach
{
  using Request = qbo_msgs::srv::Text2Speach_Request;
  using Response = qbo_msgs::srv::Text2Speach_Response;
};

}  // namespace srv

}  // namespace qbo_msgs

#endif  // QBO_MSGS__SRV__DETAIL__TEXT2_SPEACH__STRUCT_HPP_
