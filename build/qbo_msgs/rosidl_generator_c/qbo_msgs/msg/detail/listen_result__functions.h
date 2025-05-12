// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from qbo_msgs:msg/ListenResult.idl
// generated code does not contain a copyright notice

#ifndef QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__FUNCTIONS_H_
#define QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "qbo_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "qbo_msgs/msg/detail/listen_result__struct.h"

/// Initialize msg/ListenResult message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * qbo_msgs__msg__ListenResult
 * )) before or use
 * qbo_msgs__msg__ListenResult__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_qbo_msgs
bool
qbo_msgs__msg__ListenResult__init(qbo_msgs__msg__ListenResult * msg);

/// Finalize msg/ListenResult message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_qbo_msgs
void
qbo_msgs__msg__ListenResult__fini(qbo_msgs__msg__ListenResult * msg);

/// Create msg/ListenResult message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * qbo_msgs__msg__ListenResult__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_qbo_msgs
qbo_msgs__msg__ListenResult *
qbo_msgs__msg__ListenResult__create();

/// Destroy msg/ListenResult message.
/**
 * It calls
 * qbo_msgs__msg__ListenResult__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_qbo_msgs
void
qbo_msgs__msg__ListenResult__destroy(qbo_msgs__msg__ListenResult * msg);

/// Check for msg/ListenResult message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_qbo_msgs
bool
qbo_msgs__msg__ListenResult__are_equal(const qbo_msgs__msg__ListenResult * lhs, const qbo_msgs__msg__ListenResult * rhs);

/// Copy a msg/ListenResult message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_qbo_msgs
bool
qbo_msgs__msg__ListenResult__copy(
  const qbo_msgs__msg__ListenResult * input,
  qbo_msgs__msg__ListenResult * output);

/// Initialize array of msg/ListenResult messages.
/**
 * It allocates the memory for the number of elements and calls
 * qbo_msgs__msg__ListenResult__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_qbo_msgs
bool
qbo_msgs__msg__ListenResult__Sequence__init(qbo_msgs__msg__ListenResult__Sequence * array, size_t size);

/// Finalize array of msg/ListenResult messages.
/**
 * It calls
 * qbo_msgs__msg__ListenResult__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_qbo_msgs
void
qbo_msgs__msg__ListenResult__Sequence__fini(qbo_msgs__msg__ListenResult__Sequence * array);

/// Create array of msg/ListenResult messages.
/**
 * It allocates the memory for the array and calls
 * qbo_msgs__msg__ListenResult__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_qbo_msgs
qbo_msgs__msg__ListenResult__Sequence *
qbo_msgs__msg__ListenResult__Sequence__create(size_t size);

/// Destroy array of msg/ListenResult messages.
/**
 * It calls
 * qbo_msgs__msg__ListenResult__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_qbo_msgs
void
qbo_msgs__msg__ListenResult__Sequence__destroy(qbo_msgs__msg__ListenResult__Sequence * array);

/// Check for msg/ListenResult message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_qbo_msgs
bool
qbo_msgs__msg__ListenResult__Sequence__are_equal(const qbo_msgs__msg__ListenResult__Sequence * lhs, const qbo_msgs__msg__ListenResult__Sequence * rhs);

/// Copy an array of msg/ListenResult messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_qbo_msgs
bool
qbo_msgs__msg__ListenResult__Sequence__copy(
  const qbo_msgs__msg__ListenResult__Sequence * input,
  qbo_msgs__msg__ListenResult__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // QBO_MSGS__MSG__DETAIL__LISTEN_RESULT__FUNCTIONS_H_
