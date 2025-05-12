// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from qbo_msgs:msg/ListenResult.idl
// generated code does not contain a copyright notice
#include "qbo_msgs/msg/detail/listen_result__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `sentence`
#include "rosidl_runtime_c/string_functions.h"

bool
qbo_msgs__msg__ListenResult__init(qbo_msgs__msg__ListenResult * msg)
{
  if (!msg) {
    return false;
  }
  // sentence
  if (!rosidl_runtime_c__String__init(&msg->sentence)) {
    qbo_msgs__msg__ListenResult__fini(msg);
    return false;
  }
  // confidence
  return true;
}

void
qbo_msgs__msg__ListenResult__fini(qbo_msgs__msg__ListenResult * msg)
{
  if (!msg) {
    return;
  }
  // sentence
  rosidl_runtime_c__String__fini(&msg->sentence);
  // confidence
}

bool
qbo_msgs__msg__ListenResult__are_equal(const qbo_msgs__msg__ListenResult * lhs, const qbo_msgs__msg__ListenResult * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // sentence
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sentence), &(rhs->sentence)))
  {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  return true;
}

bool
qbo_msgs__msg__ListenResult__copy(
  const qbo_msgs__msg__ListenResult * input,
  qbo_msgs__msg__ListenResult * output)
{
  if (!input || !output) {
    return false;
  }
  // sentence
  if (!rosidl_runtime_c__String__copy(
      &(input->sentence), &(output->sentence)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  return true;
}

qbo_msgs__msg__ListenResult *
qbo_msgs__msg__ListenResult__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  qbo_msgs__msg__ListenResult * msg = (qbo_msgs__msg__ListenResult *)allocator.allocate(sizeof(qbo_msgs__msg__ListenResult), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(qbo_msgs__msg__ListenResult));
  bool success = qbo_msgs__msg__ListenResult__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
qbo_msgs__msg__ListenResult__destroy(qbo_msgs__msg__ListenResult * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    qbo_msgs__msg__ListenResult__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
qbo_msgs__msg__ListenResult__Sequence__init(qbo_msgs__msg__ListenResult__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  qbo_msgs__msg__ListenResult * data = NULL;

  if (size) {
    data = (qbo_msgs__msg__ListenResult *)allocator.zero_allocate(size, sizeof(qbo_msgs__msg__ListenResult), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = qbo_msgs__msg__ListenResult__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        qbo_msgs__msg__ListenResult__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
qbo_msgs__msg__ListenResult__Sequence__fini(qbo_msgs__msg__ListenResult__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      qbo_msgs__msg__ListenResult__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

qbo_msgs__msg__ListenResult__Sequence *
qbo_msgs__msg__ListenResult__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  qbo_msgs__msg__ListenResult__Sequence * array = (qbo_msgs__msg__ListenResult__Sequence *)allocator.allocate(sizeof(qbo_msgs__msg__ListenResult__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = qbo_msgs__msg__ListenResult__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
qbo_msgs__msg__ListenResult__Sequence__destroy(qbo_msgs__msg__ListenResult__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    qbo_msgs__msg__ListenResult__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
qbo_msgs__msg__ListenResult__Sequence__are_equal(const qbo_msgs__msg__ListenResult__Sequence * lhs, const qbo_msgs__msg__ListenResult__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!qbo_msgs__msg__ListenResult__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
qbo_msgs__msg__ListenResult__Sequence__copy(
  const qbo_msgs__msg__ListenResult__Sequence * input,
  qbo_msgs__msg__ListenResult__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(qbo_msgs__msg__ListenResult);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    qbo_msgs__msg__ListenResult * data =
      (qbo_msgs__msg__ListenResult *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!qbo_msgs__msg__ListenResult__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          qbo_msgs__msg__ListenResult__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!qbo_msgs__msg__ListenResult__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
