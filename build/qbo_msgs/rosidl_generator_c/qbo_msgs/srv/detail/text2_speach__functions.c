// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from qbo_msgs:srv/Text2Speach.idl
// generated code does not contain a copyright notice
#include "qbo_msgs/srv/detail/text2_speach__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `sentence`
#include "rosidl_runtime_c/string_functions.h"

bool
qbo_msgs__srv__Text2Speach_Request__init(qbo_msgs__srv__Text2Speach_Request * msg)
{
  if (!msg) {
    return false;
  }
  // sentence
  if (!rosidl_runtime_c__String__init(&msg->sentence)) {
    qbo_msgs__srv__Text2Speach_Request__fini(msg);
    return false;
  }
  return true;
}

void
qbo_msgs__srv__Text2Speach_Request__fini(qbo_msgs__srv__Text2Speach_Request * msg)
{
  if (!msg) {
    return;
  }
  // sentence
  rosidl_runtime_c__String__fini(&msg->sentence);
}

bool
qbo_msgs__srv__Text2Speach_Request__are_equal(const qbo_msgs__srv__Text2Speach_Request * lhs, const qbo_msgs__srv__Text2Speach_Request * rhs)
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
  return true;
}

bool
qbo_msgs__srv__Text2Speach_Request__copy(
  const qbo_msgs__srv__Text2Speach_Request * input,
  qbo_msgs__srv__Text2Speach_Request * output)
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
  return true;
}

qbo_msgs__srv__Text2Speach_Request *
qbo_msgs__srv__Text2Speach_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  qbo_msgs__srv__Text2Speach_Request * msg = (qbo_msgs__srv__Text2Speach_Request *)allocator.allocate(sizeof(qbo_msgs__srv__Text2Speach_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(qbo_msgs__srv__Text2Speach_Request));
  bool success = qbo_msgs__srv__Text2Speach_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
qbo_msgs__srv__Text2Speach_Request__destroy(qbo_msgs__srv__Text2Speach_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    qbo_msgs__srv__Text2Speach_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
qbo_msgs__srv__Text2Speach_Request__Sequence__init(qbo_msgs__srv__Text2Speach_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  qbo_msgs__srv__Text2Speach_Request * data = NULL;

  if (size) {
    data = (qbo_msgs__srv__Text2Speach_Request *)allocator.zero_allocate(size, sizeof(qbo_msgs__srv__Text2Speach_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = qbo_msgs__srv__Text2Speach_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        qbo_msgs__srv__Text2Speach_Request__fini(&data[i - 1]);
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
qbo_msgs__srv__Text2Speach_Request__Sequence__fini(qbo_msgs__srv__Text2Speach_Request__Sequence * array)
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
      qbo_msgs__srv__Text2Speach_Request__fini(&array->data[i]);
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

qbo_msgs__srv__Text2Speach_Request__Sequence *
qbo_msgs__srv__Text2Speach_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  qbo_msgs__srv__Text2Speach_Request__Sequence * array = (qbo_msgs__srv__Text2Speach_Request__Sequence *)allocator.allocate(sizeof(qbo_msgs__srv__Text2Speach_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = qbo_msgs__srv__Text2Speach_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
qbo_msgs__srv__Text2Speach_Request__Sequence__destroy(qbo_msgs__srv__Text2Speach_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    qbo_msgs__srv__Text2Speach_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
qbo_msgs__srv__Text2Speach_Request__Sequence__are_equal(const qbo_msgs__srv__Text2Speach_Request__Sequence * lhs, const qbo_msgs__srv__Text2Speach_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!qbo_msgs__srv__Text2Speach_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
qbo_msgs__srv__Text2Speach_Request__Sequence__copy(
  const qbo_msgs__srv__Text2Speach_Request__Sequence * input,
  qbo_msgs__srv__Text2Speach_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(qbo_msgs__srv__Text2Speach_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    qbo_msgs__srv__Text2Speach_Request * data =
      (qbo_msgs__srv__Text2Speach_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!qbo_msgs__srv__Text2Speach_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          qbo_msgs__srv__Text2Speach_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!qbo_msgs__srv__Text2Speach_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
qbo_msgs__srv__Text2Speach_Response__init(qbo_msgs__srv__Text2Speach_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
qbo_msgs__srv__Text2Speach_Response__fini(qbo_msgs__srv__Text2Speach_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
qbo_msgs__srv__Text2Speach_Response__are_equal(const qbo_msgs__srv__Text2Speach_Response * lhs, const qbo_msgs__srv__Text2Speach_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
qbo_msgs__srv__Text2Speach_Response__copy(
  const qbo_msgs__srv__Text2Speach_Response * input,
  qbo_msgs__srv__Text2Speach_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

qbo_msgs__srv__Text2Speach_Response *
qbo_msgs__srv__Text2Speach_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  qbo_msgs__srv__Text2Speach_Response * msg = (qbo_msgs__srv__Text2Speach_Response *)allocator.allocate(sizeof(qbo_msgs__srv__Text2Speach_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(qbo_msgs__srv__Text2Speach_Response));
  bool success = qbo_msgs__srv__Text2Speach_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
qbo_msgs__srv__Text2Speach_Response__destroy(qbo_msgs__srv__Text2Speach_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    qbo_msgs__srv__Text2Speach_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
qbo_msgs__srv__Text2Speach_Response__Sequence__init(qbo_msgs__srv__Text2Speach_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  qbo_msgs__srv__Text2Speach_Response * data = NULL;

  if (size) {
    data = (qbo_msgs__srv__Text2Speach_Response *)allocator.zero_allocate(size, sizeof(qbo_msgs__srv__Text2Speach_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = qbo_msgs__srv__Text2Speach_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        qbo_msgs__srv__Text2Speach_Response__fini(&data[i - 1]);
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
qbo_msgs__srv__Text2Speach_Response__Sequence__fini(qbo_msgs__srv__Text2Speach_Response__Sequence * array)
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
      qbo_msgs__srv__Text2Speach_Response__fini(&array->data[i]);
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

qbo_msgs__srv__Text2Speach_Response__Sequence *
qbo_msgs__srv__Text2Speach_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  qbo_msgs__srv__Text2Speach_Response__Sequence * array = (qbo_msgs__srv__Text2Speach_Response__Sequence *)allocator.allocate(sizeof(qbo_msgs__srv__Text2Speach_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = qbo_msgs__srv__Text2Speach_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
qbo_msgs__srv__Text2Speach_Response__Sequence__destroy(qbo_msgs__srv__Text2Speach_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    qbo_msgs__srv__Text2Speach_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
qbo_msgs__srv__Text2Speach_Response__Sequence__are_equal(const qbo_msgs__srv__Text2Speach_Response__Sequence * lhs, const qbo_msgs__srv__Text2Speach_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!qbo_msgs__srv__Text2Speach_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
qbo_msgs__srv__Text2Speach_Response__Sequence__copy(
  const qbo_msgs__srv__Text2Speach_Response__Sequence * input,
  qbo_msgs__srv__Text2Speach_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(qbo_msgs__srv__Text2Speach_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    qbo_msgs__srv__Text2Speach_Response * data =
      (qbo_msgs__srv__Text2Speach_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!qbo_msgs__srv__Text2Speach_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          qbo_msgs__srv__Text2Speach_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!qbo_msgs__srv__Text2Speach_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
