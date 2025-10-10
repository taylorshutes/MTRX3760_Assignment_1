// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from dynamixel_sdk_custom_interfaces:msg/SetPosition.idl
// generated code does not contain a copyright notice
#include "dynamixel_sdk_custom_interfaces/msg/detail/set_position__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "dynamixel_sdk_custom_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dynamixel_sdk_custom_interfaces/msg/detail/set_position__struct.h"
#include "dynamixel_sdk_custom_interfaces/msg/detail/set_position__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _SetPosition__ros_msg_type = dynamixel_sdk_custom_interfaces__msg__SetPosition;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dynamixel_sdk_custom_interfaces
bool cdr_serialize_dynamixel_sdk_custom_interfaces__msg__SetPosition(
  const dynamixel_sdk_custom_interfaces__msg__SetPosition * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: position
  {
    cdr << ros_message->position;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dynamixel_sdk_custom_interfaces
bool cdr_deserialize_dynamixel_sdk_custom_interfaces__msg__SetPosition(
  eprosima::fastcdr::Cdr & cdr,
  dynamixel_sdk_custom_interfaces__msg__SetPosition * ros_message)
{
  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: position
  {
    cdr >> ros_message->position;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dynamixel_sdk_custom_interfaces
size_t get_serialized_size_dynamixel_sdk_custom_interfaces__msg__SetPosition(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetPosition__ros_msg_type * ros_message = static_cast<const _SetPosition__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: position
  {
    size_t item_size = sizeof(ros_message->position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dynamixel_sdk_custom_interfaces
size_t max_serialized_size_dynamixel_sdk_custom_interfaces__msg__SetPosition(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: id
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: position
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dynamixel_sdk_custom_interfaces__msg__SetPosition;
    is_plain =
      (
      offsetof(DataType, position) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dynamixel_sdk_custom_interfaces
bool cdr_serialize_key_dynamixel_sdk_custom_interfaces__msg__SetPosition(
  const dynamixel_sdk_custom_interfaces__msg__SetPosition * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: position
  {
    cdr << ros_message->position;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dynamixel_sdk_custom_interfaces
size_t get_serialized_size_key_dynamixel_sdk_custom_interfaces__msg__SetPosition(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetPosition__ros_msg_type * ros_message = static_cast<const _SetPosition__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: position
  {
    size_t item_size = sizeof(ros_message->position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dynamixel_sdk_custom_interfaces
size_t max_serialized_size_key_dynamixel_sdk_custom_interfaces__msg__SetPosition(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: id
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: position
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dynamixel_sdk_custom_interfaces__msg__SetPosition;
    is_plain =
      (
      offsetof(DataType, position) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _SetPosition__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const dynamixel_sdk_custom_interfaces__msg__SetPosition * ros_message = static_cast<const dynamixel_sdk_custom_interfaces__msg__SetPosition *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_dynamixel_sdk_custom_interfaces__msg__SetPosition(ros_message, cdr);
}

static bool _SetPosition__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  dynamixel_sdk_custom_interfaces__msg__SetPosition * ros_message = static_cast<dynamixel_sdk_custom_interfaces__msg__SetPosition *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_dynamixel_sdk_custom_interfaces__msg__SetPosition(cdr, ros_message);
}

static uint32_t _SetPosition__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dynamixel_sdk_custom_interfaces__msg__SetPosition(
      untyped_ros_message, 0));
}

static size_t _SetPosition__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dynamixel_sdk_custom_interfaces__msg__SetPosition(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SetPosition = {
  "dynamixel_sdk_custom_interfaces::msg",
  "SetPosition",
  _SetPosition__cdr_serialize,
  _SetPosition__cdr_deserialize,
  _SetPosition__get_serialized_size,
  _SetPosition__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _SetPosition__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SetPosition,
  get_message_typesupport_handle_function,
  &dynamixel_sdk_custom_interfaces__msg__SetPosition__get_type_hash,
  &dynamixel_sdk_custom_interfaces__msg__SetPosition__get_type_description,
  &dynamixel_sdk_custom_interfaces__msg__SetPosition__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dynamixel_sdk_custom_interfaces, msg, SetPosition)() {
  return &_SetPosition__type_support;
}

#if defined(__cplusplus)
}
#endif
