// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dynamixel_sdk_custom_interfaces:srv/GetPosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dynamixel_sdk_custom_interfaces/srv/get_position.h"


#ifndef DYNAMIXEL_SDK_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__STRUCT_H_
#define DYNAMIXEL_SDK_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetPosition in the package dynamixel_sdk_custom_interfaces.
typedef struct dynamixel_sdk_custom_interfaces__srv__GetPosition_Request
{
  uint8_t id;
} dynamixel_sdk_custom_interfaces__srv__GetPosition_Request;

// Struct for a sequence of dynamixel_sdk_custom_interfaces__srv__GetPosition_Request.
typedef struct dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/GetPosition in the package dynamixel_sdk_custom_interfaces.
typedef struct dynamixel_sdk_custom_interfaces__srv__GetPosition_Response
{
  int32_t position;
} dynamixel_sdk_custom_interfaces__srv__GetPosition_Response;

// Struct for a sequence of dynamixel_sdk_custom_interfaces__srv__GetPosition_Response.
typedef struct dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__request__MAX_SIZE = 1
};
// response
enum
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetPosition in the package dynamixel_sdk_custom_interfaces.
typedef struct dynamixel_sdk_custom_interfaces__srv__GetPosition_Event
{
  service_msgs__msg__ServiceEventInfo info;
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence request;
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence response;
} dynamixel_sdk_custom_interfaces__srv__GetPosition_Event;

// Struct for a sequence of dynamixel_sdk_custom_interfaces__srv__GetPosition_Event.
typedef struct dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__Sequence
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIXEL_SDK_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__STRUCT_H_
