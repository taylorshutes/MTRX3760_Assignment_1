// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot3_msgs:srv/Goal.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "turtlebot3_msgs/srv/goal.h"


#ifndef TURTLEBOT3_MSGS__SRV__DETAIL__GOAL__STRUCT_H_
#define TURTLEBOT3_MSGS__SRV__DETAIL__GOAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Goal in the package turtlebot3_msgs.
typedef struct turtlebot3_msgs__srv__Goal_Request
{
  uint8_t structure_needs_at_least_one_member;
} turtlebot3_msgs__srv__Goal_Request;

// Struct for a sequence of turtlebot3_msgs__srv__Goal_Request.
typedef struct turtlebot3_msgs__srv__Goal_Request__Sequence
{
  turtlebot3_msgs__srv__Goal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_msgs__srv__Goal_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/Goal in the package turtlebot3_msgs.
typedef struct turtlebot3_msgs__srv__Goal_Response
{
  float pose_x;
  float pose_y;
  bool success;
} turtlebot3_msgs__srv__Goal_Response;

// Struct for a sequence of turtlebot3_msgs__srv__Goal_Response.
typedef struct turtlebot3_msgs__srv__Goal_Response__Sequence
{
  turtlebot3_msgs__srv__Goal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_msgs__srv__Goal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  turtlebot3_msgs__srv__Goal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  turtlebot3_msgs__srv__Goal_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Goal in the package turtlebot3_msgs.
typedef struct turtlebot3_msgs__srv__Goal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  turtlebot3_msgs__srv__Goal_Request__Sequence request;
  turtlebot3_msgs__srv__Goal_Response__Sequence response;
} turtlebot3_msgs__srv__Goal_Event;

// Struct for a sequence of turtlebot3_msgs__srv__Goal_Event.
typedef struct turtlebot3_msgs__srv__Goal_Event__Sequence
{
  turtlebot3_msgs__srv__Goal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_msgs__srv__Goal_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_MSGS__SRV__DETAIL__GOAL__STRUCT_H_
