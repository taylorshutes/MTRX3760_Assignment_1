// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from turtlebot3_msgs:msg/VersionInfo.idl
// generated code does not contain a copyright notice

#include "turtlebot3_msgs/msg/detail/version_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_type_hash_t *
turtlebot3_msgs__msg__VersionInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x69, 0xb2, 0x7e, 0x85, 0xc3, 0x8e, 0x43, 0x02,
      0x99, 0x66, 0x9c, 0x12, 0xcd, 0x2e, 0xa4, 0x65,
      0x22, 0xf8, 0x00, 0x18, 0x56, 0x0d, 0x5b, 0xae,
      0xc0, 0x6c, 0xd8, 0x9b, 0x8c, 0x03, 0x77, 0xd8,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char turtlebot3_msgs__msg__VersionInfo__TYPE_NAME[] = "turtlebot3_msgs/msg/VersionInfo";

// Define type names, field names, and default values
static char turtlebot3_msgs__msg__VersionInfo__FIELD_NAME__hardware[] = "hardware";
static char turtlebot3_msgs__msg__VersionInfo__FIELD_NAME__firmware[] = "firmware";
static char turtlebot3_msgs__msg__VersionInfo__FIELD_NAME__software[] = "software";

static rosidl_runtime_c__type_description__Field turtlebot3_msgs__msg__VersionInfo__FIELDS[] = {
  {
    {turtlebot3_msgs__msg__VersionInfo__FIELD_NAME__hardware, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__msg__VersionInfo__FIELD_NAME__firmware, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__msg__VersionInfo__FIELD_NAME__software, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
turtlebot3_msgs__msg__VersionInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {turtlebot3_msgs__msg__VersionInfo__TYPE_NAME, 31, 31},
      {turtlebot3_msgs__msg__VersionInfo__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "########################################\n"
  "# Messages\n"
  "########################################\n"
  "string hardware   # <yyyy>.<mm>.<dd>        : hardware version of Turtlebot3 (ex. 2017.05.23)\n"
  "string firmware   # <major>.<minor>.<patch> : firmware version of OpenCR\n"
  "string software   # <major>.<minor>.<patch> : software version of Turtlebot3 ROS packages";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
turtlebot3_msgs__msg__VersionInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {turtlebot3_msgs__msg__VersionInfo__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 350, 350},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
turtlebot3_msgs__msg__VersionInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *turtlebot3_msgs__msg__VersionInfo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
