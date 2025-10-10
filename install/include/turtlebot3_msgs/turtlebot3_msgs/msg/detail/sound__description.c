// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from turtlebot3_msgs:msg/Sound.idl
// generated code does not contain a copyright notice

#include "turtlebot3_msgs/msg/detail/sound__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_type_hash_t *
turtlebot3_msgs__msg__Sound__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9a, 0xbe, 0x7c, 0x25, 0x5c, 0x34, 0xfd, 0x80,
      0xa4, 0xa7, 0xa5, 0x7a, 0xa0, 0x03, 0x4a, 0x8d,
      0xc3, 0xe4, 0x15, 0xe0, 0xed, 0xff, 0x28, 0x27,
      0xcb, 0x23, 0x64, 0xb4, 0x45, 0xf9, 0x81, 0x29,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char turtlebot3_msgs__msg__Sound__TYPE_NAME[] = "turtlebot3_msgs/msg/Sound";

// Define type names, field names, and default values
static char turtlebot3_msgs__msg__Sound__FIELD_NAME__value[] = "value";

static rosidl_runtime_c__type_description__Field turtlebot3_msgs__msg__Sound__FIELDS[] = {
  {
    {turtlebot3_msgs__msg__Sound__FIELD_NAME__value, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
turtlebot3_msgs__msg__Sound__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {turtlebot3_msgs__msg__Sound__TYPE_NAME, 25, 25},
      {turtlebot3_msgs__msg__Sound__FIELDS, 1, 1},
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
  "# CONSTANTS\n"
  "########################################\n"
  "uint8 OFF           = 0\n"
  "uint8 ON            = 1\n"
  "uint8 LOW_BATTERY   = 2\n"
  "uint8 ERROR         = 3\n"
  "uint8 BUTTON1       = 4\n"
  "uint8 BUTTON2       = 5\n"
  "\n"
  "########################################\n"
  "# Messages\n"
  "########################################\n"
  "uint8 value";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
turtlebot3_msgs__msg__Sound__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {turtlebot3_msgs__msg__Sound__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 344, 344},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
turtlebot3_msgs__msg__Sound__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *turtlebot3_msgs__msg__Sound__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
