// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from dynamixel_sdk_custom_interfaces:msg/SetPosition.idl
// generated code does not contain a copyright notice

#include "dynamixel_sdk_custom_interfaces/msg/detail/set_position__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_dynamixel_sdk_custom_interfaces
const rosidl_type_hash_t *
dynamixel_sdk_custom_interfaces__msg__SetPosition__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x21, 0xa0, 0x75, 0xf9, 0x07, 0x9e, 0x90, 0x60,
      0x4d, 0xc8, 0x5d, 0x51, 0x22, 0xcc, 0x97, 0xd0,
      0x1f, 0x01, 0xd3, 0x88, 0x5f, 0xbd, 0x95, 0x69,
      0x28, 0xf6, 0x53, 0xb4, 0x06, 0x44, 0x43, 0xea,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char dynamixel_sdk_custom_interfaces__msg__SetPosition__TYPE_NAME[] = "dynamixel_sdk_custom_interfaces/msg/SetPosition";

// Define type names, field names, and default values
static char dynamixel_sdk_custom_interfaces__msg__SetPosition__FIELD_NAME__id[] = "id";
static char dynamixel_sdk_custom_interfaces__msg__SetPosition__FIELD_NAME__position[] = "position";

static rosidl_runtime_c__type_description__Field dynamixel_sdk_custom_interfaces__msg__SetPosition__FIELDS[] = {
  {
    {dynamixel_sdk_custom_interfaces__msg__SetPosition__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dynamixel_sdk_custom_interfaces__msg__SetPosition__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dynamixel_sdk_custom_interfaces__msg__SetPosition__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dynamixel_sdk_custom_interfaces__msg__SetPosition__TYPE_NAME, 47, 47},
      {dynamixel_sdk_custom_interfaces__msg__SetPosition__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Messages\n"
  "uint8 id\n"
  "int32 position";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
dynamixel_sdk_custom_interfaces__msg__SetPosition__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dynamixel_sdk_custom_interfaces__msg__SetPosition__TYPE_NAME, 47, 47},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 34, 34},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dynamixel_sdk_custom_interfaces__msg__SetPosition__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dynamixel_sdk_custom_interfaces__msg__SetPosition__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
