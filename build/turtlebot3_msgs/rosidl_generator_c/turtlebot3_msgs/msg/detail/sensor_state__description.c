// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from turtlebot3_msgs:msg/SensorState.idl
// generated code does not contain a copyright notice

#include "turtlebot3_msgs/msg/detail/sensor_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_type_hash_t *
turtlebot3_msgs__msg__SensorState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe3, 0x5b, 0xc7, 0xdb, 0x55, 0xe7, 0xa7, 0xae,
      0xb2, 0x49, 0x90, 0xb6, 0xd7, 0xf1, 0x89, 0xa2,
      0x00, 0x2a, 0xe4, 0x42, 0xe0, 0xd9, 0x92, 0x50,
      0x77, 0xe2, 0x98, 0x95, 0xf8, 0x38, 0xcc, 0xea,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char turtlebot3_msgs__msg__SensorState__TYPE_NAME[] = "turtlebot3_msgs/msg/SensorState";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char turtlebot3_msgs__msg__SensorState__FIELD_NAME__header[] = "header";
static char turtlebot3_msgs__msg__SensorState__FIELD_NAME__bumper[] = "bumper";
static char turtlebot3_msgs__msg__SensorState__FIELD_NAME__cliff[] = "cliff";
static char turtlebot3_msgs__msg__SensorState__FIELD_NAME__sonar[] = "sonar";
static char turtlebot3_msgs__msg__SensorState__FIELD_NAME__illumination[] = "illumination";
static char turtlebot3_msgs__msg__SensorState__FIELD_NAME__led[] = "led";
static char turtlebot3_msgs__msg__SensorState__FIELD_NAME__button[] = "button";
static char turtlebot3_msgs__msg__SensorState__FIELD_NAME__torque[] = "torque";
static char turtlebot3_msgs__msg__SensorState__FIELD_NAME__left_encoder[] = "left_encoder";
static char turtlebot3_msgs__msg__SensorState__FIELD_NAME__right_encoder[] = "right_encoder";
static char turtlebot3_msgs__msg__SensorState__FIELD_NAME__battery[] = "battery";

static rosidl_runtime_c__type_description__Field turtlebot3_msgs__msg__SensorState__FIELDS[] = {
  {
    {turtlebot3_msgs__msg__SensorState__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__msg__SensorState__FIELD_NAME__bumper, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__msg__SensorState__FIELD_NAME__cliff, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__msg__SensorState__FIELD_NAME__sonar, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__msg__SensorState__FIELD_NAME__illumination, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__msg__SensorState__FIELD_NAME__led, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__msg__SensorState__FIELD_NAME__button, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__msg__SensorState__FIELD_NAME__torque, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__msg__SensorState__FIELD_NAME__left_encoder, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__msg__SensorState__FIELD_NAME__right_encoder, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__msg__SensorState__FIELD_NAME__battery, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription turtlebot3_msgs__msg__SensorState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
turtlebot3_msgs__msg__SensorState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {turtlebot3_msgs__msg__SensorState__TYPE_NAME, 31, 31},
      {turtlebot3_msgs__msg__SensorState__FIELDS, 11, 11},
    },
    {turtlebot3_msgs__msg__SensorState__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "########################################\n"
  "# CONSTANTS\n"
  "########################################\n"
  "# Bumper states (states are combined, when multiple bumpers are pressed)\n"
  "uint8 BUMPER_FORWARD  = 1\n"
  "uint8 BUMPER_BACKWARD = 2\n"
  "\n"
  "# Cliff sensor states (states are combined, when multiple cliff sensors are triggered)\n"
  "uint8 CLIFF = 1\n"
  "\n"
  "# Sonar sensor states (states are combined, when multiple sonar sensors are triggered)\n"
  "uint8 SONAR = 1\n"
  "\n"
  "# Illumination sensor (states are combined, when multiple illumination sensors are triggered) \n"
  "uint8 ILLUMINATION = 1\n"
  "\n"
  "# Button states (states are combined, when multiple buttons are pressed)\n"
  "uint8 BUTTON0 = 1\n"
  "uint8 BUTTON1 = 2\n"
  "\n"
  "# Motor errors\n"
  "uint8 ERROR_LEFT_MOTOR  = 1\n"
  "uint8 ERROR_RIGHT_MOTOR = 2\n"
  "\n"
  "# Motor torque\n"
  "uint8 TORQUE_ON  = 1\n"
  "uint8 TORQUE_OFF = 2\n"
  "\n"
  "########################################\n"
  "# Messages\n"
  "########################################\n"
  "std_msgs/Header  header\n"
  "uint8   bumper\n"
  "float32 cliff\n"
  "float32 sonar\n"
  "float32 illumination\n"
  "uint8   led\n"
  "uint8  button\n"
  "bool   torque\n"
  "int32  left_encoder    # (-2,147,483,648 ~ 2,147,483,647)\n"
  "int32  right_encoder   # (-2,147,483,648 ~ 2,147,483,647)\n"
  "float32  battery";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
turtlebot3_msgs__msg__SensorState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {turtlebot3_msgs__msg__SensorState__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1141, 1141},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
turtlebot3_msgs__msg__SensorState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *turtlebot3_msgs__msg__SensorState__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
