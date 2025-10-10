// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from turtlebot3_msgs:srv/Dqn.idl
// generated code does not contain a copyright notice

#include "turtlebot3_msgs/srv/detail/dqn__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_type_hash_t *
turtlebot3_msgs__srv__Dqn__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7b, 0xf4, 0x47, 0x71, 0x19, 0x5b, 0xc4, 0xfc,
      0x42, 0xa8, 0x4c, 0xf1, 0xb4, 0x3c, 0x94, 0xa0,
      0x6e, 0x5e, 0xc4, 0x7e, 0x60, 0x7b, 0x26, 0xf3,
      0x72, 0x32, 0x87, 0x6e, 0xfe, 0x4a, 0xe6, 0x74,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_type_hash_t *
turtlebot3_msgs__srv__Dqn_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x02, 0x07, 0x4d, 0x05, 0xaa, 0xcd, 0xc8, 0x38,
      0xc1, 0x35, 0x15, 0x98, 0x1a, 0x43, 0x3a, 0x3b,
      0x7b, 0xc2, 0x16, 0xf1, 0x43, 0xb4, 0xad, 0xef,
      0xba, 0xba, 0xd6, 0x33, 0xcf, 0x26, 0x79, 0xed,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_type_hash_t *
turtlebot3_msgs__srv__Dqn_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe4, 0xb8, 0x3d, 0x08, 0x84, 0xf4, 0x16, 0x4c,
      0xf1, 0x78, 0xb3, 0xe8, 0x55, 0xbf, 0xa0, 0x76,
      0x79, 0x5c, 0x30, 0xd6, 0x7a, 0xdf, 0xfb, 0x0b,
      0x4a, 0x44, 0x97, 0x51, 0x43, 0x4d, 0x51, 0x30,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_type_hash_t *
turtlebot3_msgs__srv__Dqn_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x55, 0x5b, 0xa8, 0x3f, 0xf8, 0xc8, 0xea, 0x99,
      0x11, 0xe6, 0xda, 0x6f, 0x2a, 0xe1, 0x36, 0x38,
      0x6e, 0x55, 0x39, 0x5d, 0x6b, 0xeb, 0x95, 0xe5,
      0xb7, 0xaf, 0x67, 0xf3, 0x64, 0x2b, 0xb9, 0xf9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char turtlebot3_msgs__srv__Dqn__TYPE_NAME[] = "turtlebot3_msgs/srv/Dqn";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char turtlebot3_msgs__srv__Dqn_Event__TYPE_NAME[] = "turtlebot3_msgs/srv/Dqn_Event";
static char turtlebot3_msgs__srv__Dqn_Request__TYPE_NAME[] = "turtlebot3_msgs/srv/Dqn_Request";
static char turtlebot3_msgs__srv__Dqn_Response__TYPE_NAME[] = "turtlebot3_msgs/srv/Dqn_Response";

// Define type names, field names, and default values
static char turtlebot3_msgs__srv__Dqn__FIELD_NAME__request_message[] = "request_message";
static char turtlebot3_msgs__srv__Dqn__FIELD_NAME__response_message[] = "response_message";
static char turtlebot3_msgs__srv__Dqn__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field turtlebot3_msgs__srv__Dqn__FIELDS[] = {
  {
    {turtlebot3_msgs__srv__Dqn__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {turtlebot3_msgs__srv__Dqn_Request__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__srv__Dqn__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {turtlebot3_msgs__srv__Dqn_Response__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__srv__Dqn__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {turtlebot3_msgs__srv__Dqn_Event__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription turtlebot3_msgs__srv__Dqn__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__srv__Dqn_Event__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__srv__Dqn_Request__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__srv__Dqn_Response__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
turtlebot3_msgs__srv__Dqn__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {turtlebot3_msgs__srv__Dqn__TYPE_NAME, 23, 23},
      {turtlebot3_msgs__srv__Dqn__FIELDS, 3, 3},
    },
    {turtlebot3_msgs__srv__Dqn__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = turtlebot3_msgs__srv__Dqn_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = turtlebot3_msgs__srv__Dqn_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = turtlebot3_msgs__srv__Dqn_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char turtlebot3_msgs__srv__Dqn_Request__FIELD_NAME__action[] = "action";
static char turtlebot3_msgs__srv__Dqn_Request__FIELD_NAME__init[] = "init";

static rosidl_runtime_c__type_description__Field turtlebot3_msgs__srv__Dqn_Request__FIELDS[] = {
  {
    {turtlebot3_msgs__srv__Dqn_Request__FIELD_NAME__action, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__srv__Dqn_Request__FIELD_NAME__init, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
turtlebot3_msgs__srv__Dqn_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {turtlebot3_msgs__srv__Dqn_Request__TYPE_NAME, 31, 31},
      {turtlebot3_msgs__srv__Dqn_Request__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char turtlebot3_msgs__srv__Dqn_Response__FIELD_NAME__state[] = "state";
static char turtlebot3_msgs__srv__Dqn_Response__FIELD_NAME__reward[] = "reward";
static char turtlebot3_msgs__srv__Dqn_Response__FIELD_NAME__done[] = "done";

static rosidl_runtime_c__type_description__Field turtlebot3_msgs__srv__Dqn_Response__FIELDS[] = {
  {
    {turtlebot3_msgs__srv__Dqn_Response__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__srv__Dqn_Response__FIELD_NAME__reward, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__srv__Dqn_Response__FIELD_NAME__done, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
turtlebot3_msgs__srv__Dqn_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {turtlebot3_msgs__srv__Dqn_Response__TYPE_NAME, 32, 32},
      {turtlebot3_msgs__srv__Dqn_Response__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char turtlebot3_msgs__srv__Dqn_Event__FIELD_NAME__info[] = "info";
static char turtlebot3_msgs__srv__Dqn_Event__FIELD_NAME__request[] = "request";
static char turtlebot3_msgs__srv__Dqn_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field turtlebot3_msgs__srv__Dqn_Event__FIELDS[] = {
  {
    {turtlebot3_msgs__srv__Dqn_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__srv__Dqn_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {turtlebot3_msgs__srv__Dqn_Request__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__srv__Dqn_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {turtlebot3_msgs__srv__Dqn_Response__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription turtlebot3_msgs__srv__Dqn_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__srv__Dqn_Request__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {turtlebot3_msgs__srv__Dqn_Response__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
turtlebot3_msgs__srv__Dqn_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {turtlebot3_msgs__srv__Dqn_Event__TYPE_NAME, 29, 29},
      {turtlebot3_msgs__srv__Dqn_Event__FIELDS, 3, 3},
    },
    {turtlebot3_msgs__srv__Dqn_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = turtlebot3_msgs__srv__Dqn_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = turtlebot3_msgs__srv__Dqn_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 action\n"
  "bool init\n"
  "---\n"
  "float32[] state\n"
  "float32 reward\n"
  "bool done";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
turtlebot3_msgs__srv__Dqn__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {turtlebot3_msgs__srv__Dqn__TYPE_NAME, 23, 23},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 68, 68},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
turtlebot3_msgs__srv__Dqn_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {turtlebot3_msgs__srv__Dqn_Request__TYPE_NAME, 31, 31},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
turtlebot3_msgs__srv__Dqn_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {turtlebot3_msgs__srv__Dqn_Response__TYPE_NAME, 32, 32},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
turtlebot3_msgs__srv__Dqn_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {turtlebot3_msgs__srv__Dqn_Event__TYPE_NAME, 29, 29},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
turtlebot3_msgs__srv__Dqn__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *turtlebot3_msgs__srv__Dqn__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *turtlebot3_msgs__srv__Dqn_Event__get_individual_type_description_source(NULL);
    sources[4] = *turtlebot3_msgs__srv__Dqn_Request__get_individual_type_description_source(NULL);
    sources[5] = *turtlebot3_msgs__srv__Dqn_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
turtlebot3_msgs__srv__Dqn_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *turtlebot3_msgs__srv__Dqn_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
turtlebot3_msgs__srv__Dqn_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *turtlebot3_msgs__srv__Dqn_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
turtlebot3_msgs__srv__Dqn_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *turtlebot3_msgs__srv__Dqn_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *turtlebot3_msgs__srv__Dqn_Request__get_individual_type_description_source(NULL);
    sources[4] = *turtlebot3_msgs__srv__Dqn_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
