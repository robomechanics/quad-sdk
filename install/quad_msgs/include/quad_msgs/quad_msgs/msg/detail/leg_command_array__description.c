// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from quad_msgs:msg/LegCommandArray.idl
// generated code does not contain a copyright notice

#include "quad_msgs/msg/detail/leg_command_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
const rosidl_type_hash_t *
quad_msgs__msg__LegCommandArray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8f, 0x70, 0x56, 0x5d, 0x79, 0x3a, 0x07, 0x55,
      0x3e, 0x4a, 0x8c, 0x8d, 0x14, 0x4c, 0xab, 0x95,
      0x41, 0x66, 0x9c, 0xbd, 0xf8, 0x80, 0xa9, 0xf0,
      0xe5, 0x88, 0x81, 0x81, 0xc7, 0x01, 0xd9, 0xe0,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "quad_msgs/msg/detail/motor_command__functions.h"
#include "quad_msgs/msg/detail/leg_command__functions.h"
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
static const rosidl_type_hash_t quad_msgs__msg__LegCommand__EXPECTED_HASH = {1, {
    0x1c, 0xcd, 0xad, 0x51, 0x62, 0x5e, 0xb7, 0x2c,
    0xb9, 0xb2, 0x00, 0x8c, 0x16, 0xeb, 0x25, 0xf6,
    0xcb, 0xb4, 0x02, 0x7b, 0x19, 0xbb, 0xe6, 0xab,
    0x94, 0x88, 0xc0, 0xe6, 0x31, 0x76, 0x4c, 0x38,
  }};
static const rosidl_type_hash_t quad_msgs__msg__MotorCommand__EXPECTED_HASH = {1, {
    0xfe, 0x07, 0xa0, 0x7a, 0xff, 0x8d, 0xe2, 0xf4,
    0x25, 0xc0, 0x56, 0xfd, 0x90, 0xee, 0xe8, 0x76,
    0x8b, 0x9d, 0xb1, 0x1e, 0xc9, 0x61, 0xc0, 0x02,
    0x83, 0x45, 0xb5, 0x7b, 0xfd, 0x90, 0x71, 0x18,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char quad_msgs__msg__LegCommandArray__TYPE_NAME[] = "quad_msgs/msg/LegCommandArray";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char quad_msgs__msg__LegCommand__TYPE_NAME[] = "quad_msgs/msg/LegCommand";
static char quad_msgs__msg__MotorCommand__TYPE_NAME[] = "quad_msgs/msg/MotorCommand";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char quad_msgs__msg__LegCommandArray__FIELD_NAME__header[] = "header";
static char quad_msgs__msg__LegCommandArray__FIELD_NAME__leg_commands[] = "leg_commands";

static rosidl_runtime_c__type_description__Field quad_msgs__msg__LegCommandArray__FIELDS[] = {
  {
    {quad_msgs__msg__LegCommandArray__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__LegCommandArray__FIELD_NAME__leg_commands, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {quad_msgs__msg__LegCommand__TYPE_NAME, 24, 24},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription quad_msgs__msg__LegCommandArray__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__LegCommand__TYPE_NAME, 24, 24},
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__MotorCommand__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
quad_msgs__msg__LegCommandArray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {quad_msgs__msg__LegCommandArray__TYPE_NAME, 29, 29},
      {quad_msgs__msg__LegCommandArray__FIELDS, 2, 2},
    },
    {quad_msgs__msg__LegCommandArray__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&quad_msgs__msg__LegCommand__EXPECTED_HASH, quad_msgs__msg__LegCommand__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = quad_msgs__msg__LegCommand__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&quad_msgs__msg__MotorCommand__EXPECTED_HASH, quad_msgs__msg__MotorCommand__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = quad_msgs__msg__MotorCommand__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This is a message of leg commands for each leg on quad. The order is in FL, BL, FR, BR.\n"
  "#\n"
  "# Accurate timing information is stored in the header\n"
  "\n"
  "std_msgs/Header header\n"
  "\n"
  "LegCommand[] leg_commands # FL, BL, FR, BR";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
quad_msgs__msg__LegCommandArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {quad_msgs__msg__LegCommandArray__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 214, 214},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
quad_msgs__msg__LegCommandArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *quad_msgs__msg__LegCommandArray__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *quad_msgs__msg__LegCommand__get_individual_type_description_source(NULL);
    sources[3] = *quad_msgs__msg__MotorCommand__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
