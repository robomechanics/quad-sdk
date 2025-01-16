// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from quad_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#include "quad_msgs/msg/detail/motor_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
const rosidl_type_hash_t *
quad_msgs__msg__MotorCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfe, 0x07, 0xa0, 0x7a, 0xff, 0x8d, 0xe2, 0xf4,
      0x25, 0xc0, 0x56, 0xfd, 0x90, 0xee, 0xe8, 0x76,
      0x8b, 0x9d, 0xb1, 0x1e, 0xc9, 0x61, 0xc0, 0x02,
      0x83, 0x45, 0xb5, 0x7b, 0xfd, 0x90, 0x71, 0x18,
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

static char quad_msgs__msg__MotorCommand__TYPE_NAME[] = "quad_msgs/msg/MotorCommand";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char quad_msgs__msg__MotorCommand__FIELD_NAME__header[] = "header";
static char quad_msgs__msg__MotorCommand__FIELD_NAME__pos_setpoint[] = "pos_setpoint";
static char quad_msgs__msg__MotorCommand__FIELD_NAME__vel_setpoint[] = "vel_setpoint";
static char quad_msgs__msg__MotorCommand__FIELD_NAME__kp[] = "kp";
static char quad_msgs__msg__MotorCommand__FIELD_NAME__kd[] = "kd";
static char quad_msgs__msg__MotorCommand__FIELD_NAME__torque_ff[] = "torque_ff";
static char quad_msgs__msg__MotorCommand__FIELD_NAME__pos_component[] = "pos_component";
static char quad_msgs__msg__MotorCommand__FIELD_NAME__vel_component[] = "vel_component";
static char quad_msgs__msg__MotorCommand__FIELD_NAME__fb_component[] = "fb_component";
static char quad_msgs__msg__MotorCommand__FIELD_NAME__effort[] = "effort";
static char quad_msgs__msg__MotorCommand__FIELD_NAME__fb_ratio[] = "fb_ratio";

static rosidl_runtime_c__type_description__Field quad_msgs__msg__MotorCommand__FIELDS[] = {
  {
    {quad_msgs__msg__MotorCommand__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__MotorCommand__FIELD_NAME__pos_setpoint, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__MotorCommand__FIELD_NAME__vel_setpoint, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__MotorCommand__FIELD_NAME__kp, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__MotorCommand__FIELD_NAME__kd, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__MotorCommand__FIELD_NAME__torque_ff, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__MotorCommand__FIELD_NAME__pos_component, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__MotorCommand__FIELD_NAME__vel_component, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__MotorCommand__FIELD_NAME__fb_component, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__MotorCommand__FIELD_NAME__effort, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__MotorCommand__FIELD_NAME__fb_ratio, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription quad_msgs__msg__MotorCommand__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
quad_msgs__msg__MotorCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {quad_msgs__msg__MotorCommand__TYPE_NAME, 26, 26},
      {quad_msgs__msg__MotorCommand__FIELDS, 11, 11},
    },
    {quad_msgs__msg__MotorCommand__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
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
  "# This is a message to hold the desired position, desired velocity, feedforward torques and control gains for a single joint on Quad\n"
  "#\n"
  "# Accurate timing information is stored in the header\n"
  "\n"
  "std_msgs/Header header\n"
  "\n"
  "# Commands\n"
  "float64 pos_setpoint # Position command\n"
  "float64 vel_setpoint # Velocity command\n"
  "float32 kp # Position setpoint gain\n"
  "float32 kd # Derivative setpoint gain\n"
  "float64 torque_ff # Feedforward torque\n"
  "\n"
  "# Diagnostics\n"
  "float64 pos_component # Feedback position component\n"
  "float64 vel_component # Feedback velocity component\n"
  "float64 fb_component # Feedback total component\n"
  "float64 effort # Total effort\n"
  "float64 fb_ratio # Feedback to total ratio";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
quad_msgs__msg__MotorCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {quad_msgs__msg__MotorCommand__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 658, 658},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
quad_msgs__msg__MotorCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *quad_msgs__msg__MotorCommand__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
