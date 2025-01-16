// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from quad_msgs:msg/MultiFootPlanContinuous.idl
// generated code does not contain a copyright notice

#include "quad_msgs/msg/detail/multi_foot_plan_continuous__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
const rosidl_type_hash_t *
quad_msgs__msg__MultiFootPlanContinuous__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x60, 0x29, 0x7a, 0x8e, 0x26, 0x5f, 0xaa, 0xbe,
      0x37, 0x40, 0x3f, 0xec, 0xa5, 0xcd, 0x71, 0x4c,
      0xce, 0xb1, 0x67, 0x22, 0x92, 0xc4, 0xc7, 0xb3,
      0x35, 0x76, 0x8b, 0xb2, 0x6b, 0x37, 0xff, 0xf1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "quad_msgs/msg/detail/multi_foot_state__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "geometry_msgs/msg/detail/vector3__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "quad_msgs/msg/detail/foot_state__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
static const rosidl_type_hash_t quad_msgs__msg__FootState__EXPECTED_HASH = {1, {
    0xb3, 0x08, 0x0b, 0x7d, 0x37, 0xba, 0x52, 0x05,
    0x3e, 0x0a, 0xa7, 0x59, 0x4e, 0x16, 0x51, 0xbc,
    0x50, 0x1f, 0xbe, 0x82, 0x03, 0x48, 0x3e, 0xa7,
    0x60, 0x52, 0xb1, 0x2d, 0x9b, 0x08, 0x83, 0xa6,
  }};
static const rosidl_type_hash_t quad_msgs__msg__MultiFootState__EXPECTED_HASH = {1, {
    0xe8, 0xd5, 0xfc, 0xed, 0x45, 0x75, 0x77, 0x6a,
    0xfb, 0xcd, 0x54, 0x60, 0x10, 0x43, 0x59, 0xcb,
    0x17, 0xbc, 0x56, 0xaa, 0x05, 0xd0, 0x29, 0xc4,
    0x3d, 0x10, 0xe7, 0xff, 0x0f, 0x82, 0x54, 0x5b,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char quad_msgs__msg__MultiFootPlanContinuous__TYPE_NAME[] = "quad_msgs/msg/MultiFootPlanContinuous";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";
static char quad_msgs__msg__FootState__TYPE_NAME[] = "quad_msgs/msg/FootState";
static char quad_msgs__msg__MultiFootState__TYPE_NAME[] = "quad_msgs/msg/MultiFootState";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char quad_msgs__msg__MultiFootPlanContinuous__FIELD_NAME__header[] = "header";
static char quad_msgs__msg__MultiFootPlanContinuous__FIELD_NAME__states[] = "states";

static rosidl_runtime_c__type_description__Field quad_msgs__msg__MultiFootPlanContinuous__FIELDS[] = {
  {
    {quad_msgs__msg__MultiFootPlanContinuous__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__MultiFootPlanContinuous__FIELD_NAME__states, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {quad_msgs__msg__MultiFootState__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription quad_msgs__msg__MultiFootPlanContinuous__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__FootState__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__MultiFootState__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
quad_msgs__msg__MultiFootPlanContinuous__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {quad_msgs__msg__MultiFootPlanContinuous__TYPE_NAME, 37, 37},
      {quad_msgs__msg__MultiFootPlanContinuous__FIELDS, 2, 2},
    },
    {quad_msgs__msg__MultiFootPlanContinuous__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&quad_msgs__msg__FootState__EXPECTED_HASH, quad_msgs__msg__FootState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = quad_msgs__msg__FootState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&quad_msgs__msg__MultiFootState__EXPECTED_HASH, quad_msgs__msg__MultiFootState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = quad_msgs__msg__MultiFootState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This is a message to hold a continuous foot plan for multiple robot feet\n"
  "#\n"
  "# The plan is defined as a vector of MultiFootState messages\n"
  "# Accurate timing information to localize the plans is stored in the header\n"
  "\n"
  "std_msgs/Header header\n"
  "quad_msgs/MultiFootState[] states";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
quad_msgs__msg__MultiFootPlanContinuous__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {quad_msgs__msg__MultiFootPlanContinuous__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 271, 271},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
quad_msgs__msg__MultiFootPlanContinuous__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *quad_msgs__msg__MultiFootPlanContinuous__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    sources[3] = *quad_msgs__msg__FootState__get_individual_type_description_source(NULL);
    sources[4] = *quad_msgs__msg__MultiFootState__get_individual_type_description_source(NULL);
    sources[5] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
