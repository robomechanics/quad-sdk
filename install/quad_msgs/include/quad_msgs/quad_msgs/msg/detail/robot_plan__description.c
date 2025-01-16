// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from quad_msgs:msg/RobotPlan.idl
// generated code does not contain a copyright notice

#include "quad_msgs/msg/detail/robot_plan__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
const rosidl_type_hash_t *
quad_msgs__msg__RobotPlan__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa9, 0x35, 0xf8, 0xde, 0x71, 0x8c, 0x63, 0xbf,
      0xe7, 0x36, 0x23, 0xdd, 0xf4, 0xa7, 0x01, 0x0d,
      0x1b, 0xb8, 0x4b, 0xec, 0x56, 0xa6, 0x35, 0xa8,
      0x88, 0x10, 0xd2, 0xaf, 0x0a, 0x68, 0x64, 0x31,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "quad_msgs/msg/detail/multi_foot_state__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "geometry_msgs/msg/detail/point__functions.h"
#include "geometry_msgs/msg/detail/vector3__functions.h"
#include "quad_msgs/msg/detail/robot_state__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "quad_msgs/msg/detail/robot_plan_diagnostics__functions.h"
#include "quad_msgs/msg/detail/foot_state__functions.h"
#include "geometry_msgs/msg/detail/quaternion__functions.h"
#include "geometry_msgs/msg/detail/twist__functions.h"
#include "quad_msgs/msg/detail/grf_array__functions.h"
#include "quad_msgs/msg/detail/body_state__functions.h"
#include "geometry_msgs/msg/detail/pose__functions.h"
#include "sensor_msgs/msg/detail/joint_state__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Pose__EXPECTED_HASH = {1, {
    0xd5, 0x01, 0x95, 0x4e, 0x94, 0x76, 0xce, 0xa2,
    0x99, 0x69, 0x84, 0xe8, 0x12, 0x05, 0x4b, 0x68,
    0x02, 0x6a, 0xe0, 0xbf, 0xae, 0x78, 0x9d, 0x9a,
    0x10, 0xb2, 0x3d, 0xaf, 0x35, 0xcc, 0x90, 0xfa,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Quaternion__EXPECTED_HASH = {1, {
    0x8a, 0x76, 0x5f, 0x66, 0x77, 0x8c, 0x8f, 0xf7,
    0xc8, 0xab, 0x94, 0xaf, 0xcc, 0x59, 0x0a, 0x2e,
    0xd5, 0x32, 0x5a, 0x1d, 0x9a, 0x07, 0x6f, 0xff,
    0xf3, 0x8f, 0xbc, 0xe3, 0x6f, 0x45, 0x86, 0x84,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Twist__EXPECTED_HASH = {1, {
    0x9c, 0x45, 0xbf, 0x16, 0xfe, 0x09, 0x83, 0xd8,
    0x0e, 0x3c, 0xfe, 0x75, 0x0d, 0x68, 0x35, 0x84,
    0x3d, 0x26, 0x5a, 0x9a, 0x6c, 0x46, 0xbd, 0x2e,
    0x60, 0x9f, 0xcd, 0xdd, 0xe6, 0xfb, 0x8d, 0x2a,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
static const rosidl_type_hash_t quad_msgs__msg__BodyState__EXPECTED_HASH = {1, {
    0x7c, 0x8a, 0x06, 0xbd, 0xa9, 0x33, 0x86, 0x50,
    0x57, 0xc6, 0x34, 0x04, 0xec, 0xfd, 0xc9, 0xa7,
    0x2b, 0x7a, 0xfa, 0xd8, 0x86, 0x8e, 0xc4, 0x25,
    0xe8, 0xaa, 0xf8, 0x9f, 0x12, 0xd4, 0x10, 0x94,
  }};
static const rosidl_type_hash_t quad_msgs__msg__FootState__EXPECTED_HASH = {1, {
    0xb3, 0x08, 0x0b, 0x7d, 0x37, 0xba, 0x52, 0x05,
    0x3e, 0x0a, 0xa7, 0x59, 0x4e, 0x16, 0x51, 0xbc,
    0x50, 0x1f, 0xbe, 0x82, 0x03, 0x48, 0x3e, 0xa7,
    0x60, 0x52, 0xb1, 0x2d, 0x9b, 0x08, 0x83, 0xa6,
  }};
static const rosidl_type_hash_t quad_msgs__msg__GRFArray__EXPECTED_HASH = {1, {
    0x34, 0xc3, 0x76, 0xd2, 0xb6, 0xdd, 0xfc, 0x9e,
    0xe4, 0x0f, 0x76, 0xae, 0xd3, 0x5a, 0x29, 0xff,
    0x7a, 0x88, 0x10, 0x34, 0x87, 0x1c, 0xdc, 0x8d,
    0x47, 0x6f, 0x61, 0x29, 0x45, 0x6a, 0x64, 0x7a,
  }};
static const rosidl_type_hash_t quad_msgs__msg__MultiFootState__EXPECTED_HASH = {1, {
    0xe8, 0xd5, 0xfc, 0xed, 0x45, 0x75, 0x77, 0x6a,
    0xfb, 0xcd, 0x54, 0x60, 0x10, 0x43, 0x59, 0xcb,
    0x17, 0xbc, 0x56, 0xaa, 0x05, 0xd0, 0x29, 0xc4,
    0x3d, 0x10, 0xe7, 0xff, 0x0f, 0x82, 0x54, 0x5b,
  }};
static const rosidl_type_hash_t quad_msgs__msg__RobotPlanDiagnostics__EXPECTED_HASH = {1, {
    0xef, 0xf6, 0x97, 0xba, 0x23, 0x6a, 0xed, 0xa1,
    0x1c, 0x7a, 0x7e, 0xef, 0x16, 0x75, 0x08, 0x68,
    0x44, 0xb4, 0x7c, 0xc1, 0x29, 0x24, 0xe5, 0xf3,
    0xf5, 0xc3, 0xc1, 0x8f, 0xc2, 0x69, 0x12, 0xa8,
  }};
static const rosidl_type_hash_t quad_msgs__msg__RobotState__EXPECTED_HASH = {1, {
    0x35, 0x7f, 0xe0, 0x57, 0xd0, 0xe1, 0x51, 0x8b,
    0x96, 0xdd, 0x12, 0x48, 0x2a, 0xd2, 0x62, 0xa3,
    0xd9, 0x4a, 0xe0, 0x38, 0x3a, 0xff, 0xd1, 0xe8,
    0x7f, 0xe7, 0xc3, 0xf6, 0xe8, 0xe5, 0x3a, 0x6a,
  }};
static const rosidl_type_hash_t sensor_msgs__msg__JointState__EXPECTED_HASH = {1, {
    0xa1, 0x3e, 0xe3, 0xa3, 0x30, 0xe3, 0x46, 0xc9,
    0xd8, 0x7b, 0x5a, 0xa1, 0x8d, 0x24, 0xe1, 0x16,
    0x90, 0x75, 0x2b, 0xd3, 0x3a, 0x03, 0x50, 0xf1,
    0x1c, 0x58, 0x82, 0xbc, 0x91, 0x79, 0x26, 0x0e,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char quad_msgs__msg__RobotPlan__TYPE_NAME[] = "quad_msgs/msg/RobotPlan";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__Pose__TYPE_NAME[] = "geometry_msgs/msg/Pose";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";
static char geometry_msgs__msg__Twist__TYPE_NAME[] = "geometry_msgs/msg/Twist";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";
static char quad_msgs__msg__BodyState__TYPE_NAME[] = "quad_msgs/msg/BodyState";
static char quad_msgs__msg__FootState__TYPE_NAME[] = "quad_msgs/msg/FootState";
static char quad_msgs__msg__GRFArray__TYPE_NAME[] = "quad_msgs/msg/GRFArray";
static char quad_msgs__msg__MultiFootState__TYPE_NAME[] = "quad_msgs/msg/MultiFootState";
static char quad_msgs__msg__RobotPlanDiagnostics__TYPE_NAME[] = "quad_msgs/msg/RobotPlanDiagnostics";
static char quad_msgs__msg__RobotState__TYPE_NAME[] = "quad_msgs/msg/RobotState";
static char sensor_msgs__msg__JointState__TYPE_NAME[] = "sensor_msgs/msg/JointState";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char quad_msgs__msg__RobotPlan__FIELD_NAME__header[] = "header";
static char quad_msgs__msg__RobotPlan__FIELD_NAME__global_plan_timestamp[] = "global_plan_timestamp";
static char quad_msgs__msg__RobotPlan__FIELD_NAME__state_timestamp[] = "state_timestamp";
static char quad_msgs__msg__RobotPlan__FIELD_NAME__states[] = "states";
static char quad_msgs__msg__RobotPlan__FIELD_NAME__grfs[] = "grfs";
static char quad_msgs__msg__RobotPlan__FIELD_NAME__plan_indices[] = "plan_indices";
static char quad_msgs__msg__RobotPlan__FIELD_NAME__primitive_ids[] = "primitive_ids";
static char quad_msgs__msg__RobotPlan__FIELD_NAME__compute_time[] = "compute_time";
static char quad_msgs__msg__RobotPlan__FIELD_NAME__diagnostics[] = "diagnostics";

static rosidl_runtime_c__type_description__Field quad_msgs__msg__RobotPlan__FIELDS[] = {
  {
    {quad_msgs__msg__RobotPlan__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__RobotPlan__FIELD_NAME__global_plan_timestamp, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__RobotPlan__FIELD_NAME__state_timestamp, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__RobotPlan__FIELD_NAME__states, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {quad_msgs__msg__RobotState__TYPE_NAME, 24, 24},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__RobotPlan__FIELD_NAME__grfs, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {quad_msgs__msg__GRFArray__TYPE_NAME, 22, 22},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__RobotPlan__FIELD_NAME__plan_indices, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__RobotPlan__FIELD_NAME__primitive_ids, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__RobotPlan__FIELD_NAME__compute_time, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__RobotPlan__FIELD_NAME__diagnostics, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {quad_msgs__msg__RobotPlanDiagnostics__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription quad_msgs__msg__RobotPlan__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Twist__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__BodyState__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__FootState__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__GRFArray__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__MultiFootState__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__RobotPlanDiagnostics__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__RobotState__TYPE_NAME, 24, 24},
    {NULL, 0, 0},
  },
  {
    {sensor_msgs__msg__JointState__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
quad_msgs__msg__RobotPlan__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {quad_msgs__msg__RobotPlan__TYPE_NAME, 23, 23},
      {quad_msgs__msg__RobotPlan__FIELDS, 9, 9},
    },
    {quad_msgs__msg__RobotPlan__REFERENCED_TYPE_DESCRIPTIONS, 14, 14},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose__EXPECTED_HASH, geometry_msgs__msg__Pose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Pose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Twist__EXPECTED_HASH, geometry_msgs__msg__Twist__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = geometry_msgs__msg__Twist__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&quad_msgs__msg__BodyState__EXPECTED_HASH, quad_msgs__msg__BodyState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = quad_msgs__msg__BodyState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&quad_msgs__msg__FootState__EXPECTED_HASH, quad_msgs__msg__FootState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = quad_msgs__msg__FootState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&quad_msgs__msg__GRFArray__EXPECTED_HASH, quad_msgs__msg__GRFArray__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = quad_msgs__msg__GRFArray__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&quad_msgs__msg__MultiFootState__EXPECTED_HASH, quad_msgs__msg__MultiFootState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[9].fields = quad_msgs__msg__MultiFootState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&quad_msgs__msg__RobotPlanDiagnostics__EXPECTED_HASH, quad_msgs__msg__RobotPlanDiagnostics__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[10].fields = quad_msgs__msg__RobotPlanDiagnostics__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&quad_msgs__msg__RobotState__EXPECTED_HASH, quad_msgs__msg__RobotState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[11].fields = quad_msgs__msg__RobotState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sensor_msgs__msg__JointState__EXPECTED_HASH, sensor_msgs__msg__JointState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[12].fields = sensor_msgs__msg__JointState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[13].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This is a message to hold a robot plan\n"
  "#\n"
  "# The plan is defined as an array of odometry messages.\n"
  "# Accurate timing information for localization is stored in the header.\n"
  "# This should match the first state in the states vector.\n"
  "\n"
  "std_msgs/Header header\n"
  "builtin_interfaces/Time global_plan_timestamp\n"
  "builtin_interfaces/Time state_timestamp\n"
  "quad_msgs/RobotState[] states\n"
  "quad_msgs/GRFArray[] grfs\n"
  "uint32[] plan_indices\n"
  "uint32[] primitive_ids\n"
  "float64 compute_time\n"
  "quad_msgs/RobotPlanDiagnostics diagnostics";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
quad_msgs__msg__RobotPlan__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {quad_msgs__msg__RobotPlan__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 504, 504},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
quad_msgs__msg__RobotPlan__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[15];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 15, 15};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *quad_msgs__msg__RobotPlan__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Pose__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__Twist__get_individual_type_description_source(NULL);
    sources[6] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    sources[7] = *quad_msgs__msg__BodyState__get_individual_type_description_source(NULL);
    sources[8] = *quad_msgs__msg__FootState__get_individual_type_description_source(NULL);
    sources[9] = *quad_msgs__msg__GRFArray__get_individual_type_description_source(NULL);
    sources[10] = *quad_msgs__msg__MultiFootState__get_individual_type_description_source(NULL);
    sources[11] = *quad_msgs__msg__RobotPlanDiagnostics__get_individual_type_description_source(NULL);
    sources[12] = *quad_msgs__msg__RobotState__get_individual_type_description_source(NULL);
    sources[13] = *sensor_msgs__msg__JointState__get_individual_type_description_source(NULL);
    sources[14] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
