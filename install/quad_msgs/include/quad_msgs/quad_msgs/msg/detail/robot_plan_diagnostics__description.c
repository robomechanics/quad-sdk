// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from quad_msgs:msg/RobotPlanDiagnostics.idl
// generated code does not contain a copyright notice

#include "quad_msgs/msg/detail/robot_plan_diagnostics__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
const rosidl_type_hash_t *
quad_msgs__msg__RobotPlanDiagnostics__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xef, 0xf6, 0x97, 0xba, 0x23, 0x6a, 0xed, 0xa1,
      0x1c, 0x7a, 0x7e, 0xef, 0x16, 0x75, 0x08, 0x68,
      0x44, 0xb4, 0x7c, 0xc1, 0x29, 0x24, 0xe5, 0xf3,
      0xf5, 0xc3, 0xc1, 0x8f, 0xc2, 0x69, 0x12, 0xa8,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char quad_msgs__msg__RobotPlanDiagnostics__TYPE_NAME[] = "quad_msgs/msg/RobotPlanDiagnostics";

// Define type names, field names, and default values
static char quad_msgs__msg__RobotPlanDiagnostics__FIELD_NAME__compute_time[] = "compute_time";
static char quad_msgs__msg__RobotPlanDiagnostics__FIELD_NAME__cost[] = "cost";
static char quad_msgs__msg__RobotPlanDiagnostics__FIELD_NAME__iterations[] = "iterations";
static char quad_msgs__msg__RobotPlanDiagnostics__FIELD_NAME__horizon_length[] = "horizon_length";
static char quad_msgs__msg__RobotPlanDiagnostics__FIELD_NAME__complexity_schedule[] = "complexity_schedule";
static char quad_msgs__msg__RobotPlanDiagnostics__FIELD_NAME__element_times[] = "element_times";

static rosidl_runtime_c__type_description__Field quad_msgs__msg__RobotPlanDiagnostics__FIELDS[] = {
  {
    {quad_msgs__msg__RobotPlanDiagnostics__FIELD_NAME__compute_time, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__RobotPlanDiagnostics__FIELD_NAME__cost, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__RobotPlanDiagnostics__FIELD_NAME__iterations, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__RobotPlanDiagnostics__FIELD_NAME__horizon_length, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__RobotPlanDiagnostics__FIELD_NAME__complexity_schedule, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__RobotPlanDiagnostics__FIELD_NAME__element_times, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
quad_msgs__msg__RobotPlanDiagnostics__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {quad_msgs__msg__RobotPlanDiagnostics__TYPE_NAME, 34, 34},
      {quad_msgs__msg__RobotPlanDiagnostics__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This is a message to hold local plan diagnostics\n"
  "\n"
  "float64 compute_time\n"
  "float64 cost\n"
  "uint32 iterations\n"
  "uint32 horizon_length\n"
  "uint32[] complexity_schedule\n"
  "float64[] element_times";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
quad_msgs__msg__RobotPlanDiagnostics__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {quad_msgs__msg__RobotPlanDiagnostics__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 179, 179},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
quad_msgs__msg__RobotPlanDiagnostics__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *quad_msgs__msg__RobotPlanDiagnostics__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
