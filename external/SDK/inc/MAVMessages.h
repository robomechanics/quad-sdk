/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Tom Jacobs <tom.jacobs@ghostrobotics.io>
 */

#pragma once

/**
 * @file MAVMessages.h
 * @brief This file contains public message types needed for the ethernet interface.
 */

#include <simple.pb.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef struct GRMTwist
{
	Vector3 linear, angular;
} GRMTwist;// 24

typedef struct GRMPose
{
	float position[3], euler[3];
} GRMPose;

typedef struct GRMBehaviorCmd
{
	uint32_t id;
	GRMTwist twist;
	GRMPose pose;
	uint32_t mode;
} GRMBehaviorCmd;

#ifdef __cplusplus
} // extern "C"
#endif
