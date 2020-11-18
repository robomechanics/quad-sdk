/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#pragma once

/** @addtogroup Simulation Simulation-specific only for supported simulation environments
 *  @{
 */

namespace gr {

typedef void (*ResetPoseType)(void *);

/**
 * @brief Set a URDF joint to a position (before applying constraints or starting SDK control). You must call this for all the joints.
 * 
 * @param jointName URDF joint name
 * @param targetPos Position of joint in radians
 * @return true Joint was found
 * @return false Joint not found
 */
bool simResetJointState(std::string jointName, float targetPos);

/**
 * @brief Call between init() and begin() to use a custom URDF file (alpha)
 * 
 * @param filename custom URDF in the urdf/ directory. e.g. if there is a file urdf/myrobot.urdf, call `setCustomURDF("myrobot.urdf")`
 * @param resetPoseFunction Function to call after loading URDF to set the initial joint positions. You must call simResetJointState for each joint the SDK can use.
 * @param resetPoseParam Will be called as `resetPoseFunction(resetPoseParam)`
 * @param hoisted set to true to fix the base
 */
void simSetCustomURDF(std::string filename, ResetPoseType resetPoseFunction, void *resetPoseParam, bool hoisted = false);

enum ConstraintType
{
	// ConstraintType_REVOLUTE = 0,
	ConstraintType_PRISMATIC = 1,
	// ConstraintType_SPHERICAL = 2,
	// ConstraintType_PLANAR = 3,
	ConstraintType_FIXED = 4,
	ConstraintType_POINT2POINT = 5,
};

/**
 * @brief Create a constraint between two URDF joints (alpha)
 * 
 * @param parentJointName URDF joint name of parent joint
 * @param childJointName  URDF joint name of child joint
 * @param parentPoint  Parent link-frame coordinates of the attachment point
 * @param childPoint Child link-frame coordinates of the attachment point
 * @return int 
 */
int simConstrainLinks(std::string parentJointName, std::string childJointName, const float parentPoint[/* 3 */], const float childPoint[/* 3 */]);
/**
 * @brief Create a constraint between the world and the object (does not work yet)
 * 
 * @param type Constraint type (only a few are supported)
 * @param parentPoint 
 * @param childPoint 
 * @param jointAxis 
 * @return int 
 */
int simConstrainBase(ConstraintType type, const float parentPoint[/* 3 */], const float childPoint[/* 3 */], const float jointAxis[/* 3 */]);
	
}

/** @} */ // end of addtogroup
