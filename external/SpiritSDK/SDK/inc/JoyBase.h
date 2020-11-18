/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#pragma once

#include "Peripheral.h"
#include "simple.pb.h"
#include <SMath.h>

namespace gr {

/**
	 * @brief Updates a float to desired within rate limits
	 * 
	 * @param current state to update
	 * @param desired new desired
	 * @param rateLimitOneStep limit in units of /step
	 */
void rateLimitedUpdate(float &current, float desired, float rateLimitOneStep);

/**
 * @brief Some standard that lets the RC remote override; put in this centralized place
 * 
 * @return true RC remote takes priority
 * @return false Can use high-level commands
 */
bool joyRCOverride(const Robot *R);

class JoyBase : public Peripheral {
protected:
	float JOY_SPEED_SENS = 1.0;//m/s. 0.6 (low), 0.8 (medium), 1.5 (high)
	float JOY_YAW_SENS = 1.6;//rad/s.

	float JOY_SMOOTH_TWIST_LINEAR = 1.0;
	float JOY_SMOOTH_TWIST_ANGULAR = 0.5;
	float JOY_SMOOTH_TWIST_POSE_Z = 0.7;

	BehaviorCmd normalizedBehaviorCmd; // temporary storage before scaling

public:
	/**
	 * @brief Init filters
	 * 
	 */
	void init();
	
	// From peripheral
	virtual void begin(const Robot *R) = 0;
	virtual void update(Robot *R) = 0;
	/**
	 * @brief This should output a *normalized* BehaviorCmd. The SDK will then scale the inputs according the robot's abilities.
	 * 
	 * @param b 
	 */
	virtual void toNormalizedBehaviorCmd(const Robot *R, BehaviorCmd *b) = 0;

	/**
	 * @brief This will be called by the SDK to (a) get the normalized behavior cmd, and (b) scale the inputs
	 * 
	 * @param b 
	 */
	void toBehaviorCmd(const Robot *R, BehaviorCmd *b);
	
	// Filters for twist
	DLPF axF[4];
	// set at init
	int JOY_RATE = 0;

	/**
	 * @brief Set joystick sensitivity
	 * 
	 * @param speedSens Joystick axis at max range maps to +/- speedSens m/s
	 * @param yawSens Joystick axis at max range maps to +/- yawSens rad/s
	 */
	void setSensitivity(float speedSens, float yawSens) {
		JOY_SPEED_SENS = speedSens;
		JOY_YAW_SENS = yawSens;
	}

	// State for filtering
	Vector3 linearDes = {0, 0, 0}, angularDes = {0, 0, 0};

	virtual ~JoyBase() {}
};

	
}
