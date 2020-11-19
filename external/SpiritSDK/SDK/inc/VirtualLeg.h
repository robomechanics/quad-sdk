/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef VirtualLeg_h
#define VirtualLeg_h

#include <SDK.h>
#include <Motor.h>

enum VirtualLegMode {
	VirtualLegMode_STAND=0, VirtualLegMode_STANCE, VirtualLegMode_FLIGHT, //used by all
	VirtualLegMode_FLIGHT_RETRACT, //used by trot
	// Special for leaping
	VirtualLegMode_LEAP_WAIT,
	VirtualLegMode_LEAP_STANCE,
	VirtualLegMode_LEAP_LAND
};

// BehaviorMode will be set to one of these
enum GaitMode
{
	// Common for all behaviors
	GaitMode_STAND = 0,
	GaitMode_RUN,
	GaitMode_LEAP
};

/** 
 *
 * @brief Class for controlling groups of leg abstractions
 */
template< int N > class VirtualLeg {
public:
	uint32_t tTD = 0, tLO = 0;
	VirtualLegMode mode = VirtualLegMode_STAND;
	int legi[N];//indices of physical legs
	float speedAccum = 0;
	float speed = 0;

	// init
	/**
	 * @brief begin function for initializing virtual leg
	 */
	virtual void begin(VirtualLegMode initMode = VirtualLegMode_STANCE) {
		mode = initMode;
		tTD = R->millis;
//		X.xd = 0;
	}
	/**
	 * @brief End function that puts legs in stand mode
	 *
	 */
	virtual void end() {
		mode = VirtualLegMode_STAND;
	}

	// get functions that return average
	/**
	 * @brief Gets the average of the legs 
	 * @return Average of leg positions
	 */
	inline float getPosition(int i) {
		float thesum=0;
		for (int j=0; j<N; ++j) {
			thesum += limb[legi[j]].getPosition(i);
		}
		return thesum/((float)N);
	}
	/**
	 * @brief Gets the average velocity of the legs
	 * @param i index for iterating through the legs
	 * @return Average velocity of the legs
	 */
	inline float getVelocity(int i) {
		float thesum=0;
		for (int j=0; j<N; ++j) {
			thesum += limb[legi[j]].getVelocity(i);
		}
		return thesum/((float)N);
	}

	/**
	 * @brief Get velocity of the "virtual leg" end effector in the world frame
	 * @details This can be used to get an estimate of the body velocity if the toe is known to be stationary, for instance.
	 * 
	 * @param toeSpeed world-frame velocity of the end effector
	 */
	inline Vector3 getVelocityWorldFrame()
	{
		Vector3 ret = {0, 0, 0}, dummy;
		for (int j=0; j<N; ++j) {
			dummy = limb[legi[j]].getVelocityWorldFrame();
			ret.x += dummy.x;
			ret.y += dummy.y;
			ret.z += dummy.z;
		}
		ret.x = ret.x / ((float)N);
		ret.y = ret.y / ((float)N);
		ret.z = ret.z / ((float)N);
		return ret;
	}
	/**
	 * @brief Sets proportional and derivative gains for the virtual leg PD control
	 * @param i index for iterating through the limb
	 * @param Kp Proportional gain
	 * @param Kd Derivative gain
	 */
	inline void setGain(int i, float Kp, float Kd=0) {
		for (int j=0; j<N; ++j) {
			limb[legi[j]].setGain(i, Kp, Kd);
		}
	}
//	inline float getToeForceRadial() {
//		float uth, ur, urtot = 0;
//		for (int j=0; j<N; ++j) {
//			leg[legi[j]].getToeForce(ur, uth);
//			urtot += ur;
//		}
//		return urtot;
//	}
};

/**
 * @brief Class for controlling two sets of leg abstractions at once
 *
 */
class LegPair : public VirtualLeg< 2 > {
public:
	LegPair(int i1, int i2) {
		legi[0] = i1;
		legi[1] = i2;
	}

	// MEAN/DIFF FUNCTIONS
	/** 
	 * @brief Differentially sets voltage control for a pair of legs
	 * @param i index for iterating through the leg pair
	 * @param mean the average of the voltage control PWM setting
	 * @param diff the difference from the mean for each leg 
	 */ 
	inline void setOpenLoop(int i, float mean, float diff) {
		limb[legi[0]].setOpenLoop(i, mean-diff);
		limb[legi[1]].setOpenLoop(i, mean+diff);
	}
	/** 
	 * @brief Differentially sets positions for a pair of legs
	 * @param i index for iterating through the leg pair
	 * @param mean the average of the two positions set to the legs
	 * @param diff the difference from the mean for each leg 
	 */
	inline void setPosition(int i, float mean, float diff) {
		limb[legi[0]].setPosition(i, mean-diff);
		limb[legi[1]].setPosition(i, mean+diff);
	}
};

#endif
