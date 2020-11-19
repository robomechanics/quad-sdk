/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#pragma once

#include <vector>
#include "SMath.h"
#include "Behavior.h"
#include "simple.pb.h"

namespace gr {

// TODO: reinstate non-fixed size
typedef Eigen::Matrix<float, 3, 3> MatMN_t; // Keep this type regular column major
typedef Eigen::Matrix<float, 3, 1> VecN_t;
typedef Eigen::Matrix<float, 3, 1> VecM_t;
// For converting from float arrays
typedef Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor> > MCMatMN;
typedef Eigen::Map<VecM_t> MVecM;
typedef Eigen::Map<VecN_t> MVecN;
typedef Eigen::Map<const VecN_t> MCVecN;
typedef Eigen::Map<const VecM_t> MCVecM;

// Order of the joints in the limb
#define HIPJ (0)
#define KNEEJ (1)
#define ABDUCTIONJ (2)

class Robot : public RobotData {
public:
	/**
	 * @brief Initialize the SDK for a particular robot platform
	 * 
	 * @param type Robot type (see RobotParams::Type)
	 * @param argc Number of command line arguments (pass from main())
	 * @param argv Command line arguments (pass from main())
	 * @return True if succeeded
	 */
	bool init(RobotParams_Type type, int argc, char *argv[]);

	/**
	 * @brief Commences the various control loops and tasks.
	 * @details This should be the *last* function the user calls at the end of main. It never returns.
	 * @return Never returns
	 */
	int begin();

	void updatePeripherals() {
		for (auto p : peripherals)
			p->update(this);
	}

	inline VecM_t getLimbPosition(int i) const {
		return MCVecM(S.limbs[i].position);
	}

	inline VecM_t getLimbVelocity(int i) const {
		return MCVecM(S.limbs[i].velocity);
	}

	inline MatMN_t getLimbJac(int i) const {
		return MCMatMN(S.limbs[i].Jac);
	}

	inline float getJointPosition(int legi, int jointInLegi) const {
		return S.limbs[legi].q[jointInLegi];
	}

	inline VecN_t getJointExternalTorqueEstimate(int i) const {
		return MCVecN(S.limbs[i].torqueExt);
	}

	inline void setJointOpenLoops(int legi, bool torqueMode, const VecN_t &u) {
		C.limbs[legi].mode = torqueMode ? LimbCmdMode_JOINT_TORQUE : LimbCmdMode_JOINT_CURRENT;
		MVecN(C.limbs[legi].feedforward) = u;
	}

	inline void setJointPositions(int legi, bool torqueMode, const VecN_t &positionSetpoints, const VecN_t &kp, const VecM_t &kd) {
		C.limbs[legi].mode = torqueMode ? LimbCmdMode_JOINT_POSITION_OVER_TORQUE : LimbCmdMode_JOINT_POSITION_OVER_CURRENT;
		MVecN(C.limbs[legi].feedforward) = positionSetpoints; // this value is used to hold the setpoint
		MVecN(C.limbs[legi].kp) = kp;
		MVecN(C.limbs[legi].kd) = kd;
	}

	/**
	 * @brief Set a limb force command (assumes joints in torque mode). `u = ff - kp * position - kd * velocity`
	 * 
	 * @param ff Feedforward torque (can include `kp * position_desired + kd * velocity_desired`)
	 * @param kp Position tracking gain
	 * @param kd Velocity tracking gain
	 */
	void setLimbForce(int i, const VecM_t &ff, const VecM_t &kp = VecM_t::Zero(), const VecM_t &kd = VecM_t::Zero());
	/**
	 * @brief Set a limb position; `u = kp * (setpoint - position) - kd * velocity`
	 */
	inline void setLimbPosition(int i, const VecM_t &setpoint, const VecM_t &kp, const VecM_t &kd) {
		setLimbForce(i, kp.cwiseProduct(setpoint), kp, kd);
	}
	inline void setLimbPosition(int i, const VecM_t &setpoint, float kp, float kd) {
		setLimbPosition(i, setpoint, VecM_t::Constant(kp), VecM_t::Constant(kd));
	}
	/**
	 * @brief Set a limb position; `u = kp * (setpoint - position) - kd * velocity`
	 */
	inline void setLimbForce(int i, const VecM_t &setpoint, float kp, float kd) {
		setLimbForce(i, kp * setpoint, VecM_t::Constant(kp), VecM_t::Constant(kd));
	}

	inline void setLimbForce(int i, const float *ff, const float *kp, const float *kd) {
		setLimbForce(i, MCVecM(ff), MCVecM(kp), MCVecM(kd));
	}

	/**
	 * @brief Call after setting standard limb commands to set advanced options. 
	 * mode8[0] = LSB = standard robot commands. Higher bytes described in arguments
	 * 
	 * @param i Limb number
	 * @param ji Joint index within limb
	 * @param value Bypass value
	 * @param zeroCurrent Set true to override the limb command and set zero current
	 * @param bypassMode Send direct commands to motor controller for diagnostics (bypasses other commands completely)
	 */
	void setJointAdvanced(int i, int ji, float value = 0, bool bypassCurrent = false, uint8_t bypassMode = 0);

	// not inheriting from this to keep it isolated
	RobotParams P = RobotParams_init_zero;
	// These have to be public for sdkUpdateA to modify them
	RobotState S = RobotState_init_zero;
	RobotCmd C = RobotCmd_init_zero;

	HardwareConfig hardwareConfig = HardwareConfig_init_zero;
	BehaviorConfig behaviorConfig = BehaviorConfig_init_zero;
	// Use push_back etc.
	std::vector<Behavior *> behaviors;
	std::vector<Peripheral *> peripherals;
protected:
};

}
