/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef Motor_h
#define Motor_h

#include <SDK.h>
#include <SMath.h>


/** @addtogroup Joint
 *  @{
 */

/**
 * @brief Class for controlling individual joints
 */
class Joint {
	const int i;
	/**
	 * @brief PWM or CURRENT
	 */
	JointMode openLoopMode = JointMode_PWM;
public:
	Joint(int _i) :
			i(_i) {
	}
	/**
	 * @brief Returns motor position before a zero offset or direction is applied
	 * @details Can be used to set the zero position of motors.
	 * @return Raw zero position returned from the motor.
	 */
	float getRawPosition() const;
	
	/**
     * @brief Gets the position of the specified joint. 
     * 
     * @return The position of the motor in radians relative to a pre-defined zero. 
     */
	inline float getPosition() const { return S->joints[i].position; }
	/**
     * @brief Gets the velocity of the specified joint. 
     * 
     * @return The speed of the motor in radians per second. 
     */
	inline float getVelocity() const { return S->joints[i].velocity; }
	/**
     * @brief Gets an estimate of the torque experienced by the motor. Depending on the underlying motor driver, current readings may be filtered together with the commanded signal to get the best possible estimate.
     * 
     * @return The perceived torque of the motor in Newton-meters. 
     */
	inline float getTorqueEst() const { return S->joints[i].torqueEst; }
	/**
     * @brief Gets the joint current reported by the motor controller (if available).
     * 
     * @return The current in Amps. 
     */
	inline float getCurrent() const { return S->joints[i].current; }
	/**
     * @brief Gets the joint temperature reported by the motor controller (if available).
     * 
     * @return The temperature in C.
     */
	inline float getTemperature() const { return S->joints[i].temperature; }
	/**
	 * @brief Returns the raw signal sent to the motor when in PWM mode (`P->joints[i].mode = JointMode_PWM`), or when setOpenLoop() or setPosition() are used.
	 * @return Raw PWM value between [-1, 1].
	 */
	float getOpenLoop() const;

	/**
	 * @brief Uses PWM to approximate voltage control of the motor. 
	 * @details The PWM value should be between 0 and 1 for forward operation, and -1 and 0 for reverse operation.
	 * @param setpoint The PWM value between [-1, 1].
	 */
	inline void setOpenLoop(float setpoint) {
		C->joints[i].mode = openLoopMode;
		C->joints[i].setpoint = setpoint;
	}
	/**
	 * @brief Set the Open Loop Mode (PWM or CURRENT)
	 * 
	 * @param mode JointMode_PWM or JointMode_CURRENT
	 */
	inline void setOpenLoopMode(JointMode mode)
	{
		if (mode == JointMode_PWM || mode == JointMode_CURRENT || mode == JointMode_TORQUE)
			openLoopMode = mode;
	}
	/**
	 * @brief Get the Open Loop Mode
	 * 
	 * @return JointMode 
	 */
	inline JointMode getOpenLoopMode()
	{
		return openLoopMode;
	}
	/**
     * @brief Sets the proportional and derivative gains for a PD controller of motor position. 
     * @param Kp The desired proportional gain.
     * @param Kd The desired derivative gain.
     */
	inline void setGain(float Kp, float Kd = 0) {
		C->joints[i].Kp = Kp;
		C->joints[i].Kd = Kd;
	}
	/**
	 * @brief Sets the desired position for the motor, to be used in the PD Controller defined by setGain().
	 * @param setpoint The desired position for the motor, in radians relative to a pre-defined zero.
	 */
	inline void setPosition(float setpoint) {
		C->joints[i].mode = JointMode_POSITION;
		C->joints[i].setpoint = setpoint;
	}
	/**
	 * @brief Set the unwrap offset (only relevant when gearRatio != 1)
	 * 
	 * @param unwrapOffset integer such as -2, -1, 0, 1, 2, ... (default 0 when initializing)
	 */
	void setUnwrapOffset(int unwrapOffset);
};
/**
 * @brief Globally accessible array of each Joint.
 */
extern Joint joint[];


/** @} */ // end of addtogroup


/** @addtogroup Limb
 *  @{
 */

// For MultiDOF kinematics

/**
 * Maximum number of limbs
 */
#define MAX_LIMB_COUNT              	6

/**
 * N = dim of joint space
 */
#define MAX_JOINT_PER_LIMB_COUNT    	6

/**
 * M = dim of end-effector space.
 */
#define MAX_END_EFF_PER_LIMB_COUNT    6

//Using Eigen, low level control time ~= 750us, with CMSIS DSP ~= 570us (F3@96)

#if !defined(ARM_MATH_CM4)
// Dynamic size with max, since robot type is set at run time
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor, MAX_JOINT_PER_LIMB_COUNT, MAX_JOINT_PER_LIMB_COUNT> MatN;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, 0, MAX_JOINT_PER_LIMB_COUNT, 1> VecN;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, 0, MAX_END_EFF_PER_LIMB_COUNT, 1> VecM;
#endif

/**
 * @brief Typedef for function pointer to infinitesimal forward kinematics function
 * @details Use this function prototype to provide custom kinematics for limbs in P
 * 
 * @param kinParams (input) kinematic parameters needed in the kinematics (e.g. link lengths) that are set in P
 * @param jointAngs (input) vector of joint angles (size N)
 * @param limbPos (output) vector of end-effector coordinates (size M)
 * @param JacRowMajor (output) Jacobian matrix in row major form (size MxN)
 */
typedef void (*FKFunType)(const float *kinParams, const float *jointAngs, float *limbPos, float *JacRowMajor);
//typedef void (*FKFunType)(const float *, const VecN&, VecN&, MatN&);

/**
 * @brief Helpful define for extension end-effector coordinate for planar leg.
 */
#define EXTENSION (0)
/**
 * @brief Helpful define for angle end-effector coordinate for planar leg.
 */
#define ANGLE			(1)
/**
 * @brief Helpful define for abduction angle end-effector coordinate for 3DOF leg.
 */
#define ABDUCTION	(2)
/**
 * @brief Helpful define for x coordinate
 */
#define CARTX 		(0)
/**
 * @brief Helpful define for y coordinate for 3DOF leg
 */
#define CARTY 		(1)
/**
 * @brief Helpful define for z coordinate
 */
#define CARTZ 		(2)

/**
 * @brief Basic abstraction for controlling pre-defined kinematic chains.
 */
class Limb {
	const int i; // referenced to the limbs in LimbParams

	JointCmd cmd[MAX_END_EFF_PER_LIMB_COUNT];

public:
#if !defined(ARM_MATH_CM4)
	VecM position, velocity, force;
	// related joint states
	VecN jointPos, jointVel, jointTorque, jointExternalTorque;
	MatN Jac, JacI;// square for now
	// desired limb forces
	VecM limbForceDes;
	VecM extForceFromObs;

	// separate metric versions (may be not required if we force metric kinematics in the future)
	VecM metricPos, metricVel;
	MatN metricJac;

#else

	float position[MAX_END_EFF_PER_LIMB_COUNT];
	float velocity[MAX_END_EFF_PER_LIMB_COUNT];
	float force[MAX_END_EFF_PER_LIMB_COUNT];
	float extForceFromObs[MAX_END_EFF_PER_LIMB_COUNT];

	float jointPos[MAX_JOINT_PER_LIMB_COUNT];
	float jointVel[MAX_JOINT_PER_LIMB_COUNT];
	float jointTorque[MAX_JOINT_PER_LIMB_COUNT];
	float jointExternalTorque[MAX_JOINT_PER_LIMB_COUNT];
	float Jac[MAX_END_EFF_PER_LIMB_COUNT * MAX_JOINT_PER_LIMB_COUNT], JacI[MAX_END_EFF_PER_LIMB_COUNT * MAX_JOINT_PER_LIMB_COUNT];
	// desired limb forces
	float limbForceDes[MAX_END_EFF_PER_LIMB_COUNT];

	float *metricPos, *metricVel;
#endif

	Vector3 extForce, extTorque, velCartW;
	bool extWrenchAvailable = false;

	/**
	 * @brief At runtime, set dims of the dynamic sizes at init. For now must use M==N.
	 * @details Use this when adding new limbs or modifying existing presets after init().
	 * 
	 * @param M The number of end-effector DOFs
	 * @param N The number of joints (*must equal M for now*)
	 */
	void setDims(int M, int N);

	void updateState();

	void handleCommand();

	Limb(int _i) :
			i(_i) {
	}
	/**
	 * @brief This function gets the position of a generalized coordinate of the limb.
	 * @param coord an integer associated with one of the degrees-of-freedom of the limb
	 * @return The position of the limb in that coordinate.
	 */
	inline float getPosition(int coord) const { return position[coord]; }
	/**
	 * @brief This function gets the velocity of a generalized coordinate of the limb.
	 * @param coord an integer associated with one of the degrees-of-freedom of the limb
	 * @return The velocity of the limb in that coordinate.
	 */
	inline float getVelocity(int coord) const { return velocity[coord]; }
	/**
	 * @brief Get the world-frame velocity of the end effector.
	 * @details This can be used to estimate body velocity if the end-effector is known to be stationary.
	 * 
	 * @return world-frame velocity of the end-effector relative to the CoM
	 */
	Vector3 getVelocityWorldFrame() const;
	/**
	 * @brief Gets the actuator-produced end-effector force (linear component of wrench) in limb coordinates.
	 * @details Uses joint torque estimates and transforms through the infinitesimal kinematics to give force estimates at the end effector.
	 *
	 * @param coord an integer associated with one of the degrees-of-freedom of the limb
	 * @return The component of force along coordinate coord.
	 */
	inline float getForce(int coord) const { return force[coord]; }
	inline const float *getForce() const { 
#ifdef ARM_MATH_CM4
		return force;
#else
		return force.data();
#endif
	}
	/**
	 * @brief Gets the external end-effector force (linear component of wrench) using a momentum observer in limb coordinates.
	 * 
	 * @param coord an integer associated with one of the degrees-of-freedom of the limb
	 * @return The component of force along coordinate coord.
	 */
	inline float getExternalForce(int coord) const { return extForceFromObs[coord]; }
	inline const float *getExternalForce() const {
#ifdef ARM_MATH_CM4
		return extForceFromObs;
#else
		return extForceFromObs.data();
#endif
	}

	/**
	 * @brief Gets the end-effector external wrench (external contact force and momentum). Uses sensors if available, otherwise uses estimator.
	 * 
	 * @param extForceOut external force vector (can pass NULL)
	 * @param extTorqueOut external torque vector (can pass NULL)
	 * @return true if available,
	 *         false if unavailable (outputs not set)
	 */
	inline bool getExternalWrench(Vector3 *extForceOut, Vector3 *extTorqueOut) const
	{
		if (!extWrenchAvailable)
			return false;
		if (extForceOut != NULL)
			*extForceOut = extForce;
		if (extTorqueOut != NULL)
			*extTorqueOut = extTorque;
		return true;
	}
	/**
	 * Sets a limb to enable open loop torque control.
	 * @brief This function uses voltage control on a generalized coordinate of the limb.
     * @param coord an integer associated with one of the degrees-of-freedom of the limb
	 * @param setpoint the PWM value for the associated voltage control
	 */
	inline void setOpenLoop(int coord, float setpoint)
	{
		cmd[coord].mode = JointMode_PWM;
		cmd[coord].setpoint = setpoint;
	}
	/**
	 * @brief Sets PD controller gains for a generalized coordinate of the limb
	 * @param coord an integer associated with one of the degrees-of-freedom of the limb
	 * @param Kp the proportional gain for the PD controller
	 * @param Kd the derivative gain for the PD controller 
     */
	inline void setGain(int coord, float Kp, float Kd = 0)
	{
		cmd[coord].Kp = Kp;
		cmd[coord].Kd = Kd;
	}
	/**
	 * @brief This function sets the desired position for PD control of one of the degrees-of-freedom of the limb
	 * @param coord an integer associated with one of the degrees-of-freedom of the limb
	 * @param setpoint The desired position of the limb in generaized coordniates 
	 */
	inline void setPosition(int coord, float setpoint)
	{
		cmd[coord].mode = JointMode_POSITION;
		cmd[coord].setpoint = setpoint;
	}
};
/**
 * Globally accessible array of each Limb.
 */
extern Limb limb[];

/** @} */ // end of addtogroup


#endif
