/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#pragma once

#include "simple.pb.h"

namespace gr
{

/**
 * @brief Get the MotorModelParams object
 * 
 * @param type 
 * @return const MotorModelParams* 
 */
const MotorModelParams *getMotorModelParams(MotorType type);

/**
 * @brief Get current -> torque from motor model
 * 
 * @param current Commanded current (A)
 * @param params 
 * @return float Torque (Nm)
 */
float modelTorqueFromCurrent(float current, const MotorModelParams *params);

/**
 * @brief Get PWM -> torque using a motor model
 * 
 * @param pwm supplied PWM command [-1, 1]
 * @param vel velocity of the motor (rad/s)
 * @param params 
 * @return float  Torque (Nm)
 */
float modelTorqueFromPWM(float pwm, float vel, const MotorModelParams *params);

} // namespace gr
