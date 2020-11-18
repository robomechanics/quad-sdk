/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#pragma once

#include "Robot.hpp"

/**
 * @brief Set of functions each platform needs to implement for the SDK
 * 
 */
namespace gr {
  
void bspInit(Robot *pRobot, int argc, char *argv[]);

void bspBegin(Robot *pRobot);

uint32_t bspGetMillis();

int bspJointGetRaw(const RobotParams *P, int i, JointState *rawState);

int bspJointSendCommand(const RobotParams *P, int i, uint16_t mode, float setpoint);

bool bspGetExternalForce(const RobotParams *P, int i, Eigen::Vector3f &endEffForce);


/**
 * @brief Actuation update (critical, hard real-time)
 * @details Should be called by the implementation
 */
void sdkUpdateA(Robot *r);
/**
 * @brief Behavior and other task update (less critical, but still should be called as often as possition)
 * @details Should be called by the implementation
 */
void sdkUpdateB(Robot *r);

}
