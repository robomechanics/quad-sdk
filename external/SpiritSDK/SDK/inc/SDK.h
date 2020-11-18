/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#pragma once

#include "Robot.hpp"
#include "bsp.hpp"
#include "SimSetup.hpp"
#include "MAVLinkUser.hpp"
#include "HardwareSetup.h"

namespace gr {
	
/** @addtogroup SDK Basic SDK operations
 *  @{
 */

/**
 * Defined separately for each platform, in units of Hz
 */
extern int CONTROL_RATE;
/**
 * Maximum number of limbs
 */
#define MAX_LIMB_COUNT              	4

// for _write, special FILE*
//#define LOGGER_FILENO								3

/**
 * @brief Value of BehaviorCmd::mode that should stop the behavior
 */
#define BehaviorMode_STOP		(0)
/**
 * @brief Value of BehaviorCmd::mode that should start the behavior. Higher values can be customized for each behavior
 */
#define BehaviorMode_RUN		(1)

/**
 * @brief System clock time (in microseconds)
 */
uint32_t clockTimeUS();

/** @} */ // end of addtogroup


/** @addtogroup Debug Debugging support
 *  @{
 */

/**
 * @brief Set the rate at which the debug() user function is called
 * @param hz Rate in Hz; set 0 to disable debug printing
 */
void setDebugRate(int hz);

/** @} */ // end of addtogroup

}
