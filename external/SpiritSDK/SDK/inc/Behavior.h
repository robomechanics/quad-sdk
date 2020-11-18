/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#pragma once

#include "Peripheral.h"
#include <stdint.h>

#if defined(__clang__)

#elif defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#elif defined(_MSC_VER)

#endif

namespace gr {
  
/** @addtogroup Behavior Behavior
 *  @{
 */

/**
 * @brief Abstract base class for implementing behaviors. Only one behavior is active
 * after initialization.
 */
class Behavior : public Peripheral {
public:
	/**
	 * @brief Should return false if the task is complete (for switching to another task)
	 * @return True if running
	 */
  virtual bool running() { return false; }
  /**
   * @brief Called when a stop is requested by a BehaviorCmd
   */
  virtual void end() {}

  /**
   * @brief Send a signal to the behavior; for example a "leap" signal, or a "rollover" signal. These are typically used to send a short message to a running behavior about executing some kind of transition.
   * 
   * @param mode ::BehaviorMode_STOP and ::BehaviorMode_RUN are reserved, 2 and 3 are set by the joystick.
   */
  virtual void signal(uint32_t sig) {}
};

/** @} */ // end of addtogroup

#if defined(__clang__)

#elif defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic pop
#elif defined(_MSC_VER)

#endif
  
}

