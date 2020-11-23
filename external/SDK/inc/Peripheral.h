/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#pragma once

namespace gr {

/** @addtogroup Behavior Behavior
 *  @{
 */

class Robot;

/**
 * @brief Class for integrating peripheral sensors, tools, etc. The update() for these
 * is always called; every peripheral is "active" after starting up.
 */
class Peripheral {
public:
  /**
   * @brief Function called to initialize behavior / peripheral.
   * This gets called when the behavior / peripheral is started. May be called multiple times, i.e. when switching behaviors.
   */
  virtual void begin(const Robot *R) = 0;
  /**
   * @brief 
   *
   * @details 
   */

	/**
	 * @brief Function called repeatedly (at CONTROL_RATE) to update.
	 * The main logic for the behavior / peripheral goes here.
	 * 
	 * @param R Pointer to the robot class that is running this peripheral
	 */
  virtual void update(Robot *R) = 0;
};
/** @} */ // end of addtogroup
  
}
