/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef DebugOutput_h
#define DebugOutput_h

#include <stdint.h>

#ifndef ISRType
typedef void (*ISRType)();
#endif
/**
 * @brief Class for printing debugging information 
 */
class DebugOutput {
public:
  /**
   * @brief Enable function for starting the debug interrupt
   * @param freqHz The update frequency for the debug updates
   * @param fnptr Pointer to the interrupt function used 
   */	
  bool enable(uint32_t freqHz, ISRType fnptr);
};

#endif
