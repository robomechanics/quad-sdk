/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef GaitRunner_h
#define GaitRunner_h

#include <SMath.h>

/** @addtogroup Behavior
 *  @{
 */


extern int CONTROL_RATE;

/**
 * @brief Class for clocked gaits
 */
class GaitRunner {
public:
  float duty, offset, sweep, speed;
  GaitRunner(float duty, float offset, float sweep, float speed) : duty(duty), offset(offset), sweep(sweep), speed(speed) {
    phase = 0;
  }

  virtual void update() {
    phase += speed / ((float) CONTROL_RATE);
  }

  // Buehler clock
  float legClock(float phaseOffset, float turn) {
    // "turn" > 0 speeds up this leg, <0 slows down
    // bump up the sweep and offset
    float turnSweep = sweep * (1.0 + turn);
    float turnOffset = offset;// * (1.0 + turn);

    float legphase = fmodf_0_1(phase + phaseOffset);
    // fraction into phase
    float frac;
    if (legphase < duty) {
      // Slow phase
      frac = legphase / duty;
      return turnOffset + frac * turnSweep;
    }
    else {
      // Fast phase
      frac = (legphase - duty)/(1.0 - duty);
      return turnOffset + turnSweep + frac * (TWO_PI - turnSweep);
    }
  }

  // // Tail
  // float tailClock() {
  //   // +- 0.3 in yaw ("1") direction
  //   float tailphase = fmodf_0_1(phase);
  //   return -0.25 * arm_cos_f32((tailphase) * TWO_PI);
  // }
  // // cos, cos+0.5, sin, sin+0.5

// protected:
  float phase;
};

/** @} */ // end of addtogroup

#endif
