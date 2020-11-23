/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef SMath_h
#define SMath_h

#include <stdint.h>
#include <math.h>
#ifdef CORE_CM4
#include <arm_math.h>
#endif
// Eigen
#if defined(__clang__)

#elif defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#elif defined(_MSC_VER)

#endif
/**
 * @brief This is defined in the SDK to turn off asserts in Eigen.
 */
#define EIGEN_NO_DEBUG
#include <Eigen/Core>
#if defined(__clang__)

#elif defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic pop
#elif defined(_MSC_VER)
// 
#endif

namespace gr
{

/** @addtogroup SMath SDK math support
 * @brief Math functions for the Ghost Robotics SDK.
 *  @{
 */

#ifdef CORE_CM4
#define fastcos 	arm_cos_f32
#define fastsin	 	arm_sin_f32
#define fastsqrt	sqrtf
#else
#define fastcos 	cosf
#define fastsin	 	sinf
#define fastsqrt	sqrtf
#endif

#ifndef PI
/**
 * @brief Macro for PI
 */
#define PI 				(3.14159265359f)
#endif
#ifndef HALF_PI
/**
 * @brief Macro for PI/2
 */
#define HALF_PI 	(1.57079632679f)
#endif
#ifndef TWO_PI
/**
 * @brief Macro for 2*PI
 */
#define TWO_PI 		(6.28318530718f)
#endif


//Other macros

/**
 * @brief Constraints an amount between low and high inclusive.
 */
float constrain(float amt, float low, float high);

#ifndef DEG_TO_RAD
/**
 * @brief Used to convert degrees to radians.
 */
#define DEG_TO_RAD 0.017453292519943295769236907684886f 
#endif
#ifndef radians
/**
 * @brief Convert degrees to radians.
 */
#define radians(deg) ((deg)*DEG_TO_RAD)
#endif
#ifndef degrees
/**
 * @brief Convert radians to degrees.
 */
#define degrees(rad) ((rad)*RAD_TO_DEG)
#endif

// other funcs from WMath...define only if not MCU

/**
 * @brief Map a float (this is different from the Arduino implementation).
 * 
 * @param x some real number
 * @param in_min 
 * @param in_max 
 * @param out_min 
 * @param out_max 
 * @return x mapped from [in_min, in_max] to [out_min, out_max]
 */
float map(float x, float in_min, float in_max, float out_min, float out_max);

inline float constrain_map(float x, float in_min, float in_max, float out_min, float out_max) {
  return constrain(map(x, in_min, in_max, out_min, out_max), fminf(out_min, out_max), fmaxf(out_min, out_max));
}

/**
 * @brief Linear interpolation between given endpoints.
 * @param from Lower endpoint
 * @param to Higher endpoint
 * @param frac float between 0.0 and 1.0
 * @return Interpolated value
 */
float interp1(float from, float to, float frac);

/**
 * @brief Assumes xdata is sorted.
 * 
 * @param Ndata 
 * @param xdata
 * @param x 
 */
inline float interp1(int Ndata, const float xdata[], const float ydata[], float x) {
  // Need at least two points
  if (Ndata < 2)
    return ydata[0];
  // If test point withing data OK
  for (int i=1; i<Ndata; ++i) {
    if (x <= xdata[i] && x > xdata[i-1]) {
      return map(x, xdata[i-1], xdata[i], ydata[i-1], ydata[i]);
    }
  }
  // At the extremes saturate
  if (x < xdata[0])
    return ydata[0];
  if (x > xdata[Ndata-1])
    return ydata[Ndata-1];

  // Shouldn't get here
  return ydata[0];
}

/**
 * @brief Interpolate time.
 */ 
float interpFrac(uint32_t startTime, uint32_t endTime, uint32_t now);

// Mod functions

/**
 * @brief Mod an angle to between -PI and PI.
 * @param f Input angle
 * @return Result between -PI and PI
 */
float fmodf_mpi_pi(float f);
/**
 * @brief Mod an angle to between 0 and 2*PI.
 * @param f Input angle
 * @return Result between 0 and 2*PI
 */
float fmodf_0_2pi(float f);
/**
 * @brief Mod between 0 and 1.
 * @param f Input value
 * @return Result between 0 and 1
 */
float fmodf_0_1(float f);
/**
 * @brief Mod between -0.5 and +0.5.
 * @param f Input value
 * @return Result between -0.5 and +0.5
 */
float fmodf_mp5_p5(float f);
/**
 * @brief Change coords to mean and diff on a circle (accounting for wraparound).
 * @param a First angle in rad
 * @param b Second angle in rad
 * @param mean Mean angle
 * @param diff Difference angle
 */
extern void circleMeanDiff(float a, float b, float *mean, float *diff);

/**
 * @brief Digital low pass filter.
 */
enum DLPFType {
	DLPF_ANGRATE,   // Returns a speed for an angle (mods 2*pi)
	DLPF_RATE,      // Returns a speed
	DLPF_SMOOTH,    // Just smooths the input (no derivative involved)
	DLPF_INTEGRATE  // Integrates once, and decays for drift by smooth
};

/**
 * @brief Digital autoregressive low pass filter.
 * @details Use by calling DLPF
 *
 */
class DLPF {
public:
	// Parameters
	DLPFType type;
	float smooth; // between 0 and 1; high means more smooth
	float freq; // in Hz
	// State
	float oldVal;
	float vel;

	DLPF() :
			type(DLPF_SMOOTH), smooth(0.5), freq(1000), oldVal(0), vel(0) {
	}
	/**
	 * @brief Initialize filter
	 * @param smooth Smoothing factor (0 = no smoothing, 1 = never updates)
	 * @param freq Update frequency in Hz
	 * @param type One of `DLPF_ANGRATE`, `DLPF_RATE`, `DLPF_SMOOTH`, `DLPF_INTEGRATE`
	 */
	virtual void init(float smooth, float freq, DLPFType type) {
		this->smooth = smooth;
		this->freq = freq;
		this->type = type;
	}
	/**
	 * @brief Call this at the rate given in freq in init()
	 * @param val New measurement
	 */
	virtual float update(float val)  {
		float delta;
		switch (type) {
		case DLPF_ANGRATE:
			delta = fmodf_mpi_pi(val - oldVal);
			vel = interp1(delta * freq, vel, smooth);
			oldVal = val;
			return vel;
		case DLPF_RATE:
			delta = val - oldVal;
			vel = interp1(delta * freq, vel, smooth);
			oldVal = val;
			return vel;
		case DLPF_INTEGRATE:
			oldVal = interp1(oldVal + val / freq, oldVal, smooth);
			return oldVal;
		case DLPF_SMOOTH:
		default:
			oldVal = interp1(val, oldVal, smooth);
			return oldVal;
		}
	}
};

/**
 * @brief Generic proportional-derivative control (for use in a motor, look at the @ref Joint instead).
 */
class PD: public DLPF {
public:
	float Kp, Kd;

	PD() {
		// DLPF constructor is automatically called
		type = DLPF_RATE;
	}

	/**
	 * @brief Computes the PD control from the given error
	 * 
	 * @param pos Current measured position
	 * @param setpoint Specified position setpoint
	 * 
	 * @return - Kp * val - Kd * (filtered velocity)
	 */
	virtual float update(float pos, float setpoint) {
		float error =
				(type == DLPF_ANGRATE) ?
						fmodf_mpi_pi(pos - setpoint) : (pos - setpoint);
		return -Kp * error - Kd * DLPF::update(pos);
	}

	/**
	 * @brief Set gains
	 *
	 * @param Kp Proportional gain
	 * @param Kd Dissipative (derivative) gain
	 */
	void setGain(float Kp, float Kd) {
		this->Kp = Kp;
		this->Kd = Kd;
	}
};
	
} // namespace gr

/** @} */ // end of addtogroup


#endif
