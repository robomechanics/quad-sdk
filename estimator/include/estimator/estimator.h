#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <ros/ros.h>

//! Implements online EKF based state estimation 
/*!
   Estimator implements all estimator logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class Estimator {
  public:
	/**
	 * @brief Constructor for Estimator
	 * @return Constructed object of type Estimator
	 */
	Estimator();

	/**
	 * @brief Primary work function in class, called in node file for this component
	 */
	void update();

};


#endif // ESTIMATOR_H
