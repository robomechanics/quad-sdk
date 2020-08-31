#ifndef SPIRIT_ESTIMATOR_H
#define SPIRIT_ESTIMATOR_H

#include <ros/ros.h>

//! Implements online EKF based state estimation for spirit
/*!
   SpiritEstimator implements all estimator logic for spirit. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class SpiritEstimator {
  public:
	/**
	 * @brief Constructor for SpiritEstimator
	 * @param[in] nh ROS Nodehandle to publish and subscribe from
	 * @return Constructed object of type SpiritEstimator
	 */
	SpiritEstimator(ros::NodeHandle nh);

	/**
	 * @brief Primary work function in class, called in node file for this component
	 */
	void update();

  private:
	//! ROS Nodehandle to publish and subscribe from
	ros::NodeHandle _nh;
};


#endif // SPIRIT_ESTIMATOR_H
