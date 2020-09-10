#ifndef EKF_ESTIMATOR_H
#define EKF_ESTIMATOR_H

#include <ros/ros.h>

//! Implements online EKF based state estimation 
/*!
   EKFEstimator implements all estimator logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class EKFEstimator {
  public:
	/**
	 * @brief Constructor for EKFEstimator
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type EKFEstimator
	 */
	EKFEstimator(ros::NodeHandle nh);

private:

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;
};
#endif // EKF_ESTIMATOR_H
