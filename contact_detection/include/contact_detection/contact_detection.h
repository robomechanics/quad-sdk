#ifndef CONTACTDETECTION_H
#define CONTACTDETECTION_H

#include <ros/ros.h>

//! Contact detection class for spirit
/*!
  Contact detection module determines the probability of contact for each leg for state estimation and mpc control
*/
class ContactDetection {
public:
	/**
	 * @brief Constructor for ContactDetection Class
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type ContactDetection
	 */
	ContactDetection(ros::NodeHandle nh);

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();

private:
	/// ROS subscriber
	ros::Subscriber sample_sub;

	/// ROS Publisher
	ros::Publisher sample_pub;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data;
	double update_rate_;
};

#endif // CONTACTDETECTION_H
