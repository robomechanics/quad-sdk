#ifndef PACKAGE_TEMPLATE_H
#define PACKAGE_TEMPLATE_H

#include <ros/ros.h>

//! A template class for spirit
/*!
   PackageTemplate is a container for all of the logic utilized in the template node.
   The implementation must provide a clean and high level interface to the core algorithm
*/
class contact_detection {
public:
	/**
	 * @brief Constructor for PackageTemplate Class
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type PackageTemplate
	 */
	contact_detection(ros::NodeHandle nh);

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

#endif // PACKAGE_TEMPLATE_H
