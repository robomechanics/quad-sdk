#ifndef PACKAGE_TEMPLATE_H
#define PACKAGE_TEMPLATE_H

#include <ros/ros.h>

//! A template class for spirit
/*!
   PackageTemplate is a container for all of the logic utilized in the template node.
   The implementation must provide a clean and high level interface to the core algorithm
*/
class PackageTemplate {
public:
	/**
	 * @brief Constructor for PackageTemplate Class
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type PackageTemplate
	 */
	PackageTemplate(ros::NodeHandle nh);

private:

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;
};

#endif // PACKAGE_TEMPLATE_H
