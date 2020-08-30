#ifndef SPIRIT_COMPONENT_TEMPLATE_H
#define SPIRIT_COMPONENT_TEMPLATE_H

#include <ros/ros.h>

//! A template class for spirit
/*!
   SpiritComponentTemplate is a container for all of the logic utilized in the template node.
   The implementation must provide a clean and high level interface to the core algorithm
*/
class SpiritComponentTemplate {
  public:
	/**
	 * @brief Constructor for SpiritComponentTemplate Class
	 * @param[in] nh ROS Nodehandle to publish and subscribe from
	 * @return Constructed object of type SpiritComponentTemplate
	 */
	SpiritComponentTemplate(ros::NodeHandle nh);

	/**
	 * @brief Primary work function in class, called in node file for this component
	 */
	void loop();

  private:
	//! ROS Nodehandle to publish and subscribe from
	ros::NodeHandle _nh;
};


#endif // SPIRIT_COMPONENT_TEMPLATE_H
