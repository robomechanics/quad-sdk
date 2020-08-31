#ifndef SPIRIT_CONTROLLER_H
#define SPIRIT_CONTROLLER_H

#include <ros/ros.h>

//! Implements online MPC for spirit
/*!
   SpiritController implements all control logic for spirit. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class SpiritController {
  public:
	/**
	 * @brief Constructor for SpiritController
	 * @param[in] nh ROS Nodehandle to publish and subscribe from
	 * @return Constructed object of type SpiritController
	 */
	SpiritController(ros::NodeHandle nh);

	/**
	 * @brief Primary work function in class, called in node file for this component
	 */
	void update();

  private:
	//! ROS Nodehandle to publish and subscribe from
	ros::NodeHandle _nh;
};


#endif // SPIRIT_CONTROLLER_H
