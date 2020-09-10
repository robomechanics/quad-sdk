#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <ros/ros.h>

//! Implements online MPC
/*!
   MPCController implements all control logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class MPCController {
  public:
	/**
	 * @brief Constructor for MPCController
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type MPCController
	 */
	MPCController(ros::NodeHandle nh);

private:

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;
};


#endif // MPC_CONTROLLER_H
