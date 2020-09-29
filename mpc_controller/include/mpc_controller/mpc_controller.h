#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

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

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();
  
private:

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data;
	double update_rate_;
};


#endif // MPC_CONTROLLER_H
