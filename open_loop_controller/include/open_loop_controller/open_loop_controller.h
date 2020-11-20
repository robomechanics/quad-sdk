#ifndef OPEN_LOOP_CONTROLLER_H
#define OPEN_LOOP_CONTROLLER_H

#include <ros/ros.h>
#include <spirit_msgs/LegCommandArray.h>
#include <math.h>

//! Implements open loop controller
/*!
   OpenLoopController implements all control logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class OpenLoopController {
  public:
	/**
	 * @brief Constructor for OpenLoopController
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type OpenLoopController
	 */
	OpenLoopController(ros::NodeHandle nh);

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();

private:

	/**
	 * @brief Compute hip and knee positions to hit x y end effector pos
	 * @param[in] x target x position in shoulder frame
	 * @param[in] y target y position in shoulder frame
	 * @return pair of hip angle and knee angle
	 */
	std::pair<double,double> computeIk(double x, double y);

	/**
	 * @brief Compute and send open loop joint positions
	 * @param[in] elapsed_time Time since node began
	 */
  void sendJointPositions(double &elapsed_time);

	/// Publisher for desired joint positions
	ros::Publisher joint_control_pub_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data;
	double update_rate_;

	/// Time for one step cycle
	double t_cycle_;

	/// Robot mode (Walk or Stand)
	int mode_;
};


#endif // OPEN_LOOP_CONTROLLER_H
