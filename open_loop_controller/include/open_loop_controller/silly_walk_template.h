#ifndef SILLY_WALK_TEMPLATE_H
#define SILLY_WALK_TEMPLATE_H

#include <ros/ros.h>
#include <spirit_msgs/LegCommandArray.h>
#include <spirit_utils/ros_utils.h>
#include <std_msgs/UInt8.h>
#include <math.h>
#include <algorithm>

//! A Silly Walk class. Brief class description goes here.
/*!
   SillyWalkController implements a silly walk. Put a detailed description of your class here.
*/
class SillyWalkController {
  public:
	/**
	 * @brief Constructor for SillyWalkController
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type SillyWalkController
	 */
	SillyWalkController(ros::NodeHandle nh);

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();

private:
	/**
	 * @brief Verifies and updates new control mode
	 * @param[in] msg New control mode
	 */ 
	void controlModeCallback(const std_msgs::UInt8::ConstPtr& msg);

	/**
	 * @brief Compute open loop joint control
	 */
  void computeJointControl();

	/**
	 * @brief Send open loop joint control
	 */
  void publishJointControl();

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Subscriber for control mode
	ros::Subscriber control_mode_sub_;

	/// Publisher for desired joint control
	ros::Publisher joint_control_pub_;

	/// Update rate for sending and receiving data;
	double update_rate_;

	/// Robot mode
	int control_mode_;

	/// Define ids for control modes: Sit
	const int SIT = 0;

	/// Define ids for control modes: Stand
	const int STAND = 1;

	/// Joint control message
	spirit_msgs::LegCommandArray control_msg_;

	/// Number of legs
	const int num_legs_ = 4;
};


#endif // SILLY_WALK_TEMPLATE_H
