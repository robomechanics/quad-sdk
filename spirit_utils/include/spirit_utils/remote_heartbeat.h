#ifndef REMOTE_HEARTBEAT_H
#define REMOTE_HEARTBEAT_H

#include <ros/ros.h>
#include <spirit_utils/ros_utils.h>
#include <spirit_msgs/LegCommandArray.h>

//! A class for implementing a remote heartbeat
/*!
   RemoteHeartbeat publishes stamped messages at a fixed rate as a heartbeat
*/
class RemoteHeartbeat {
public:
	/**
	 * @brief Constructor for RemoteHeartbeat Class
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type RemoteHeartbeat
	 */
	RemoteHeartbeat(ros::NodeHandle nh);

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();

private:

	/**
	* @brief Callback function to handle new leg command data
	* @param[in] msg spirit_msgs<LegCommandArray> containing pos, vel and torque setpoints and gains
	*/
	void legControlCallback(const spirit_msgs::LegCommandArray::ConstPtr& msg);

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Subscriber for motor control messages
  	ros::Subscriber leg_control_sub_;

	/// ROS publisher for mocap data
	ros::Publisher heartbeat_pub_;

	/// Update rate for sending and receiving data
	double update_rate_;

	/// Update rate for sending and receiving data
	double latency_threshold_warn_;

	/// Update rate for sending and receiving data
	double latency_threshold_error_;
  
};

#endif // REMOTE_HEARTBEAT_H
