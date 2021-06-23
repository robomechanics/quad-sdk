#ifndef REMOTE_HEARTBEAT_H
#define REMOTE_HEARTBEAT_H

#include <ros/ros.h>
#include <spirit_utils/ros_utils.h>

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

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

  /// ROS publisher for mocap data
  ros::Publisher heartbeat_pub_;

	/// Update rate for sending and receiving data
	double update_rate_;
  
};

#endif // REMOTE_HEARTBEAT_H
