#ifndef REMOTE_HEARTBEAT_H
#define REMOTE_HEARTBEAT_H

#include <quad_msgs/LegCommandArray.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>

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
   * @brief Callback function to handle new robot heartbeat
   * @param[in] msg header containing robot heartbeat
   */
  void robotHeartbeatCallback(const std_msgs::Header::ConstPtr& msg);

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Subscriber for robot heartbeat messages
  ros::Subscriber robot_heartbeat_sub_;

  /// ROS publisher for remote heartbeat messages
  ros::Publisher remote_heartbeat_pub_;

  /// Update rate for sending and receiving data
  double update_rate_;

  /// Latency threshold on robot messages for warnings (s)
  double robot_latency_threshold_warn_;

  /// Latency threshold on robot messages for error (s)
  double robot_latency_threshold_error_;
};

#endif  // REMOTE_HEARTBEAT_H
