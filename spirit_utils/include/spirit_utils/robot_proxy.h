#ifndef ROBOT_PROXY_H
#define ROBOT_PROXY_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <spirit_msgs/GRFArray.h>

//! A class for interfacing between dummy robot data and other spirit-software nodes and topics.
/*!
   RobotProxy contains the approximation for the real spirit robot. Its purposes are purely for development.
   The class provides publishers for dummy robot data (encoder, imu, and mocap measurements) as well as a subscriber for control inputs.
*/
class RobotProxy {
public:
	/**
	 * @brief Constructor for RobotProxy Class
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type RobotProxy
	 */
	RobotProxy(ros::NodeHandle nh);

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();

private:
	/**
   * @brief Callback function to handle new control inputs
   * @param[in] Joint state message contining desired position, velocity, and torque for each joint
   */
  void grfArrayCallback(const spirit_msgs::GRFArray::ConstPtr& msg);

  /**
   * @brief Function to publish dummy joint encoder data
   */
  void publishJointEncoders();

  /**
   * @brief Function to publish dummy imu data
   */
  void publishImu();

  /**
   * @brief Function to publish dummy mocap data
   */
  void publishMocap();

	/// ROS subscriber for the control inputs
	ros::Subscriber control_input_sub_;

	/// ROS publisher for joint encoder data
	ros::Publisher joint_encoder_pub_;

  /// ROS publisher for imu data
  ros::Publisher imu_pub_;

  /// ROS publisher for twist data
  ros::Publisher twist_pub_;

  /// ROS publisher for mocap data
  ros::Publisher mocap_pub_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data
	double update_rate_;

  /// Time of initialization
  double t_init_;
  
};

#endif // ROBOT_PROXY_H
