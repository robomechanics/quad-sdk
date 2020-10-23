#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <spirit_msgs/ControlInput.h>

//! A class for interfacing between stand-in robot data and other spirit-software nodes and topics.
/*!
   RobotInterface contains the approximation for the real spirit robot. Its purposes are purely for development.
   The class provides publishers for robot data (encoder and imu measurements) as well as a subscriber for control inputs.
*/
class RobotInterface {
public:
	/**
	 * @brief Constructor for RobotInterface Class
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type RobotInterface
	 */
	RobotInterface(ros::NodeHandle nh);

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();

private:
	/**
   * @brief Callback function to handle new control inputs
   * @param[in] Joint state message contining desired position, velocity, and torque for each joint
   */
  void controlInputCallback(const spirit_msgs::ControlInput::ConstPtr& msg);

  /**
   * @brief Function to publish joint encoder data. Likely empty.
   */
  void publishJointEncoders();

  /**
   * @brief Function to publish imu data. Likely empty.
   */
  void publishImu();

	/// ROS subscriber for the control inputs
	ros::Subscriber control_input_sub_;

	/// ROS publisher for joint encoder data
	ros::Publisher joint_encoder_pub_;

	/// ROS publisher for imu data
	ros::Publisher imu_pub_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data
	double update_rate_;
};

#endif // ROBOT_INTERFACE_H
