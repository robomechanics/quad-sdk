#ifndef CONTACTDETECTION_H
#define CONTACTDETECTION_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <spirit_msgs/ContactMode.h>

//! Contact detection class for spirit
/*!
  Contact detection module determines the probability of contact for each leg for state estimation and mpc control
*/
class ContactDetection {
public:
	/**
	 * @brief Constructor for ContactDetection Class
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type ContactDetection
	 */
	ContactDetection(ros::NodeHandle nh);

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();

private:
	/**
	 * @brief Callback function to handle new joint encoder data
	 * @param[in] joint_encoder_msg sensor_msgs<JointState> containing joint pos,vel,current
	 */
	void jointEncoderCallback(const sensor_msgs::JointState::ConstPtr& msg);

	/**
	 * @brief Callback function to handle new imu data
	 * @param[in] imu_msg sensors_msgs<Imu> containing new imu data
	 */
	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

	spirit_msgs::ContactMode updateStep();

	// ROS subscriber for joint encoder messages
	ros::Subscriber joint_encoder_sub_;

	// ROS subscriber for imu messages
	ros::Subscriber imu_sub_;

	// ROS Publisher for contact detection messages
	ros::Publisher contact_mode_pub_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data;
	double update_rate_;

	/// Last contact detection message
	spirit_msgs::ContactMode last_contact_est_;

	/// Most recent IMU callback (should be timestamped!)
 	sensor_msgs::Imu::ConstPtr last_imu_msg_;

	/// Most recent encoder callback (should be timestamped!)
	sensor_msgs::JointState::ConstPtr last_joint_state_msg_;

};

#endif // CONTACTDETECTION_H
