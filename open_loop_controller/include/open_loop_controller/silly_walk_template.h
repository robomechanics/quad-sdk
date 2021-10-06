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
   SillyWalkTemplate implements a silly walk. Put a detailed description of your class here.
*/
class SillyWalkTemplate {
  public:
	/**
	 * @brief Constructor for SillyWalkTemplate
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type SillyWalkTemplate
	 */
	SillyWalkTemplate(ros::NodeHandle nh);

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

	/**
	 * @brief Send open loop joint control
	 * @param[in] input Describe variables going into the function (past by const ref if large)
	 * @param[out] output Describe variables coming out of the function
	 * @return Describe the return value of the function
	 */
  void doxygenExampleFunction(const std::vector<int> &input, std::vector<double> &output);

  	/**
	 * @brief Callback function to handle current robot state
	 * @param[in] msg input message contining current robot state
	 */
	void robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg);

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
	const int STANDUP = 1;
	const int STAND = 100;

	/// Define ids for control modes: Walk
	const int WALK_1 = 2;
	const int WALK_2 = 3;

	/// Joint control message
	spirit_msgs::LegCommandArray control_msg_;

	/// Number of legs
	const int num_legs_ = 4;

	/// Standing joint angles
	std::vector<double> stand_joint_angles_;

	/// Sitting joint angles
	std::vector<double> sit_joint_angles_;

	/// Walking joint angles
	std::vector<double> swing_joint_angles_;
	std::vector<double> td_joint_angles_;

	// Use a kinematics object to make kinematics calculations easier
	spirit_utils::SpiritKinematics kinematics_;

	/// Duration for sit to stand behavior
	const double transition_duration_ = 1.0;

	/// Time at which to start transition
	ros::Time transition_timestamp_;

	// State timeout threshold in seconds
	double last_state_time_;
};


#endif // SILLY_WALK_TEMPLATE_H
