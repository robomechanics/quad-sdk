#ifndef OPEN_LOOP_CONTROLLER_H
#define OPEN_LOOP_CONTROLLER_H

#include <ros/ros.h>
#include <spirit_msgs/LegCommandArray.h>
#include <spirit_utils/ros_utils.h>
#include <spirit_utils/kinematics.h>
#include <std_msgs/UInt8.h>
#include <math.h>
#include <algorithm>

//! Implements open loop controller
/*!
   OpenLoopController implements a simple, easily configurable open loop 
   controller. It transmits desired positions, velocities, feedforward
   torques and gains to the low level controller. Trajectories are
   specified via waypoints and waypoint transition durations.
*/
class OpenLoopController
{
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
	 * @brief Verifies and updates new control mode
	 * @param[in] msg New control mode
	 */
	void controlModeCallback(const std_msgs::UInt8::ConstPtr &msg);

	/**
	 * @brief Callback function to handle current robot state
	 * @param[in] msg input message contining current robot state
	 */
	void robotStateCallback(const spirit_msgs::RobotState::ConstPtr &msg);

	/**
	 * @brief Callback function to handle new desired twist data when using twist input
	 * @param[in] msg the message contining twist data
	 */
	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

	/**
	 * @brief Compute and send open loop joint positions
	 * @param[in] elapsed_time Time since node began
	 */
	void sendJointPositions(double &elapsed_time);

	/// Publisher for desired joint positions
	ros::Publisher joint_control_pub_;

	/// Subscriber for control mode
	ros::Subscriber control_mode_sub_;

	/// ROS subscriber for state estimate
	ros::Subscriber robot_state_sub_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Most recent state estimate
	spirit_msgs::RobotState::ConstPtr last_robot_state_msg_;

	/// Update rate for sending and receiving data;
	double update_rate_;

	/// Time for one step cycle
	double t_cycle_;

	/// Robot mode (Sit 0, Walk 1 or Stand 2)
	int control_mode_;

	/// Timestep to interpolate points at
	double interp_dt_;

	/// Target points to hit (hip angle, knee angle)
	std::vector<std::pair<double, double>> target_pts_;

	/// Vector of timestamps to hit each target_pt at
	std::vector<double> target_times_;

	/// Joint angles in stand config
	std::vector<double> stand_joint_angles_;

	/// Joint angles in stand config
	std::vector<double> sit_joint_angles_;

	/// Stand proportional gain for each joint
	std::vector<double> stand_kp_;

	/// Stand derivative gain for each joint
	std::vector<double> stand_kd_;

	/// Walk proportional gain for each joint
	std::vector<double> swing_kp_;

	/// Walk derivative gain for each joint
	std::vector<double> swing_kd_;

	/// Walk proportional gain for each joint
	std::vector<double> stance_kp_;

	/// Walk derivative gain for each joint
	std::vector<double> stance_kd_;

	/// Gait phase info for each leg
	std::vector<double> leg_phases_;

	/// Numerically differentiate trajectory for velocity command
	bool use_diff_for_velocity_;

	std::vector<double> x_;

	std::vector<double> y_;

	/// Spirit Kinematics class
	std::shared_ptr<spirit_utils::SpiritKinematics> kinematics_;

	/// Subscriber for twist input messages
	ros::Subscriber cmd_vel_sub_;

	std::vector<double> ff_force_;

	double abad_scale_;

	double abad_const_;

	double hip_scale_;

	double hip_const_;

	double knee_scale_;

	double knee_const_;

	double alpha_;

	double beta_;
};

#endif // OPEN_LOOP_CONTROLLER_H
