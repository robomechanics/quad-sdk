#ifndef TAIL_CONTROLLER_H
#define TAIL_CONTROLLER_H

#include <ros/ros.h>
#include <spirit_msgs/LegCommandArray.h>
#include <spirit_utils/ros_utils.h>
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
class TailController {
  public:
	/**
	 * @brief Constructor for OpenLoopController
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type OpenLoopController
	 */
	TailController(ros::NodeHandle nh);

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();

private:
	/**
	 * @brief Verifies and updates new control mode
	 * @param[in] msg New control mode
	 */ 
	// void controlModeCallback(const std_msgs::UInt8::ConstPtr& msg);

	/**
	 * @brief Setup open loop trajectory in shoulder space
	 */ 
	void setupTrajectory();
	
	/**
	 * @brief Compute hip and knee positions to hit x y end effector pos
	 * @param[in] x Target x position in shoulder space 
	 * @param[in] y Target y position in shoulder space
	 * @return pair of hip angle and knee angle
	 */
	// std::pair<double,double> compute2DIk(double x, double y);

	/**
	 * @brief Compute and send open loop joint positions
	 * @param[in] elapsed_time Time since node began
	 */
  void sendJointPositions(double &elapsed_time);

	/// Publisher for desired joint positions
	ros::Publisher joint_control_pub_;

	/// Subscriber for control mode
	ros::Subscriber control_mode_sub_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data;
	double update_rate_;

	/// Time for one step cycle
	double t_cycle_;

	/// Robot mode (Sit 0, Walk 1 or Stand 2)
	int control_mode_;

	/// Timestep to interpolate points at
	double interp_dt_;

	/// Target points to hit (hip angle, knee angle)
	std::vector<std::pair<double,double>> target_pts_;

	std::vector<std::pair<double,double>> target_torque_;

	/// Vector of timestamps to hit each target_pt at
	std::vector<double> target_times_;

	/// Joint angles in stand config
	std::vector<double> stand_joint_angles_;

	/// Stand proportional gain for each joint
	std::vector<double> stand_kp_;

	/// Stand derivative gain for each joint
	std::vector<double> stand_kd_;

	/// Walk proportional gain for each joint
	std::vector<double> walk_kp_;

	/// Walk derivative gain for each joint
	std::vector<double> walk_kd_;

	/// Gait phase info for each leg
	std::vector<double> leg_phases_;

	/// Numerically differentiate trajectory for velocity command
	bool use_diff_for_velocity_;

	bool track_traj_;
};


#endif // TAIL_CONTROLLER_H
