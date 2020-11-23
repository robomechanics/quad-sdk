#ifndef OPEN_LOOP_CONTROLLER_H
#define OPEN_LOOP_CONTROLLER_H

#include <ros/ros.h>
#include <spirit_msgs/LegCommandArray.h>
#include <spirit_utils/ros_utils.h>
#include <math.h>
#include <algorithm>

//! Implements open loop controller
/*!
   OpenLoopController implements a simple, easily configurable open loop 
   controller. It transmits desired positions, velocities, feedforward
   torques and gains to the low level controller. Trajectories are
   specified via waypoints and waypoint transition durations.
*/
class OpenLoopController {
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
	 * @brief Setup open loop trajectory in shoulder space
	 */ 
	void setupTrajectory();
	
	/**
	 * @brief Compute hip and knee positions to hit x y end effector pos
	 * @param[in] pt Target x y position as a pair (shoulder frame)
	 * @return pair of hip angle and knee angle
	 */
	std::pair<double,double> compute2DIk(std::pair<double,double> pt);

	/**
	 * @brief Compute and send open loop joint positions
	 * @param[in] elapsed_time Time since node began
	 */
  void sendJointPositions(double &elapsed_time);

	/// Publisher for desired joint positions
	ros::Publisher joint_control_pub_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data;
	double update_rate_;

	/// Time for one step cycle
	double t_cycle_;

	/// Robot mode (Walk or Stand)
	int mode_;

	/// Target points to hit (x,y) w/ x, y in shoulder space)
	std::vector<std::pair<double,double>> target_pts_;

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
};


#endif // OPEN_LOOP_CONTROLLER_H
