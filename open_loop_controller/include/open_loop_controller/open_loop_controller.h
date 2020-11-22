#ifndef OPEN_LOOP_CONTROLLER_H
#define OPEN_LOOP_CONTROLLER_H

#include <ros/ros.h>
#include <spirit_msgs/LegCommandArray.h>
#include <math.h>
#include <algorithm>


//! Implements open loop controller
/*!
   OpenLoopController implements all control logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
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
	std::pair<double,double> computeIk(std::pair<double,double> pt);

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

	// Vector of timestamps to hit each target_pt at
	std::vector<double> target_times_;
};


#endif // OPEN_LOOP_CONTROLLER_H
