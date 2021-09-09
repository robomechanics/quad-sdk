#ifndef TAIL_CONTROLLER_H
#define TAIL_CONTROLLER_H

#include <ros/ros.h>
#include <spirit_msgs/LegCommandArray.h>
#include <spirit_utils/tail_type.h>
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
class TailController
{
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
	void tailPlanCallback(const spirit_msgs::LegCommandArray::ConstPtr &msg);

	void robotStateCallback(const spirit_msgs::RobotState::ConstPtr &msg);

	/**
	 * @brief Compute and send open loop joint positions
	 * @param[in] elapsed_time Time since node began
	 */
	void publishTailCommand();

	ros::NodeHandle nh_;

	double update_rate_;

	double roll_kp_;

	double roll_kd_;

	double pitch_kp_;

	double pitch_kd_;

	ros::Publisher tail_control_pub_;

	ros::Subscriber tail_plan_sub_;

	ros::Subscriber robot_state_sub_;

	spirit_msgs::RobotState::ConstPtr robot_state_msg_;

	spirit_msgs::LegCommandArray::ConstPtr last_tail_plan_msg_;

	double dt_;

	int tail_type_;

	Eigen::VectorXd tail_current_state_;

	std::string param_ns_;

	Eigen::VectorXd current_state_;
};

#endif // TAIL_CONTROLLER_H
