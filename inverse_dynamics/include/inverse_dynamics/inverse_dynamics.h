#ifndef INVERSE_DYNAMICS_H
#define INVERSE_DYNAMICS_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
// #include <Eigen/Dense>
#include <spirit_utils/ros_utils.h>
#include <spirit_utils/foot_jacobians.h>
#include <spirit_utils/math_utils.h>
#include <spirit_utils/ros_utils.h>
#include <spirit_msgs/GRFArray.h>
#include <std_msgs/UInt8.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/MotorCommand.h>
#include <spirit_msgs/LegCommand.h>
#include <spirit_msgs/LegCommandArray.h>
#include <spirit_msgs/MultiFootPlanContinuous.h>
#include <eigen_conversions/eigen_msg.h>
#include "spirit_utils/matplotlibcpp.h"


#include <cmath>
#define MATH_PI 3.141592


//! Implements inverse dynamics
/*!
   InverseDynamics implements inverse dynamics logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class InverseDynamics {
  public:
	/**
	 * @brief Constructor for InverseDynamics
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type InverseDynamics
	 */
	InverseDynamics(ros::NodeHandle nh);
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
	 * @brief Callback function to handle new control input (GRF)
	 * @param[in] Control input message contining ground reaction forces and maybe nominal leg positions
	 */
	void grfInputCallback(const spirit_msgs::GRFArray::ConstPtr& msg);
	/**
	 * @brief Callback function to handle new control input (GRF)
	 * @param[in] Control input message contining ground reaction forces and maybe nominal leg positions
	 */
	void robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg);
		/**
	 * @brief Callback function to handle new control input (GRF)
	 * @param[in] Control input message contining ground reaction forces and maybe nominal leg positions
	 */
	void trajectoryCallback(const spirit_msgs::RobotState::ConstPtr& msg);
		/**
	 * @brief Callback function to handle new control input (GRF)
	 * @param[in] Control input message contining ground reaction forces and maybe nominal leg positions
	 */
	void publishLegCommandArray();

	/// Subscriber for control mode
	ros::Subscriber control_mode_sub_;

	/// ROS subscriber for control input
	ros::Subscriber grf_input_sub_;

	/// ROS subscriber for state estimate
	ros::Subscriber robot_state_sub_;

	/// ROS subscriber for trajectory
	ros::Subscriber trajectory_sub_;

	/// ROS publisher for inverse dynamics
	ros::Publisher leg_command_array_pub_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data;
	double update_rate_;

	/// Robot mode
	int control_mode_;

	/// Define ids for control modes: Sit
	const int SIT = 0;

	/// Define ids for control modes: Stand
	const int STAND = 1;

	/// Define ids for control modes: Sit to stand
	const int SIT_TO_STAND = 2;

	/// Define ids for control modes: Stand to sit
	const int STAND_TO_SIT = 3;
	
	/// Most recent control input
	spirit_msgs::GRFArray last_grf_input_msg_;

	/// Most recent state estimate
	spirit_msgs::RobotState last_robot_state_msg_;

	/// Most recent trajectory state
	spirit_msgs::RobotState last_trajectory_msg_;

	/// Duration for sit to stand behavior
	const double transition_duration_ = 2.0;
	
	/// Time at which to start transition
	ros::Time transition_timestamp_;

	std::vector<double> f0x;
	std::vector<double> f1x;
	std::vector<double> f2x;
	std::vector<double> f3x;
	std::vector<double> f0y;
	std::vector<double> f1y;
	std::vector<double> f2y;
	std::vector<double> f3y;
	std::vector<double> f0z;
	std::vector<double> f1z;
	std::vector<double> f2z;
	std::vector<double> f3z;

	std::vector<double> f0xJ;
	std::vector<double> f1xJ;
	std::vector<double> f2xJ;
	std::vector<double> f3xJ;
	std::vector<double> f0yJ;
	std::vector<double> f1yJ;
	std::vector<double> f2yJ;
	std::vector<double> f3yJ;
	std::vector<double> f0zJ;
	std::vector<double> f1zJ;
	std::vector<double> f2zJ;
	std::vector<double> f3zJ;
	
	std::vector<double> counterVec;

	double step_number;
	
};


#endif // MPC_CONTROLLER_H
