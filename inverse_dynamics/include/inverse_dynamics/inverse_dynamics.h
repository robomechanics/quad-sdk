#ifndef INVERSE_DYNAMICS_H
#define INVERSE_DYNAMICS_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <spirit_utils/ros_utils.h>
#include <spirit_utils/foot_jacobians.h>
#include <spirit_msgs/ControlInput.h>
#include <std_msgs/UInt8.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/MotorCommand.h>
#include <spirit_msgs/LegCommand.h>
#include <spirit_msgs/LegCommandArray.h>
#include <spirit_msgs/MultiFootPlanContinuous.h>
// #include <spirit_msgs/FootstepPlan.h>

#include <cmath>
#define MATH_PI 3.141592


//! Implements inverse dynamics
/*!
   inverseDynamics implements inverse dynamics logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class inverseDynamics {
  public:
	/**
	 * @brief Constructor for inverseDynamics
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type inverseDynamics
	 */
	inverseDynamics(ros::NodeHandle nh);
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
	// void controlInputCallback(const spirit_msgs::ControlInput::ConstPtr& msg);
	/**
	 * @brief Callback function to handle new control input (GRF)
	 * @param[in] Control input message contining ground reaction forces and maybe nominal leg positions
	 */
	void robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg);
		/**
	 * @brief Callback function to handle new control input (GRF)
	 * @param[in] Control input message contining ground reaction forces and maybe nominal leg positions
	 */
	void footPlanContinuousCallback(const spirit_msgs::MultiFootPlanContinuous::ConstPtr& msg);
		/**
	 * @brief Callback function to handle new control input (GRF)
	 * @param[in] Control input message contining ground reaction forces and maybe nominal leg positions
	 */
	void publishLegCommandArray();

	/// Subscriber for control mode
	ros::Subscriber control_mode_sub_;

	/// ROS subscriber for control input
	// ros::Subscriber control_input_sub_;

	/// ROS subscriber for state estimate
	ros::Subscriber robot_state_sub_;

	/// ROS subscriber for foot plan
	ros::Subscriber foot_plan_continuous_sub_;

	/// ROS publisher for inverse dynamics
	ros::Publisher leg_command_array_pub_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data;
	double update_rate_;

	/// Robot mode (Stand 0, ID Control)
	int control_mode_;

	/// Most recent control input
	// spirit_msgs::ControlInput last_control_input_msg_;

	/// Most recent state estimate
	spirit_msgs::RobotState last_robot_state_msg_;

	/// Most recent swing leg plan
	spirit_msgs::MultiFootPlanContinuous last_foot_plan_continuous_msg_;
	
};


#endif // MPC_CONTROLLER_H
