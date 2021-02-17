#ifndef INVERSE_DYNAMICS_H
#define INVERSE_DYNAMICS_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <spirit_utils/ros_utils.h>
#include <spirit_utils/foot_jacobians.h>
#include <spirit_msgs/ControlInput.h>
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
	// void swingLegPlanCallback(const spirit_msgs::SwingLegPlan::ConstPtr& msg);
		/**
	 * @brief Callback function to handle new control input (GRF)
	 * @param[in] Control input message contining ground reaction forces and maybe nominal leg positions
	 */
	// void footStepPlanCallback(const spirit_msgs::FootstepPlan::ConstPtr& msg);
		/**
	 * @brief Function to handle new control input (GRF)
	 * @param[in] Control input message contining ground reaction forces and maybe nominal leg positions
	 */
	void publishLegCommandArray();

	/// ROS subscriber for control input
	// ros::Subscriber control_input_sub_;

	/// ROS subscriber for state estimate
	ros::Subscriber state_estimate_sub_;

	/// ROS subscriber for Swing Leg Plan
	// ros::Subscriber swing_leg_plan_sub_;

	/// ROS subscriber for Swing Leg Plan
	// ros::Subscriber foot_step_plan_sub_;

	/// ROS publisher for inverse dynamics
	ros::Publisher leg_command_array_pub_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data;
	double update_rate_;

	/// Most recent control input
	// spirit_msgs::ControlInput last_control_input_msg_;

	/// Most recent state estimate
	spirit_msgs::RobotState last_state_estimate_msg_;

	/// Most recent swing leg plan
	// spirit_msgs::SwingLegPlan last_swing_leg_plan_msg_;

	/// Most recent foot step plan
	// spirit_msgs::FootstepPlan last_foot_step_plan_msg_;
	
};


#endif // MPC_CONTROLLER_H
