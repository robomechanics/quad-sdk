#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <mpcplusplus/mpcplusplus.h>
#include <spirit_msgs/BodyPlan.h>
#include <spirit_msgs/MultiFootPlanDiscrete.h>
#include <spirit_msgs/ControlInput.h>
#include <spirit_msgs/RobotState.h>

//! Implements online MPC
/*!
   MPCController implements all control logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class MPCController {
  public:
	/**
	 * @brief Constructor for MPCController
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type MPCController
	 */
	MPCController(ros::NodeHandle nh);

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();
  
private:
	/**
   * @brief Callback function to handle new state estimates
   * @param[in] State estimate message contining position and velocity for each joint and robot body
   */
  void robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg);
	/**
   * @brief Callback function to handle new footstep plan data
   * @param[in] MultiFootPlanDiscrete message contining foothold data for each foot
   */
  void footPlanDiscreteCallback(const spirit_msgs::MultiFootPlanDiscrete::ConstPtr& msg);
    /**
     * @brief Callback function to handle new body plan data
     * @param[in] nav_msgs::Path contining map data
     */
  void bodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg);
    /**
     * @brief Callback function to handle new discrete body plan data
     * @param[in] nav_msgs::Path contining map data
     */
  void discreteBodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg);

  /**
   * @brief Function to publish imu data. Likely empty.
   */
  void publishControlInput();

	/// ROS subscriber for the state estimate
	ros::Subscriber robot_state_sub_;

	/// ROS subscriber for the footstep plan
	ros::Subscriber footstep_plan_sub_;

	/// ROS subscriber for the body plan
	ros::Subscriber body_plan_sub_;

	/// ROS subscriber for the discrete body plan
	ros::Subscriber discrete_body_plan_sub_;

	/// ROS publisher for control input
	ros::Publisher control_input_pub_;

	/// Define map frame
	std::string map_frame_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data;
	double update_rate_;

  /// Linear MPC object
  std::shared_ptr<mpcplusplus::LinearMPC> mpc;
};


#endif // MPC_CONTROLLER_H
