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
	 * @brief Get instant legs' joint states
	 * @param[in] msg New joint state
	 */
	void jointStateCallback(const spirit_msgs::RobotState::ConstPtr& msg);

	/**
	 * @brief Compute open loop joint control
	 */
  	void computeJointControl();


	/**
	 * @brief Send open loop joint control
	 */
  	void publishJointControl();

	/**
   	* @brief Set up trajectory for each foot
    */
  	void setupTrajectory(Eigen::Vector3d init_point_);
	
	/**
   	* @brief Function to pre-process the body plan and robot state messages into Eigen arrays
   	*/
  	void getStateAndReferencePlan();

	/**
   	* @brief Function to compute the local plan
   	* @return Boolean if local plan was found successfully
   	*/
  	bool computeNextFlight();

	/**
   	* @brief Function to publish the local plan
    */
    void calculateNextPlan();

	/**
   	* @brief Update data after finishing a flight
	* @return check if the leg on flight has reached the ground  
    */
	bool finishFlight();

	/**
   	* @brief Check foot has reached target point
   	* @return Boolean if target point is reached
   	*/
	bool isReached(int leg_number_);

	/**
    * @brief Callback function to handle new plans
    * @param[in] msg Robot state trajectory message
    */
    void robotPlanCallback(const spirit_msgs::RobotPlan::ConstPtr& msg);

	/**
	 * @brief Send open loop joint control
	 * @param[in] input Describe variables going into the function (past by const ref if large)
	 * @param[out] output Describe variables coming out of the function
	 * @return Describe the return value of the function
	 */
 	 void doxygenExampleFunction(const std::vector<int> &input, std::vector<double> &output);


	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Subscriber for control mode
	ros::Subscriber control_mode_sub_;

	/// SUbscribe for joint state
	ros::Subscriber joint_state_sub_;


	/// Publisher for desired joint control
	ros::Publisher joint_control_pub_;

	/// Update rate for sending and receiving data;
	double update_rate_;

	/// Robot mode
	int control_mode_;

	/**
	 *  Flight mode: which foot is on the flight
	 *  All feet on the ground: 0
	 *  Front left: 1; Back left: 2; Front right: 3; Back right: 4
	*/
	int flight_mode;

	/**
	 * Next flight mode: from 1 to 4 
	*/
	int next_flight;

	/// Define ids for control modes: Sit
	const int SIT = 0;

	/// Define ids for control modes: Stand
	const int STAND = 1;

	/// Define ids for control modes: Walk
	const int STANCE = 2;

	const int WALK = 3;

	const int leg_length = 0.206;

	int cur_traj_track_seq;

	Eigen::MatrixXd traj;

	/// Joint control message
	spirit_msgs::LegCommandArray control_msg_;

	/// Number of legs
	const int num_legs_ = 4;

	/// one step horizen length
	const int N_ = 24;

	/// Standing joint angles
	std::vector<double> stand_joint_angles_;

	/// Joint state angles
	std::vector<double> joint_state_angles_;


	/// Matrix of continuous foot positions in body frame
  	Eigen::MatrixXd foot_positions_body_;

	/// Spirit Kinematics class
  	std::shared_ptr<spirit_utils::SpiritKinematics> kinematics_;

	/// Contact schedule
  	std::vector<std::vector<bool>> contact_schedule_;

	/// Matrix of body states (N x Nx: rows correspond to individual states in the horizon)
  	Eigen::MatrixXd body_plan_;

	/// PD gain when in standing mode
	std::vector<double> stand_kp_;
	std::vector<double> stand_kd_;

	ros::Time transition_timestamp_;

	const double transition_duration_ = 1.0;

	/// PD gain when foot is in stance
	std::vector<double> stance_kp_;
	std::vector<double> stance_kd_;
};


#endif // SILLY_WALK_TEMPLATE_H

