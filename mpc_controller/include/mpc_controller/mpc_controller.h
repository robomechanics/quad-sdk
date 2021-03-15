#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <ros/ros.h>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <spirit_msgs/BodyPlan.h>
#include <spirit_msgs/MultiFootPlanDiscrete.h>
#include <spirit_msgs/GRFArray.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/RobotStateTrajectory.h>
#include <mpc_controller/quadruped_mpc.h>
#include <spirit_utils/ros_utils.h>
#include <spirit_utils/kinematics.h>
#include "spirit_utils/matplotlibcpp.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//! MPC controller ROS node
/*!
   Wrapper around Quadrupedal MPC that interfaces with our ROS architecture
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
   * @brief Callback function to handle new plans
   * @param[in] msg Robot state trajectory message
   */
  void robotPlanCallback(const spirit_msgs::RobotStateTrajectory::ConstPtr& msg);

  /**
   * @brief Callback function to handle new state estimates
   * @param[in] State estimate message contining position and velocity for each joint and robot body
   */
  void robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg);

  
  Eigen::VectorXd state_to_eigen(spirit_msgs::RobotState robot_state, bool zero_vel=false);

  /**
   * @brief Internal function to convert robot state trajectory into MPC useful
   * @param[out] start_idx Index into plan corresponding to start of MPC trajectory
   * @param[out] contact_sequences Vector of boolean vectors declaring which feet are on the ground at each step
   * @param[out] foot_positions N x 12 matrix of foot positions in body frame (x0,y0,z0,x1,..)
   * @param[out] ref_traj (N+1) x Nx matrix of state reference trajectory
   **/
  void extractMPCTrajectory(int start_idx,
                            std::vector<std::vector<bool>> &contact_sequences,
                            Eigen::MatrixXd &foot_positions, 
                            Eigen::MatrixXd &ref_traj);

  /**
   * @brief Function to publish commanded grfs
   */
  void publishGRFArray();

	/// ROS subscriber for incoming plans
	ros::Subscriber robot_state_traj_sub_;

  /// ROS Subscriber for incoming states
  ros::Subscriber robot_state_sub_;

	/// ROS publisher for control input
	ros::Publisher grf_array_pub_;

	/// Define map frame
	std::string map_frame_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data;
	double update_rate_;

  /// Quadruped MPC object
  std::shared_ptr<QuadrupedMPC> quad_mpc_;

	/// Most recent robot plan
	spirit_msgs::RobotStateTrajectory::ConstPtr last_plan_msg_;

  /// Current state (ground truth or estimate)
  Eigen::VectorXd cur_state_;

  /// Minimum normal force in contact phase
  double normal_lo_;

  /// Maximum normal force in contact phase
  double normal_hi_;

  /// MPC and trajectory publisher timestep (seconds)
  double dt_;

  /// MPC Horizon length
  int N_;

  /// Number of states
  const int Nx_ = 12;

  /// Number of controls
  const int Nu_ = 13;

  /// Number of legs
  const int num_legs_ = 4;

  /// Number of joints per leg
  const int num_joints_per_leg_ = 3;

  /// Spirit Kinematics class
  std::shared_ptr<spirit_utils::SpiritKinematics> kinematics_;


};


#endif // MPC_CONTROLLER_H
