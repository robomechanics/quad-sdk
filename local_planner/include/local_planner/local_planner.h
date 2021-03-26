#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <ros/ros.h>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <spirit_msgs/BodyPlan.h>
#include <spirit_msgs/MultiFootPlanDiscrete.h>
#include <spirit_msgs/GRFArray.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/RobotStateTrajectory.h>
#include <local_planner/quadruped_mpc.h>
#include <local_planner/local_body_planner.h>
#include <local_planner/local_footstep_planner.h>
#include <spirit_utils/ros_utils.h>
#include <spirit_utils/kinematics.h>
#include "spirit_utils/matplotlibcpp.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//! Local Body Planner library
/*!
   Wrapper around Quadrupedal MPC that interfaces with our ROS architecture
*/
class LocalPlanner {
  public:
	/**
	 * @brief Constructor for LocalPlanner
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type LocalPlanner
	 */
	LocalPlanner(ros::NodeHandle nh);
  
private:
	/**
   * @brief Callback function to handle new plans
   * @param[in] msg Robot state trajectory message
   */
  void globalPlanCallback(const spirit_msgs::RobotStateTrajectory::ConstPtr& msg);

  /**
   * @brief Callback function to handle new state estimates
   * @param[in] State estimate message contining position and velocity for each joint and robot body
   */
  void robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg);
  
  Eigen::VectorXd state_msg_to_eigen(spirit_msgs::RobotState robot_state, bool zero_vel=false);


  /**
   * @brief Function to compute the local plan
   */
  void computeLocalPlan();

  /**
   * @brief Function to publish the local plan
   */
  void publishLocalPlan();

	/// ROS subscriber for incoming global plans
	ros::Subscriber global_plan_sub_;

  /// ROS Subscriber for incoming states
  ros::Subscriber robot_state_sub_;

	/// ROS publisher for local plan output
	ros::Publisher local_plan_pub_;

	/// Define map frame
	std::string map_frame_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data;
	double update_rate_;

  /// Local Body Planner object
  std::shared_ptr<LocalBodyPlanner> local_body_planner_;

  /// Local Footstep Planner object
  std::shared_ptr<LocalFootstepPlanner> local_footstep_planner_;

	/// Most recent robot plan
	spirit_msgs::BodyPlan::ConstPtr global_plan_msg_;

  /// Current state (ground truth or estimate)
  Eigen::VectorXd current_state_;

  /// Minimum normal force in contact phase
  double normal_lo_;

  /// Maximum normal force in contact phase
  double normal_hi_;

  /// Number of iterations between body and footstep planners
  int iterations_

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

  /// Matrix of body states (N x Nx: rows correspond to individual states in the horizon)
  quadruped_mpc::StateTraj body_plan_;

  /// Matrix of grfs (N x Nu: rows correspond to individual arrays of GRFs in the horizon)
  quadruped_mpc::ControlTraj grf_plan_; 

  /// Matrix of continuous foot positions (N x Nu: rows correspond to foot states in the horizon)
  quadruped_mpc::FootTraj foot_positions_;
  
  /// Matrix of continuous foot positions projected underneath the hips
  quadruped_mpc::FootTraj hip_projected_foot_positions_;

  /// Matrix of foot contact locations (number of contacts x num_legs_)
  Eigen::MatrixXd foot_plan_discrete_;

  /// Spirit Kinematics class
  std::shared_ptr<spirit_utils::SpiritKinematics> kinematics_;

};


#endif // LOCAL_PLANNER_H
