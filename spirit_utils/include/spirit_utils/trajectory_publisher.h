#ifndef TRAJECTORY_PUBLISHER_H
#define TRAJECTORY_PUBLISHER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <spirit_msgs/BodyPlan.h>
#include <spirit_msgs/MultiFootPlanContinuous.h>
#include <spirit_msgs/FootState.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/RobotStateTrajectory.h>
#include "spirit_utils/math_utils.h"
#include "spirit_utils/ros_utils.h"
#include "spirit_utils/kinematics.h"
#include "spirit_utils/function_timer.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>

//! A class for interfacing between RViz and spirit-software topics.
/*!
   TrajectoryPublisher is a container for all of the logic utilized in the 
   template node. The implementation must provide a clean and high level 
   interface to the core algorithm
*/
class TrajectoryPublisher {
public:
  /**
   * @brief Constructor for TrajectoryPublisher Class
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type TrajectoryPublisher
   */
  TrajectoryPublisher(ros::NodeHandle nh);

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

private:
  /**
   * @brief Import trajectory from external source
   */
  void importTrajectory();

  /**
   * @brief Callback function to handle new body plan data
   * @param[in] Body plan message contining interpolated output of body planner
   */
  void bodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg);

  /**
   * @brief Callback function to handle new robot state data
   * @param[in] msg message contining robot state information
   */
  void robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg);

  /**
   * @brief Callback function to handle new continuous foot plan data
   * @param[in] MultiFootPlanContinuous message containing foot plan data
   */
  void multiFootPlanContinuousCallback(
    const spirit_msgs::MultiFootPlanContinuous::ConstPtr& msg);

  /**
   * @brief Update the current trajectory
   */
  void updateTrajectory();

  /**
   * @brief Publish the current trajectory
   */
  void publishTrajectory();

  /**
   * @brief Publish the current trajectory state
   */
  void publishTrajectoryState();

  /// ROS Subscriber for the body plan
  ros::Subscriber body_plan_sub_;

  /// ROS subscriber for the swing leg plan
  ros::Subscriber multi_foot_plan_continuous_sub_;

  /// ROS subscriber for the robot state data
  ros::Subscriber ground_truth_state_sub_;

  /// ROS Publisher for the current trajectory state
  ros::Publisher trajectory_state_pub_;

  /// ROS Publisher for the entire trajectory
  ros::Publisher trajectory_pub_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Vector of body states to store the body plan
  spirit_msgs::BodyPlan body_plan_msg_;

  /// Vector of body states to store the body plan
  spirit_msgs::MultiFootPlanContinuous multi_foot_plan_continuous_msg_;

  /// Vector of times corresponding to the trajectory states
  std::vector<double> t_traj_;

  /// Message for robot trajectory
  spirit_msgs::RobotStateTrajectory traj_msg_;

  /// Robot state message
  spirit_msgs::RobotState::ConstPtr robot_state_msg_;

  /// Update rate for sending and receiving data
  double update_rate_;

  /// Flag to update the trajectory
  bool update_flag_;

  /// Timestep for trajectory interpolation
  double interp_dt_;

  /// Handle for the map frame
  std::string map_frame_;

  /// Playback speed for trajectory state publishing
  double playback_speed_;

  /// The source of the current trajectory (import or otherwise)
  std::string traj_source_;

};

#endif // TRAJECTORY_PUBLISHER_H
