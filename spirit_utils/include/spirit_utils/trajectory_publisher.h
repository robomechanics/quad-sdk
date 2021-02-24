#ifndef TRAJECTORY_PUBLISHER_H
#define TRAJECTORY_PUBLISHER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <spirit_msgs/BodyPlan.h>
#include <spirit_msgs/MultiFootPlanContinuous.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/RobotStateTrajectory.h>
#include "spirit_utils/math_utils.h"
#include "spirit_utils/kinematics.h"

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
   * @brief Callback function to handle new continuous foot plan data
   * @param[in] MultiFootPlanContinuous message containing foot plan data
   */
  void footPlanContinuousCallback(
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
  ros::Subscriber foot_plan_continuous_sub_;

  /// ROS Publisher for the current trajectory state
  ros::Publisher trajectory_state_pub_;

  /// ROS Publisher for the entire trajectory
  ros::Publisher trajectory_pub_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Vector of body states to store the body plan
  std::vector<std::vector<double> > body_plan_;

  /// Vector of joint states to store the joint plan
  std::vector<std::vector<double> > joints_plan_;

  //   /// Vector of joint states to store the foot plan
  // std::vector<std::vector<Eigen::Vector3d> > > foot_plan_;

  /// Vector of times corresponding to the body plan states
  std::vector<double> t_body_plan_;

  /// Vector of times corresponding to the joint plan states
  std::vector<double> t_joints_plan_;

  //   /// Vector of times corresponding to the foot plan states
  // std::vector<double> t_foot_plan_;

  /// Vector of times corresponding to the trajectory states
  std::vector<double> t_traj_;

  /// Update rate for sending and receiving data
  double update_rate_;

  /// Timestep for trajectory interpolation
  double interp_dt_;

  /// Handle for the map frame
  std::string map_frame_;

  /// Timestamp for the beginning of the trajectory
  ros::Time trajectory_timestamp_;

  /// Message for robot trajectory
  spirit_msgs::RobotStateTrajectory traj_msg_;

  /// Playback speed for trajectory state publishing
  double playback_speed_;

};

#endif // TRAJECTORY_PUBLISHER_H
