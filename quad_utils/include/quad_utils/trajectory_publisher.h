#ifndef TRAJECTORY_PUBLISHER_H
#define TRAJECTORY_PUBLISHER_H

#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/RobotState.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen3/Eigen/Eigen>

#include "quad_utils/function_timer.h"
#include "quad_utils/math_utils.h"
#include "quad_utils/quad_kd.h"
#include "quad_utils/ros_utils.h"

//! A class for publishing the current state of a trajectory
/*!
   TrajectoryPublisher is a class for publishing the current state of a given
   robot trajectory. It subscribes to a topic of type RobotPlan or can be
   customized to import data directly (such as from a .csv), then interpolates
   that trajectory to find the state at the current time and publishes it to the
   trajectory state topic.
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
   * @brief Import trajectory from external source (user implemented)
   */
  void importTrajectory();

  /**
   * @brief Callback function to handle new body plan data
   * @param[in] msg Body plan message contining interpolated output of body
   * planner
   */
  void robotPlanCallback(const quad_msgs::RobotPlan::ConstPtr& msg);

  /**
   * @brief Publish the current trajectory state
   */
  void publishTrajectoryState();

  /// ROS Subscriber for the body plan
  ros::Subscriber body_plan_sub_;

  /// ROS Publisher for the current trajectory state
  ros::Publisher trajectory_state_pub_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Vector of body states to store the body plan
  quad_msgs::RobotPlan body_plan_msg_;

  /// Robot state message
  quad_msgs::RobotState::ConstPtr robot_state_msg_;

  /// Update rate for sending and receiving data
  double update_rate_;

  /// Handle for the map frame
  std::string map_frame_;

  /// The source of the current trajectory (import or topic)
  std::string traj_source_;

  /// QuadKD class
  std::shared_ptr<quad_utils::QuadKD> quadKD_;
};

#endif  // TRAJECTORY_PUBLISHER_H
