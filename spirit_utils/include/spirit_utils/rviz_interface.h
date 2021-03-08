#ifndef RVIZ_INTERFACE_H
#define RVIZ_INTERFACE_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/BodyPlan.h>
#include <spirit_msgs/FootState.h>
#include <spirit_msgs/MultiFootState.h>
#include <spirit_msgs/MultiFootPlanContinuous.h>
#include <spirit_msgs/FootPlanDiscrete.h>
#include <spirit_msgs/MultiFootPlanDiscrete.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>


//! A class for interfacing between RViz and spirit-software topics.
/*!
   RVizInterface is a container for all of the logic utilized in the template node.
   The implementation must provide a clean and high level interface to the core algorithm
*/
class RVizInterface {
public:
  /**
   * @brief Constructor for RVizInterface Class
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type RVizInterface
   */
  RVizInterface(ros::NodeHandle nh);

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

private:
  /**
   * @brief Callback function to handle new body plan data
   * @param[in] Body plan message contining interpolated output of body planner
   */
  void bodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg);

  /**
   * @brief Callback function to handle new body plan discrete state data
   * @param[in] Body plan message contining discrete output of body planner
   */
  void discreteBodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg);

  /**
   * @brief Callback function to handle new discrete foot plan data
   * @param[in] Footstep plan message containing output of footstep planner
   */
  void footPlanDiscreteCallback(const spirit_msgs::MultiFootPlanDiscrete::ConstPtr& msg);

  /**
   * @brief Callback function to handle new continous foot plan data
   * @param[in] SwingLegPlan message containing output of swing leg planner
   */
  void footPlanContinuousCallback(const spirit_msgs::MultiFootPlanContinuous::ConstPtr& msg);

  /**
   * @brief Callback function to handle new state estimate data
   * @param[in] msg RobotState message containing output of the state estimator node
   */
  void stateEstimateCallback(const spirit_msgs::RobotState::ConstPtr& msg);

  /**
   * @brief Callback function to handle new robot state data
   * @param[in] msg RobotState message containing output of the state estimator node
   * @param[in] pub_id Identifier of which publisher to use to handle this data
   */
  void robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg, const int pub_id);

  /// ROS subscriber for the body plan
  ros::Subscriber body_plan_sub_;

  /// ROS subscriber for the body plan
  ros::Subscriber discrete_body_plan_sub_;

  /// ROS subscriber for the discrete foot plan
  ros::Subscriber foot_plan_discrete_sub_;

  /// ROS subscriber for the continuous foot plan
  ros::Subscriber foot_plan_continuous_sub_;

  /// ROS Publisher for the interpolated body plan vizualization
  ros::Publisher body_plan_viz_pub_;

  /// ROS Publisher for the interpolated grf plan vizualization
  ros::Publisher grf_plan_viz_pub_;

  /// ROS Publisher for the discrete body plan vizualization
  ros::Publisher discrete_body_plan_viz_pub_;

  /// ROS Publisher for the footstep plan visualization
  ros::Publisher foot_plan_discrete_viz_pub_;

  /// ROS Publisher for the swing leg 0 visualization
  ros::Publisher foot_0_plan_continuous_viz_pub_;

  /// ROS Publisher for the foot 1 plan visualization
  ros::Publisher foot_1_plan_continuous_viz_pub_;

  /// ROS Publisher for the foot 2 plan visualization
  ros::Publisher foot_2_plan_continuous_viz_pub_;

  /// ROS Publisher for the foot 3 plan visualization
  ros::Publisher foot_3_plan_continuous_viz_pub_;

  /// ROS Publisher for the estimated joint states visualization
  ros::Publisher estimate_joint_states_viz_pub_;

  /// ROS Publisher for the ground truth joint states visualization
  ros::Publisher ground_truth_joint_states_viz_pub_;

  /// ROS Publisher for the trajectory joint states visualization
  ros::Publisher trajectory_joint_states_viz_pub_;

  /// ROS Subscriber for the state estimate
  ros::Subscriber state_estimate_sub_;

  /// ROS Subscriber for the ground truth state
  ros::Subscriber ground_truth_state_sub_;

  /// ROS Subscriber for the ground truth state
  ros::Subscriber trajectory_state_sub_;

  /// ROS Transform Broadcaster to publish the estimate transform for the base link
  tf2_ros::TransformBroadcaster estimate_base_tf_br_;

  /// ROS Transform Broadcaster to publish the ground truth transform for the base link
  tf2_ros::TransformBroadcaster ground_truth_base_tf_br_;

  /// ROS Transform Broadcaster to publish the trajectory transform for the base link
  tf2_ros::TransformBroadcaster trajectory_base_tf_br_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Update rate for sending and receiving data, unused since pubs are called in callbacks
  double update_rate_;

  /// Handle for the map frame
  std::string map_frame_;

  /// Colors
  std::vector<int> front_left_color_;
  std::vector<int> back_left_color_;
  std::vector<int> front_right_color_;
  std::vector<int> back_right_color_;
  std::vector<int> net_grf_color_;
  std::vector<int> individual_grf_color_;

  /// Publisher IDs
  const int ESTIMATE = 0;
  const int GROUND_TRUTH = 1;
  const int TRAJECTORY = 2;
};

#endif // RVIZ_INTERFACE_H
