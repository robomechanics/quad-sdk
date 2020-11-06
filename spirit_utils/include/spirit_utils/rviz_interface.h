#ifndef RVIZ_INTERFACE_H
#define RVIZ_INTERFACE_H

#include <ros/ros.h>
#include <spirit_msgs/BodyPlan.h>
#include <spirit_msgs/Footstep.h>
#include <spirit_msgs/FootstepPlan.h>
#include <spirit_msgs/StateEstimate.h>
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
   * @brief Callback function to handle new footstep plan data
   * @param[in] Footstep plan message containing output of footstep planner
   */
  void footstepPlanCallback(const spirit_msgs::FootstepPlan::ConstPtr& msg);

  /**
   * @brief Callback function to handle new state estimate data
   * @param[in] msg State Estimate message containing output of the state estimator node
   */
  void stateEstimateCallback(const spirit_msgs::StateEstimate::ConstPtr& msg);

  /// ROS subscriber for the body plan
  ros::Subscriber body_plan_sub_;

  /// ROS subscriber for the body plan
  ros::Subscriber discrete_body_plan_sub_;

  /// ROS subscriber for the footstep plan
  ros::Subscriber footstep_plan_sub_;

  /// ROS Subscriber for the state estimate
  ros::Subscriber state_estimate_sub_;

  /// ROS Publisher for the interpolated body plan vizualization
  ros::Publisher body_plan_viz_pub_;

  /// ROS Publisher for the discrete body plan vizualization
  ros::Publisher discrete_body_plan_viz_pub_;

  /// ROS Publisher for the footstep plan visualization
  ros::Publisher footstep_plan_viz_pub_;

  /// ROS Publisher for the state estimate visualization
  ros::Publisher joint_states_viz_pub_;

  /// ROS Transform Broadcaster to publish the transform for the base link
  tf2_ros::TransformBroadcaster base_tf_br_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Update rate for sending and receiving data, unused since pubs are called in callbacks
  double update_rate_;

  /// Handle for the map frame
    std::string map_frame_;
};

#endif // RVIZ_INTERFACE_H
