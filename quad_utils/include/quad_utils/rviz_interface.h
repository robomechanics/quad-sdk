#ifndef RVIZ_INTERFACE_H
#define RVIZ_INTERFACE_H

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/RobotPlan.h>
#include <spirit_msgs/FootState.h>
#include <spirit_msgs/MultiFootState.h>
#include <spirit_msgs/MultiFootPlanContinuous.h>
#include <spirit_msgs/FootPlanDiscrete.h>
#include <spirit_msgs/MultiFootPlanDiscrete.h>
#include <spirit_msgs/GRFArray.h>
#include <spirit_utils/ros_utils.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//! A class for interfacing between RViz and quad-sdk topics.
/*!
   RVizInterface is a container for all of the logic utilized in the template
   node. The implementation must provide a clean and high level interface to the
   core algorithm
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
   * @param[in] msg plan message contining interpolated output of body planner
   */
  void grfCallback(const quad_msgs::GRFArray::ConstPtr &msg);

  /**
   * @brief Callback function to handle new grf data
   * @param[in] msg plan message contining interpolated output of body planner
   */
  void grfCallback(const spirit_msgs::GRFArray::ConstPtr& msg);

  /**
   * @brief Callback function to handle new body plan discrete state data
   * @param[in] msg plan message contining discrete output of body planner
   */
  void discreteBodyPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg);

  /**
   * @brief Callback function to handle new discrete foot plan data
   * @param[in] Footstep plan message containing output of footstep planner
   */
  void footPlanDiscreteCallback(
      const quad_msgs::MultiFootPlanDiscrete::ConstPtr &msg);

  /**
   * @brief Callback function to handle new continous foot plan data
   * @param[in] SwingLegPlan message containing output of swing leg planner
   */
  void footPlanContinuousCallback(
      const quad_msgs::MultiFootPlanContinuous::ConstPtr &msg);

  /**
   * @brief Callback function to handle new state estimate data
   * @param[in] msg RobotState message containing output of the state estimator
   * node
   */
  void stateEstimateCallback(const quad_msgs::RobotState::ConstPtr &msg);

  /**
   * @brief Callback function to handle new robot state data
   * @param[in] msg RobotState message containing output of the state estimator
   * node
   * @param[in] pub_id Identifier of which publisher to use to handle this data
   */
  void robotStateCallback(const quad_msgs::RobotState::ConstPtr &msg,
                          const int pub_id);

  /// ROS subscriber for the global plan
  ros::Subscriber global_plan_sub_;

  /// ROS subscriber for the local plan
  ros::Subscriber local_plan_sub_;

  /// ROS subscriber for the current
  ros::Subscriber grf_sub_;

  /// ROS subscriber for the body plan
  ros::Subscriber discrete_body_plan_sub_;

  /// ROS subscriber for the discrete foot plan
  ros::Subscriber foot_plan_discrete_sub_;

  /// ROS subscriber for the continuous foot plan
  ros::Subscriber foot_plan_continuous_sub_;

  /// ROS Publisher for the interpolated global plan vizualization
  ros::Publisher global_plan_viz_pub_;

  /// ROS Publisher for the interpolated local plan vizualization
  ros::Publisher local_plan_viz_pub_;

  /// ROS Publisher for the current GRFs
  ros::Publisher current_grf_viz_pub_;

  /// ROS Publisher for local plan orientation vizualization
  ros::Publisher local_plan_ori_viz_pub_;

  /// ROS Publisher for the interpolated global plan grf vizualization
  ros::Publisher global_plan_grf_viz_pub_;

  /// ROS Publisher for the interpolated local plan grf vizualization
  ros::Publisher local_plan_grf_viz_pub_;

  /// ROS Publisher for the discrete body plan vizualization
  ros::Publisher discrete_body_plan_viz_pub_;

  /// ROS Publisher for the footstep plan visualization
  ros::Publisher foot_plan_discrete_viz_pub_;

  /// ROS Publisher for the state estimate body trace
  ros::Publisher state_estimate_trace_pub_;

  /// ROS Publisher for the ground truth state body trace
  ros::Publisher ground_truth_state_trace_pub_;

  /// ROS Publisher for the trajectory state body trace
  ros::Publisher trajectory_state_trace_pub_;

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

  /// ROS Transform Broadcaster to publish the estimate transform for the base
  /// link
  tf2_ros::TransformBroadcaster estimate_base_tf_br_;

  /// ROS Transform Broadcaster to publish the ground truth transform for the
  /// base link
  tf2_ros::TransformBroadcaster ground_truth_base_tf_br_;

  /// ROS Transform Broadcaster to publish the trajectory transform for the base
  /// link
  tf2_ros::TransformBroadcaster trajectory_base_tf_br_;

  /// Message for state estimate trace
  visualization_msgs::Marker state_estimate_trace_msg_;

  /// Message for ground truth state trace
  visualization_msgs::Marker ground_truth_state_trace_msg_;

  /// Message for trajectory state trace
  visualization_msgs::Marker trajectory_state_trace_msg_;

  /// Distance threshold for resetting the state traces
  const double trace_reset_threshold_ = 0.2;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Update rate for sending and receiving data, unused since pubs are called
  /// in callbacks
  double update_rate_;

  /// Number for showing orientation of plan
  int orientation_subsample_num_;

  /// Handle for the map frame
  std::string map_frame_;

  /// Handle multiple robots
  std::string tf_prefix_;

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

  const int GLOBAL = 0;
  const int LOCAL = 1;

  const int CONNECT = 0;
  const int LEAP_STANCE = 1;
  const int FLIGHT = 2;
  const int LAND_STANCE = 3;
};

#endif  // RVIZ_INTERFACE_H
