#ifndef RVIZ_INTERFACE_H
#define RVIZ_INTERFACE_H
#include <quad_utils/ros_utils.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <quad_msgs/msg/foot_plan_discrete.hpp>
#include <quad_msgs/msg/foot_state.hpp>
#include <quad_msgs/msg/grf_array.hpp>
#include <quad_msgs/msg/multi_foot_plan_continuous.hpp>
#include <quad_msgs/msg/multi_foot_plan_discrete.hpp>
#include <quad_msgs/msg/multi_foot_state.hpp>
#include <quad_msgs/msg/robot_plan.hpp>
#include <quad_msgs/msg/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
  RVizInterface(rclcpp::Node::SharedPtr node);

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

 private:
  /**
   * @brief Callback function to handle new body plan data
   * @param[in] msg plan message contining interpolated output of body planner
   */
  void robotPlanCallback(const quad_msgs::msg::RobotPlan::SharedPtr msg,
                         const int pub_id);

  /**
   * @brief Callback function to handle new grf data
   * @param[in] msg plan message contining interpolated output of body planner
   */
  void grfCallback(const quad_msgs::msg::GRFArray::SharedPtr msg);

  /**
   * @brief Callback function to handle new body plan discrete state data
   * @param[in] msg plan message contining discrete output of body planner
   */
  void discreteBodyPlanCallback(const quad_msgs::msg::RobotPlan::SharedPtr msg);

  /**
   * @brief Callback function to handle new discrete foot plan data
   * @param[in] Footstep plan message containing output of footstep planner
   */
  void footPlanDiscreteCallback(
      const quad_msgs::msg::MultiFootPlanDiscrete::SharedPtr msg);

  /**
   * @brief Callback function to handle new continous foot plan data
   * @param[in] SwingLegPlan message containing output of swing leg planner
   */
  void footPlanContinuousCallback(
      const quad_msgs::msg::MultiFootPlanContinuous::SharedPtr msg);

  /**
   * @brief Callback function to handle new state estimate data
   * @param[in] msg RobotState message containing output of the state estimator
   * node
   */
  void stateEstimateCallback(const quad_msgs::msg::RobotState::SharedPtr msg);

  /**
   * @brief Callback function to handle new robot state data
   * @param[in] msg RobotState message containing output of the state estimator
   * node
   * @param[in] pub_id Identifier of which publisher to use to handle this data
   */
  void robotStateCallback(const quad_msgs::msg::RobotState::SharedPtr msg,
                          const int pub_id);

  rclcpp::Node::SharedPtr node_;
  /// ROS subscriber for the global plan
  rclcpp::Subscription<quad_msgs::msg::RobotPlan>::SharedPtr global_plan_sub_;

  /// ROS subscriber for the local plan
  rclcpp::Subscription<quad_msgs::msg::RobotPlan>::SharedPtr local_plan_sub_;

  /// ROS subscriber for the current
  rclcpp::Subscription<quad_msgs::msg::GRFArray>::SharedPtr grf_sub_;

  /// ROS subscriber for the body plan
  rclcpp::Subscription<quad_msgs::msg::RobotPlan>::SharedPtr
      discrete_body_plan_sub_;

  /// ROS subscriber for the discrete foot plan
  rclcpp::Subscription<quad_msgs::msg::MultiFootPlanDiscrete>::SharedPtr
      foot_plan_discrete_sub_;

  /// ROS subscriber for the continuous foot plan
  rclcpp::Subscription<quad_msgs::msg::MultiFootPlanContinuous>::SharedPtr
      foot_plan_continuous_sub_;

  /// ROS Publisher for the interpolated global plan vizualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      global_plan_viz_pub_;

  /// ROS Publisher for the interpolated local plan vizualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      local_plan_viz_pub_;

  /// ROS Publisher for the current GRFs
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      current_grf_viz_pub_;

  /// ROS Publisher for local plan orientation vizualization
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      local_plan_ori_viz_pub_;

  /// ROS Publisher for the interpolated global plan grf vizualization
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      global_plan_grf_viz_pub_;

  /// ROS Publisher for the interpolated local plan grf vizualization
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      local_plan_grf_viz_pub_;

  /// ROS Publisher for the discrete body plan vizualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      discrete_body_plan_viz_pub_;

  /// ROS Publisher for the footstep plan visualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      foot_plan_discrete_viz_pub_;

  /// ROS Publisher for the state estimate body trace
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      state_estimate_trace_pub_;

  /// ROS Publisher for the ground truth state body trace
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      ground_truth_state_trace_pub_;

  /// ROS Publisher for the trajectory state body trace
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      trajectory_state_trace_pub_;

  /// ROS Publisher for the swing leg 0 visualization
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      foot_0_plan_continuous_viz_pub_;

  /// ROS Publisher for the foot 1 plan visualization
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      foot_1_plan_continuous_viz_pub_;

  /// ROS Publisher for the foot 2 plan visualization
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      foot_2_plan_continuous_viz_pub_;

  /// ROS Publisher for the foot 3 plan visualization
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      foot_3_plan_continuous_viz_pub_;

  /// ROS Publisher for the estimated joint states visualization
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      estimate_joint_states_viz_pub_;

  /// ROS Publisher for the ground truth joint states visualization
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      ground_truth_joint_states_viz_pub_;

  /// ROS Publisher for the trajectory joint states visualization
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      trajectory_joint_states_viz_pub_;

  /// ROS Subscriber for the state estimate
  rclcpp::Subscription<quad_msgs::msg::RobotState>::SharedPtr
      state_estimate_sub_;

  /// ROS Subscriber for the ground truth state
  rclcpp::Subscription<quad_msgs::msg::RobotState>::SharedPtr
      ground_truth_state_sub_;

  /// ROS Subscriber for the ground truth state
  rclcpp::Subscription<quad_msgs::msg::RobotState>::SharedPtr
      trajectory_state_sub_;

  /// ROS Transform Broadcaster to publish the estimate transform for the base
  /// link
  std::shared_ptr<tf2_ros::TransformBroadcaster> estimate_base_tf_br_;

  /// ROS Transform Broadcaster to publish the ground truth transform for the
  /// base link
  std::shared_ptr<tf2_ros::TransformBroadcaster> ground_truth_base_tf_br_;

  /// ROS Transform Broadcaster to publish the trajectory transform for the base
  /// link
  std::shared_ptr<tf2_ros::TransformBroadcaster> trajectory_base_tf_br_;

  /// Message for state estimate trace
  visualization_msgs::msg::Marker state_estimate_trace_msg_;

  /// Message for ground truth state trace
  visualization_msgs::msg::Marker ground_truth_state_trace_msg_;

  /// Message for trajectory state trace
  visualization_msgs::msg::Marker trajectory_state_trace_msg_;

  /// Distance threshold for resetting the state traces
  const double trace_reset_threshold_ = 0.2;

  /// Update rate for sending and receiving data, unused since pubs are called
  /// in callbacks
  double update_rate_;

  /// Interval for showing orientation of plan
  int orientation_subsample_interval_;

  /// Handle for the map frame
  std::string map_frame_;

  /// Root link name for the robot model
  std::string body_frame_;

  /// Handle multiple robots
  std::string tf_prefix_;

  /// Colors
  std::vector<int64_t> front_left_color_;
  std::vector<int64_t> back_left_color_;
  std::vector<int64_t> front_right_color_;
  std::vector<int64_t> back_right_color_;
  std::vector<int64_t> net_grf_color_;
  std::vector<int64_t> individual_grf_color_;

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
