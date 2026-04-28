#ifndef GLOBAL_BODY_PLANNER_H
#define GLOBAL_BODY_PLANNER_H

#include <nav_msgs/msg/path.hpp>
#include <quad_msgs/msg/robot_plan.hpp>
#include <quad_msgs/msg/robot_plan_constraints.hpp>
#include <quad_msgs/msg/robot_state.hpp>
#include <quad_msgs/srv/plan_with_constraints.hpp>
#include <quad_utils/ros_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include "global_body_planner/gbpl.hpp"
#include "global_body_planner/global_body_plan.hpp"
#include "rclcpp/wait_for_message.hpp"

using namespace planning_utils;

//! A global body planning class for legged robots
/*!
   GlobalBodyPlanner is a container for all of the logic utilized in the global
   body planning node. This algorithm requires an height map of the terrain as a
   GridMap message type, and will publish the global body plan as a BodyPlan
   message over a topic. It will also publish the discrete states used by the
   planner (from which the full path is interpolated).
*/
class GlobalBodyPlanner {
 public:
  /**
   * @brief Constructor for GlobalBodyPlanner Class
   * @param[in] node Shared Pointer to Node Object
   * @return Constructed object of type GlobalBodyPlanner
   */
  GlobalBodyPlanner(rclcpp::Node::SharedPtr node);

  /**
   * @brief Call the correct planning class and compute statistics
   * @return Boolean for success of the planner
   */
  bool callPlanner();

  /**
   * @brief Primary work function in class, called in node file for this
   * component
   */
  void spin();

  /**
   * @brief Service callback used by the conflict-based search node. Loads
   * the supplied constraints into the planner config, optionally warm-starts
   * from the previously cached trees, runs callPlanner(), and returns the
   * resulting plan plus its path length.
   */
  void planWithConstraintsCallback(
      const std::shared_ptr<quad_msgs::srv::PlanWithConstraints::Request>
          request,
      std::shared_ptr<quad_msgs::srv::PlanWithConstraints::Response> response);

 private:
  /**
   * @brief Callback function to handle new terrain map data
   * @param[in] msg the message contining map data
   */
  void terrainMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);

  /**
   * @brief Callback function to handle new robot state data
   * @param[in] msg the message contining robot state data
   */
  void robotStateCallback(const quad_msgs::msg::RobotState::SharedPtr msg);

  /**
   * @brief Callback function to handle new goal state
   * @param[in] msg the message contining the goal state
   */
  void goalStateCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  /**
   * @brief Trigger a reset event
   */
  void triggerReset();

  /**
   * @brief Initialize the planner by clearing out old plan data and setting the
   * start state
   * @return Index of the current plan from which to being the new plan (zero if
   * fully replanning)
   */
  int initPlanner();

  /**
   * @brief Clear all data in plan member variables
   */
  void clearPlan();

  /**
   * @brief Update the body plan with the current plan
   * @param[in] t Time of state in trajectory
   * @param[in] body_state Body state
   * @param[in] grf GRF applied to body
   * @param[in] body_plan_msg Body plan message
   */
  void addStateAndGRFToMsg(double t, int plan_index, FullState body_state,
                           GRF grf, int primitive_id,
                           quad_msgs::msg::RobotPlan& body_plan_msg);

  /**
   * @brief Publish the current body plan
   */
  void publishPlan();

  /**
   * @brief Wait until map and state messages have been received and processed
   */
  void waitForData();

  /**
   * @brief Call the planner repeatedly until startup_delay has been reached
   * then return
   */
  void getInitialPlan();

  /**
   * @brief Set the start state to be used by the next planning call
   */
  void setStartState();

  /**
   * @brief Set the goal state to be used by the next planning call
   */
  void setGoalState();

  /**
   * @brief Publish the current plan if updated
   */
  void publishCurrentPlan();

  /// Subscriber for terrain map messages
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_map_sub_;

  /// Subscriber for robot state messages
  rclcpp::Subscription<quad_msgs::msg::RobotState>::SharedPtr robot_state_sub_;

  /// Subscriber for goal state messages
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      goal_state_sub_;

  /// Publisher for body plan messages
  rclcpp::Publisher<quad_msgs::msg::RobotPlan>::SharedPtr body_plan_pub_;

  /// Publisher for discrete states in body plan messages
  rclcpp::Publisher<quad_msgs::msg::RobotPlan>::SharedPtr
      discrete_body_plan_pub_;

  /// Publisher for the planning tree
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;

  /// Publisher for goal reached signal
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;

  /// Service offered to the conflict-based search node. Each call returns a
  /// constraint-respecting plan (warm-starting from the previous solve when
  /// requested).
  rclcpp::Service<quad_msgs::srv::PlanWithConstraints>::SharedPtr
      plan_with_constraints_srv_;

  /// Persistent RRT-Connect planner kept across calls so that warm-started
  /// replans can reuse the trees built on the previous service call.
  GBPL gbpl_;

  /// While set, the spin loop does not call callPlanner() /
  /// publishCurrentPlan(), so the service path becomes the sole source
  /// of published global plans. The first plan_with_constraints call
  /// sets this true, but the constructor also reads the boolean
  /// `global_body_planner.cbs_mode` parameter — when multi_robot.py
  /// passes that as true, spin-loop publishing is suppressed from boot
  /// and avoids the race where a robot whose waitForData() completes
  /// early publishes a solo (no-constraint) plan before CBS has finished
  /// its search. The local_planner subscribed to that solo plan then
  /// receives a CBS plan as a second message, doesn't reset its internal
  /// indexing, and segfaults indexing the new plan with stale state.
  bool cbs_mode_ = false;

  /// Topic name for terrain map (needed to ensure data has been received)
  std::string terrain_map_topic_;

  /// Topic name for robot state data (needed to ensure data has been received)
  std::string robot_state_topic_;

  /// Shared Pointer to Node to pub to and sub from
  rclcpp::Node::SharedPtr node_;

  /// Update rate for sending and receiving data;
  double update_rate_;

  /// Number of times to call the planner
  int num_calls_;

  /// Max time to let the algorithm search
  double max_planning_time_;

  /// Handle for the map frame
  std::string map_frame_;

  /// Plan data for the most recently computed plan (may be suboptimal)
  GlobalBodyPlan newest_plan_;

  /// Plan data for the plan currently being executed
  GlobalBodyPlan current_plan_;

  /// Starting state for planner call
  FullState start_state_;

  /// Goal state for planner
  FullState goal_state_;

  /// Current robot state
  FullState robot_state_;

  /// goal_state_msg_
  geometry_msgs::msg::PointStamped::SharedPtr goal_state_msg_;

  /// Starting time for planner call during replans relative to t_plan_[0]
  double replan_start_time_;

  /// Horizon to commit to (replan from the next state after this horizon)
  double committed_horizon_;

  /// Threshold of state error to trigger replanning
  double pos_error_threshold_;

  /// Flag to determine if the planner needs to restart planning from the robot
  /// state
  bool restart_flag_;

  /// Sequence of discrete states in the plan
  std::vector<State> state_sequence_;

  /// Sequence of discrete actions in the plan
  std::vector<Action> action_sequence_;

  /// Vector of cost instances in each planning call
  std::vector<double> cost_vector_;

  /// Vector of time instances of cost data for each planning call
  std::vector<double> cost_vector_times_;

  /// Vector of solve times for each planning call
  std::vector<double> solve_time_info_;

  /// Vector of number of vertices for each planning call
  std::vector<int> vertices_generated_info_;

  /// Planner config
  PlannerConfig planner_config_;

  /// Delay after plan reset before publishing and refining
  double reset_publish_delay_;

  /// Boolean for whether replanning is allowed
  bool replanning_allowed_;

  /// Timestep for interpolation
  double dt_;

  bool map_recieved_ = false;

  /// ID for status of planner
  int planner_status_;

  /// Index of active plan from which to begin refinement
  int start_index_;

  /// Planning status ID for resetting (start from scratch)
  static const int RESET = 0;

  /// Planning status ID for refining (optimize current plan)
  static const int REFINE = 1;

  /// Time at which reset began
  rclcpp::Time reset_time_;

  /// Boolean for whether to publish the new plan after the reset delay has
  /// occured
  bool publish_after_reset_delay_;

  /// Timestamp for t=0 of global plan
  rclcpp::Time global_plan_timestamp_;
};

#endif  // GLOBAL_BODY_PLANNER_H
