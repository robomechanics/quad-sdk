#ifndef CONFLICT_BASED_SEARCH_HPP_
#define CONFLICT_BASED_SEARCH_HPP_

#include <chrono>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <quad_msgs/msg/robot_plan.hpp>
#include <quad_msgs/msg/robot_plan_constraints.hpp>
#include <quad_msgs/msg/robot_state.hpp>
#include <quad_msgs/srv/plan_with_constraints.hpp>
#include <quad_utils/ros_utils.hpp>
#include <rclcpp/rclcpp.hpp>

namespace conflict_based_search {

/**
 * @brief A single inter-robot collision detected when comparing two plans.
 *
 * `t_start_idx` and `t_end_idx` index into the iterating ("primary") robot's
 * plan; the other robot's body poses across the same time window are what
 * the primary robot must avoid in its replan.
 */
struct Conflict {
  std::string robot_a;  //!< Primary robot whose plan we were iterating
  std::string robot_b;  //!< Other robot involved in the collision
  int t_start_idx;      //!< First timestep (in robot_a's plan) of the conflict
  int t_end_idx;        //!< Last timestep (inclusive) of the conflict
};

/**
 * @brief A high-level constraint-tree node in the conflict-based search.
 *
 * Each node holds a complete set of per-robot plans and, for each robot, a
 * vector of forbidden body-poses inherited from ancestor conflict
 * resolutions. The cost is the sum of every robot's path length.
 */
struct CBSNode {
  std::vector<std::string> robot_names;
  std::map<std::string, quad_msgs::msg::RobotPlan> robot_plan_map;
  std::map<std::string, double> cost_map;
  std::map<std::string, quad_msgs::msg::RobotPlanConstraints> constraints;
  double cost = std::numeric_limits<double>::infinity();

  void updateCost() {
    cost = 0.0;
    for (const auto& robot : robot_names) {
      auto it = cost_map.find(robot);
      if (it != cost_map.end()) cost += it->second;
    }
  }
};

/**
 * @brief Min-heap comparator used by the CBS open list.
 */
struct CBSNodeCompare {
  bool operator()(const std::shared_ptr<CBSNode>& a,
                  const std::shared_ptr<CBSNode>& b) const {
    return a->cost > b->cost;
  }
};

/**
 * @brief Conflict-based search node that orchestrates per-robot global body
 * planning under inter-robot collision constraints.
 *
 * The node spawns one async client per robot, asks each for an initial plan,
 * detects conflicts using oriented-bounding-box tests on the time-aligned
 * trajectories, and resolves them by branching on each conflict (one branch
 * per involved robot) and replanning with new constraints.
 */
class ConflictBasedSearch {
 public:
  /**
   * @brief Constructor for ConflictBasedSearch Class.
   * @param[in] node Shared pointer to the ROS node used for pubs/subs/services.
   */
  explicit ConflictBasedSearch(rclcpp::Node::SharedPtr node);

  /**
   * @brief Run the CBS algorithm and publish the resulting per-robot plans.
   *
   * Returns once a conflict-free solution has been published, the open list
   * is exhausted, or rclcpp::ok() goes false.
   */
  void run();

  /**
   * @brief Block until every robot's plan_with_constraints service is up.
   * @param[in] timeout Maximum wait time per service.
   * @return True if every service is available before timeout.
   */
  bool waitForServices(std::chrono::seconds timeout);

#ifdef CONFLICT_BASED_SEARCH_TESTING
 public:
#else
 private:
#endif
  // -------- Service plumbing --------
  /**
   * @brief Create one plan_with_constraints service client per robot.
   */
  void createServiceClients();

  /**
   * @brief Synchronous call into one robot's plan_with_constraints service.
   * @param[in] robot Robot namespace whose planner should be called.
   * @param[in] constraints Constraint set to pass to that robot's planner.
   * @param[in] warm_start Whether the planner should reuse cached RRT trees.
   * @param[out] out_plan Plan returned by the robot planner.
   * @param[out] out_path_length Path length returned by the robot planner.
   * @return True iff the service returned a successful plan.
   */
  bool callPlanWithConstraints(
      const std::string& robot,
      const quad_msgs::msg::RobotPlanConstraints& constraints, bool warm_start,
      quad_msgs::msg::RobotPlan& out_plan, double& out_path_length);

  /**
   * @brief Populate every robot's plan in a fresh CBS root node.
   * @param[in,out] node CBS node receiving initial plans, costs, and constraints.
   * @return True if every robot returned an initial plan.
   */
  bool requestInitialPlans(CBSNode& node);

  /**
   * @brief Replan one robot in a CBS child node.
   * @param[in,out] node CBS node containing the updated constraint set.
   * @param[in] robot Robot namespace to replan.
   * @return True if the robot returned a valid constrained plan.
   */
  bool requestReplan(CBSNode& node, const std::string& robot);

  // -------- Conflict detection --------
  /**
   * @brief Detect the earliest inter-robot conflict in a CBS node.
   * @param[in] node CBS node containing the candidate per-robot plans.
   * @param[out] out First conflict found, if any.
   * @return True if at least one conflict is found.
   */
  bool findFirstConflict(const CBSNode& node, Conflict& out) const;

  /**
   * @brief Build the constraint message for one branch of a CBS conflict.
   * @param[in] node CBS node containing plans and inherited constraints.
   * @param[in] conflict Conflict to resolve.
   * @param[in] robot_to_constrain Robot that should avoid the other robot.
   * @return Constraint message for the constrained robot.
   */
  quad_msgs::msg::RobotPlanConstraints buildConstraintFromConflict(
      const CBSNode& node, const Conflict& conflict,
      const std::string& robot_to_constrain) const;

  // -------- Output --------
  /**
   * @brief Publish the selected per-robot plans.
   * @param[in] node CBS node whose plans should be published.
   */
  void publishPlans(const CBSNode& node);

  // -------- Geometry helpers --------
  struct BodyPose {
    Eigen::Vector3d pos;
    double yaw;
    double t;
  };

  /**
   * @brief Extract body position, yaw, and timestamp from a RobotState.
   * @param[in] state Robot state message to read.
   * @return Body pose used for collision checking.
   */
  static BodyPose poseFromState(const quad_msgs::msg::RobotState& state);

  /**
   * @brief Sample a robot plan at a requested plan time.
   * @param[in] plan Robot plan to sample.
   * @param[in] t Requested timestamp, s.
   * @return Interpolated body pose, clamped to plan endpoints.
   */
  static BodyPose samplePoseAtTime(const quad_msgs::msg::RobotPlan& plan,
                                   double t);

  /**
   * @brief OBB-OBB SAT collision test between two body poses with the
   * supplied half-extents.
   * @param[in] a First body pose.
   * @param[in] half_a Half-extents of the first body OBB.
   * @param[in] b Second body pose.
   * @param[in] half_b Half-extents of the second body OBB.
   * @return True if the two OBBs overlap.
   */
  static bool obbsOverlap(const BodyPose& a, const Eigen::Vector3d& half_a,
                          const BodyPose& b, const Eigen::Vector3d& half_b);

  // -------- ROS handles --------
  /// ROS node used for parameters, service clients, and plan publishers.
  rclcpp::Node::SharedPtr node_;

  /// Namespaces of the robots coordinated by CBS.
  std::vector<std::string> robot_names_;

  /// Service clients for each robot's plan_with_constraints service.
  std::map<std::string,
           rclcpp::Client<quad_msgs::srv::PlanWithConstraints>::SharedPtr>
      robot_clients_;

  /// Publishers for the selected global plan for each robot.
  std::map<std::string,
           rclcpp::Publisher<quad_msgs::msg::RobotPlan>::SharedPtr>
      robot_plan_pubs_;

  // -------- Parameters --------
  /// Rate used while polling asynchronous planner service calls, Hz.
  double update_rate_;

  /// Maximum wait time for each plan_with_constraints service call, s.
  double service_timeout_s_;

  /// Shared body OBB half-extents used for homogeneous-fleet collision checks.
  Eigen::Vector3d half_extents_;

  /// Maximum number of CBS expansions before fallback publication.
  int max_iterations_;

  /// Whether replans should reuse each robot planner's cached RRT trees.
  bool warm_start_;
};

}  // namespace conflict_based_search

#endif  // CONFLICT_BASED_SEARCH_HPP_
