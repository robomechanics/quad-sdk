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
   * Useful for testing and for guarding against startup races.
   */
  bool waitForServices(std::chrono::seconds timeout);

 private:
  // -------- Service plumbing --------
  void createServiceClients();

  /**
   * @brief Synchronous call into one robot's plan_with_constraints service.
   * Spins the node while waiting for the response so the executor can
   * service incoming subscriptions in the meantime.
   * @return True iff the service returned a successful plan.
   */
  bool callPlanWithConstraints(
      const std::string& robot,
      const quad_msgs::msg::RobotPlanConstraints& constraints, bool warm_start,
      quad_msgs::msg::RobotPlan& out_plan, double& out_path_length);

  /// Populate every robot's plan in a fresh root node.
  bool requestInitialPlans(CBSNode& node);

  /// Replan only the conflicting robot in a child node, given its updated
  /// constraint set.
  bool requestReplan(CBSNode& node, const std::string& robot);

  // -------- Conflict detection --------
  /// Detect the first inter-robot conflict in `node`. Returns true and
  /// populates `out` if at least one is found. The "first" conflict is the
  /// one with the earliest start time, mirroring textbook CBS.
  bool findFirstConflict(const CBSNode& node, Conflict& out) const;

  /// Build the constraint message that, when added to `robot_to_constrain`'s
  /// constraint list, forces the planner to avoid the other robot during
  /// the conflict window.
  quad_msgs::msg::RobotPlanConstraints buildConstraintFromConflict(
      const CBSNode& node, const Conflict& conflict,
      const std::string& robot_to_constrain) const;

  // -------- Output --------
  void publishPlans(const CBSNode& node);

  // -------- Geometry helpers --------
  struct BodyPose {
    Eigen::Vector3d pos;
    double yaw;
    double t;
  };

  /// Pull (x,y,z,yaw) and plan time out of a RobotState message.
  static BodyPose poseFromState(const quad_msgs::msg::RobotState& state);

  /// Sample plan B at the same plan-time as `state_a`, interpolating between
  /// the two surrounding states. Clamps to plan endpoints.
  static BodyPose samplePoseAtTime(const quad_msgs::msg::RobotPlan& plan,
                                   double t);

  /**
   * @brief OBB-OBB SAT collision test between two body poses with the
   * supplied half-extents.
   */
  static bool obbsOverlap(const BodyPose& a, const Eigen::Vector3d& half_a,
                          const BodyPose& b, const Eigen::Vector3d& half_b);

  // -------- ROS handles --------
  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> robot_names_;
  std::map<std::string,
           rclcpp::Client<quad_msgs::srv::PlanWithConstraints>::SharedPtr>
      robot_clients_;
  std::map<std::string,
           rclcpp::Publisher<quad_msgs::msg::RobotPlan>::SharedPtr>
      robot_plan_pubs_;

  // -------- Parameters --------
  double update_rate_;
  double service_timeout_s_;
  /// Half-extents of every robot's body. CBS does not currently support
  /// heterogeneous robot sizes — we use one set of extents shared across
  /// all robots, which matches the existing single-robot Quad-SDK config.
  Eigen::Vector3d half_extents_;
  /// Maximum number of CBS expansions before giving up.
  int max_iterations_;
  /// Whether to ask each robot's planner to warm-start from its previous
  /// solve when replanning under a new constraint.
  bool warm_start_;
};

}  // namespace conflict_based_search

#endif  // CONFLICT_BASED_SEARCH_HPP_
