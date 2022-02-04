#ifndef RRTCONNECT_H
#define RRTCONNECT_H

#include "global_body_planner/rrt.h"

#define TRAPPED 0
#define ADVANCED 1
#define REACHED 2

using namespace planning_utils;

//! A class that implements RRT-Connect sampling-based planning.
/*!
   This class inherits the RRTClass, and adds members to connect two states
   exactly and post process the resulting solution.
*/
class RRTConnectClass : public RRTClass {
public:
  /**
   * @brief Constructor for RRTConnectClass
   * @return Constructed object of type RRTConnectClass
   */
  RRTConnectClass();

  /**
   * @brief Destructor for RRTConnectClass
   */
  ~RRTConnectClass();

  /** Connect the tree to the desired state
   * @param[in] T The PlannerClass instance containing the tree
   * @param[in] s The state to connect the tree towards
   * @param[in] planner_config Configuration parameters
   * @param[in] direction The direction with which to peform the extension
   * (FORWARD to go away from the root vertex, REVERSE to go towards it)
   * @param[in] tree_pub Publisher for broadcasting the tree visual
   */
  int connect(PlannerClass &T, State s, const PlannerConfig &planner_config,
              int direction, ros::Publisher &tree_pub);

  /**
   * @brief Get the actions along the specified path of vertex indices (assumes
   * that actions are synched with the state at which they are executed)
   * @param[in] T The PlannerClass instance containing the tree
   * @param[in] path Vector of vertices in the path
   * @return The sequence of actions in the path
   */
  std::vector<Action> getActionSequenceReverse(PlannerClass &T,
                                               std::vector<int> path);

  /**
   * @brief Post process the path by removing extraneous states that can be
   * bypassed
   * @param[in] state_sequence The sequence of states in the path
   * @param[in] action_sequence The sequence of actions in the path
   * @param[in] planner_config Configuration parameters
   */
  void postProcessPath(std::vector<State> &state_sequence,
                       std::vector<Action> &action_sequence,
                       const PlannerConfig &planner_config);

  /**
   * @brief Post process the path by removing extraneous states that can be
   * bypassed
   * @param[in] Ta The planning tree originating from the start state
   * @param[in] Tb The planning tree originating from the end state
   * @param[out] state_sequence The sequence of states in the path
   * @param[out] action_sequence The sequence of actions in th  e path
   * @param[in] planner_config Configuration parameters
   */
  void extractPath(PlannerClass &Ta, PlannerClass &Tb,
                   std::vector<State> &state_sequence,
                   std::vector<Action> &action_sequence,
                   const PlannerConfig &planner_config);

  /**
   * @brief Post process the path by removing extraneous states that can be
   * bypassed
   * @param[in] Ta The planning tree originating from the start state
   * @param[in] s_start The start state of the planner
   * @param[out] state_sequence The sequence of states in the path
   * @param[out] action_sequence The sequence of actions in th  e path
   * @param[in] terrain Height map of the terrain
   */
  void extractClosestPath(PlannerClass &Ta, const State &s_goal,
                          std::vector<State> &state_sequence,
                          std::vector<Action> &action_sequence,
                          const PlannerConfig &planner_config);

  /**
   * @brief Run the full RRT-Connect planner until the goal is found or time has
   * expired, then post process and update statistics
   * @param[in] planner_config Configuration parameters
   * @param[in] s_start The start state of the planner
   * @param[in] s_goal The goal state of the planner
   * @param[out] state_sequence The sequence of states in the final path
   * @param[out] action_sequence The sequence of actions in the final path
   * @return Boolean for success of the planner
   */
  int runRRTConnect(const PlannerConfig &planner_config, State s_start,
                    State s_goal, std::vector<State> &state_sequence,
                    std::vector<Action> &action_sequence,
                    ros::Publisher &tree_pub);

protected:
  /// Time horizon (in seconds) the planner is allowed to search until restarted
  double anytime_horizon;

  /// Initial guess at how quickly the planner executes (in m/s)
  const double planning_rate_estimate =
      16.0; // m/s (meters planned/computation time)

  /// Initial anytime horizon (in seconds)
  double anytime_horizon_init;

  /// Factor by which horizon is increased if replanning is required
  const double horizon_expansion_factor = 1.2;
};

#endif