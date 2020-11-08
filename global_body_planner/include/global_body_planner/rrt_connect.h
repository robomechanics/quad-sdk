#ifndef RRTCONNECT_H
#define RRTCONNECT_H

#include "global_body_planner/rrt.h"
// #include "functions.h"

#define TRAPPED 0
#define ADVANCED 1
#define REACHED 2

using namespace planning_utils;

//! A class that implements RRT-Connect sampling-based planning.
/*!
   This class inherits the RRTClass, and adds members to connect two states exactly and post process the resulting solution.
*/
class RRTConnectClass : public RRTClass
{
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

    /** Attempt to connect two states with specified stance time, and return a new state if the full connection is not possible
     * @param[in] s_existing The state that is already in the tree and closest to the specified state
     * @param[in] s The state to extend the tree towards
     * @param[in] t_s The stance time for this connection
     * @param[out] s_new New state yielded by taking the resulting action
     * @param[out] a_new New action to connect the states
     * @param[in] terrain Height map of the terrain
     * @param[in] direction Direction of the dynamics (either FORWARD or REVERSE)
     * @return Int describing the result of the attempt (TRAPPED, ADVANCED, or REACHED)
     */
    int attemptConnect(State s_existing, State s, double t_s, State &s_new, Action &a_new, FastTerrainMap& terrain, int direction);

    /** Attempt to connect two states, and return a new state if the full connection is not possible. Internally computes stance time
     * @param[in] s_existing The state that is already in the tree and closest to the specified state
     * @param[in] s The state to extend the tree towards
     * @param[out] s_new New state yielded by taking the resulting action
     * @param[out] a_new New action to connect the states
     * @param[in] terrain Height map of the terrain
     * @param[in] direction Direction of the dynamics (either FORWARD or REVERSE)
     * @return Int describing the result of the attempt (TRAPPED, ADVANCED, or REACHED)
     */
    int attemptConnect(State s_existing, State s, State &s_new, Action &a_new, FastTerrainMap& terrain, int direction);

    /** Connect the tree to the desired state
     * @param[in] T The PlannerClass instance containing the tree
     * @param[in] s The state to connect the tree towards
     * @param[in] terrain Height map of the terrain
     * @param[in] direction The direction with which to peform the extension (FORWARD to go away from the root vertex, REVERSE to go towards it)
     */
    int connect(PlannerClass &T, State s, FastTerrainMap& terrain, int direction);

    /**
     * @brief Get the actions along the specified path of vertex indices (assumes that actions are synched with the state at which they are executed)
     * @param[in] T The PlannerClass instance containing the tree
     * @param[in] path Vector of vertices in the path
     * @return The sequence of actions in the path
     */
    std::vector<Action> getActionSequenceReverse(PlannerClass &T, std::vector<int> path);

    /**
     * @brief Post process the path by removing extraneous states that can be bypassed
     * @param[in] state_sequence The sequence of states in the path
     * @param[in] action_sequence The sequence of actions in the path
     * @param[in] terrain Height map of the terrain
     */
    void postProcessPath(std::vector<State> &state_sequence, std::vector<Action> &action_sequence, FastTerrainMap& terrain);

    /**
     * @brief Run the RRT-Connect planner once until the goal is found
     * @param[in] Ta Tree with root vertex at start state
     * @param[in] Tb Tree with root vertex at goal state
     * @param[in] terrain Height map of the terrain
     */
    void runRRTConnect(PlannerClass &Ta, PlannerClass &Tb, FastTerrainMap& terrain);
    
    /**
     * @brief Run the full RRT-Connect planner until the goal is found and time has expired, then post process and update statistics
     * @param[in] terrain Height map of the terrain
     * @param[in] s_start The start state of the planner
     * @param[in] s_goal The goal state of the planner
     * @param[out] state_sequence The sequence of states in the final path
     * @param[out] action_sequence The sequence of actions in the final path
     * @param[in] max_time The time after which replanning is halted
     */
    void buildRRTConnect(FastTerrainMap& terrain, State s_start, State s_goal, std::vector<State> &state_sequence, std::vector<Action> &action_sequence, double max_time);

  protected:

    /// Time horizon (in seconds) the planner is allowed to search until restarted
    double anytime_horizon;

    /// Initial guess at how quickly the planner executes (in m/s)
    const double planning_rate_estimate = 16.0; // m/s (meters planned/computation time)
    
    /// Initial anytime horizon (in seconds)
    double anytime_horizon_init;

    /// Factor by which horizon is increased if replanning is required
    double horizon_expansion_factor = 1.2;

    /// Hard maximum time allowed for the planner, returns unsuccessfully if reached
    const int max_time_solve = 20;
};

#endif