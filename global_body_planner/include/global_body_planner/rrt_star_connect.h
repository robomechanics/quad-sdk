#ifndef RRTSTARCONNECT_H
#define RRTSTARCONNECT_H

#include "global_body_planner/rrt_connect.h"

using namespace planning_utils;

//! A class that implements RRT*-Connect sampling-based planning.
/*!
   This class inherits the RRTConnectClass, and modifies the extend function to rewire nearby states to reduce path length.
*/
class RRTStarConnectClass : public RRTConnectClass
{
  public:
    /**
     * @brief Constructor for RRTStarConnectClass
     * @return Constructed object of type RRTStarConnectClass
     */
    RRTStarConnectClass();

    /**
     * @brief Destructor for RRTStarConnectClass
     */
    ~RRTStarConnectClass();

    /** Extend the tree towards the desired state and rewire nearby states
     * @param[in] T The PlannerClass instance containing the tree
     * @param[in] s The state to extend the tree towards
     * @param[in] terrain Height map of the terrain
     * @param[in] direction The direction with which to peform the extension (FORWARD to go away from the root vertex, REVERSE to go towards it)
     */
    int extend(PlannerClass &T, State s, FastTerrainMap& terrain, int direction);

    /** Get the path from the vertex of the two trees through the specified vertex
     * @param[in] Ta Tree with root vertex at start state
     * @param[in] Tb Tree with root vertex at goal state
     * @param[in] shared_a_idx The index of the specified vertex in Tree A
     * @param[in] shared_b_idx The index of the specified vertex in Tree B
     * @param[in] state_sequence The sequence of states in the path
     * @param[in] action_sequence The sequence of actions in the path
     */
    void getStateAndActionSequences(PlannerClass &Ta, PlannerClass &Tb, 
        int shared_a_idx, int shared_b_idx, 
        std::vector<State> &state_sequence, std::vector<Action> &action_sequence);

    /**
     * @brief Run the RRT*-Connect planner until the goal is found and time has expired, then post process and update statistics
     * @param[in] terrain Height map of the terrain
     * @param[in] s_start The start state of the planner
     * @param[in] s_goal The goal state of the planner
     * @param[out] state_sequence The sequence of states in the final path
     * @param[out] action_sequence The sequence of actions in the final path
     * @param[in] max_time The time after which rewiring is halted
     */
    void buildRRTStarConnect(FastTerrainMap& terrain, State s_start, State s_goal,
        std::vector<State> &state_sequence, std::vector<Action> &action_sequence, double max_time);

  protected:

    /// Radius of neighborhood for rewiring
    const double delta = 3.0;
};

#endif