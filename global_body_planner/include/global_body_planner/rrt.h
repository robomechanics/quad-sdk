#ifndef RRT_H
#define RRT_H

#include "global_body_planner/planner_class.h"
#include <chrono>

#define TRAPPED 0
#define ADVANCED 1
#define REACHED 2

using namespace planning_utils;

//! A class that implements RRT sampling-based planning.
/*!
   This class builds an RRT using the PlannerClass as a data structure to maintain the tree. Methods to run the standard 
   RRT algorithm are included, as well as utility methods to process and debug.
*/
class RRTClass
{
  public:
    /**
     * @brief Constructor for RRTClass
     * @return Constructed object of type RRTClass
     */
    RRTClass();

    /**
     * @brief Destructor for RRTClass
     */
    ~RRTClass();

    /** Extend the tree towards the desired state
     * @param[in] T The PlannerClass instance containing the tree
     * @param[in] s The state to extend the tree towards
     * @param[in] terrain Height map of the terrain
     * @param[in] direction The direction with which to peform the extension (FORWARD to go away from the root vertex, REVERSE to go towards it)
     */
    virtual int extend(PlannerClass &T, State s, const PlannerConfig &planner_config, int direction);

    /**
     * @brief Get the path from the root vertex to the specified one
     * @param[in] T The PlannerClass instance containing the tree
     * @param[in] idx Index of the desired vertex
     * @return Vector of vertex indices on the path from the root of the tree to the desired
     */
    std::vector<int> pathFromStart(PlannerClass &T, int idx);
    
    /**
     * @brief Print the states in the specified path via stdout
     * @param[in] T The PlannerClass instance containing the tree
     * @param[in] path Vector of vertices in the path
     */
    void printPath(PlannerClass &T, std::vector<int> path);
    
    /**
     * @brief Execute the planner by constructing the tree until a path is found
     * @param[in] terrain Height map of the terrain
     * @param[in] s_start The start state of the planner
     * @param[in] s_goal The goal state of the planner
     * @param[out] state_sequence The sequence of states in the final path
     * @param[out] action_sequence The sequence of actions in the final path
     */
    // void buildRRT(const PlannerConfig &planner_config, State s_start, State s_goal, std::vector<State> &state_sequence, std::vector<Action> &action_sequence);
    
    /**
     * @brief Get the statistics for the planner solve
     * @param[out] plan_time The total time spent in the planner
     * @param[out] vertices_generated Number of vertices generated in the tree
     * @param[out] plan_length The length of the path in meters
     * @param[out] path_duration The duration of the path in seconds
     */
    void getStatistics(double &plan_time, int &vertices_generated, double &plan_length, double& path_duration);

    /**
     * @brief Generate a new state that can be connected to the tree and is as close as possible to the specified state
     * @param[in] s Specific state to move towards
     * @param[in] s_near Closest state in the tree to the specified state
     * @param[out] s_new New state yielded by taking an action from s_near
     * @param[out] a_new New action to take from s_near to get to s
     * @param[in] terrain Height map of the terrain
     * @param[in] direction Direction of the dynamics (either FORWARD or REVERSE)
     * @return Boolean if the new state got closer to the specified state than any other in the tree
     */
    bool newConfig(State s, State s_near, State &s_new, Action &a_new, const PlannerConfig &planner_config, int direction);

    /**
     * @brief Get the states along the specified path of vertex indices
     * @param[in] T The PlannerClass instance containing the tree
     * @param[in] path Vector of vertices in the path
     * @return The sequence of states in the path
     */
    std::vector<State> getStateSequence(PlannerClass &T, std::vector<int> path);
    
    /**
     * @brief Get the actions along the specified path of vertex indices (assumes that actions are synched with the states to which they lead)
     * @param[in] T The PlannerClass instance containing the tree
     * @param[in] path Vector of vertices in the path
     * @return The sequence of actions in the path
     */
    std::vector<Action> getActionSequence(PlannerClass &T, std::vector<int> path);

  protected:

    /// Probability with which the goal is sampled (if running vanilla RRT)
    const double prob_goal_thresh = 0.05;

    /// Boolean for if the goal has been reached
    bool goal_found = false;

    /// Total time elapsed in this planner call
    std::chrono::duration<double> elapsed_total_;

    // Total time elapsed until the first solve
    std::chrono::duration<double> elapsed_to_first_;

    /// Integer checking if a solution was computed in 5 seconds
    int success_ = 0;

    /// Number of vertices in the tree
    int num_vertices_;

    /// Path quality in meters
    double path_length_;

    /// The duration of the path in seconds
    double path_duration_;

        
        
};

#endif