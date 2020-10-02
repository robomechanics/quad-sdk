#ifndef GLOBAL_BODY_PLANNER_H
#define GLOBAL_BODY_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <spirit_msgs/BodyPlan.h>
#include "global_body_planner/functions.h"
#include "spirit_utils/fast_terrain_map.h"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

//! A global body planning class for spirit
/*!
   GlobalBodyPlanner is a container for all of the logic utilized in the global body planning node.
   This algorithm requires an height map of the terrain as a GridMap message type, and will publish
   the global body plan as a BodyPlan message over a topic. It will also publish the 
*/
class GlobalBodyPlanner {
  public:
    /**
     * @brief Constructor for GlobalBodyPlanner Class
     * @param Node handle
     * @return Constructed object of type GlobalBodyPlanner
     */
    GlobalBodyPlanner(ros::NodeHandle nh);

    /**
     * @brief Call the correct planning class and compute statistics
     */
    void callPlanner();

    /**
     * @brief Primary work function in class, called in node file for this component
     */
    void spin();

  private:
    /**
     * @brief Callback function to handle new terrain map data
     * @param[in] grid_map_msgs::GridMap contining map data
     */
    void terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg);

    /**
     * @brief Set the start and goal states of the planner
     */
    void setStartAndGoalStates();

    /**
     * @brief Clear the plan member variables
     */
    void clearPlan();

    /**
     * @brief Update the body plan with the current plan
     * @param[in] Time of state in trajectory
     * @param[in] Body state
     * @param[in] Body plan message
     */
    void addBodyStateToMsg(double t, State body_state, spirit_msgs::BodyPlan& body_plan_msg);

    /**
     * @brief Publish the current body plan
     */
    void publishPlan();

    /**
     * @brief Wait until a map message has been received and processed
     */
    void waitForMap();

    /// Subscriber for terrain map messages
    ros::Subscriber terrain_map_sub_;

    /// Publisher for body plan messages
    ros::Publisher body_plan_pub_;

    /// Publisher for discrete states in body plan messages
    ros::Publisher discrete_body_plan_pub_;

    /// Nodehandle to pub to and sub from
    ros::NodeHandle nh_;

    /// Update rate for sending and receiving data;
    double update_rate_;

    /// Number of times to call the planner
    int num_calls_;

    /// Time after which replanning is halted;
    double replan_time_limit_;

    /// Handle for the map frame
    std::string map_frame_;

    /// Struct for terrain map data
    FastTerrainMap terrain_;

    /// Std vector containing the interpolated robot body plan
    std::vector<State> body_plan_;

    /// Std vector containing the interpolated time data
    std::vector<double> t_plan_;

    /// Robot starting state
    State robot_start_;

    /// Robot goal state
    State robot_goal_;
    
    /// Sequence of discrete states in the plan
    std::vector<State> state_sequence_;

    /// Sequence of discrete actions in the plan
    std::vector<Action> action_sequence_;

    /// Vector of cost instances in each planning call (nested STL vectors)
    std::vector<std::vector<double> > cost_vectors_;

    /// Vector of time instances of cost data for each planning call (nested STL vectors)
    std::vector<std::vector<double> > cost_vectors_times_;

    /// Vector of solve times for each planning call
    std::vector<double> solve_time_info_;

    /// Vector of number of vertices for each planning call
    std::vector<int> vertices_generated_info_;

};


#endif // GLOBAL_BODY_PLANNER_H
