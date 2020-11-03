#ifndef GLOBAL_BODY_PLANNER_H
#define GLOBAL_BODY_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <spirit_msgs/BodyPlan.h>

#include "global_body_planner/planning_utils.h"
#include "global_body_planner/planner_class.h"
#include "global_body_planner/rrt_star_connect.h"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>


using namespace planning_utils;

//! A global body planning class for legged robots
/*!
   GlobalBodyPlanner is a container for all of the logic utilized in the global body planning node.
   This algorithm requires an height map of the terrain as a GridMap message type, and will publish
   the global body plan as a BodyPlan message over a topic. It will also publish the discrete states
   used by the planner (from which the full path is interpolated).
*/
class GlobalBodyPlanner {
  public:
    /**
     * @brief Constructor for GlobalBodyPlanner Class
     * @param[in] nh Node handle
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
     * @param[in] msg the message contining map data
     */
    void terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg);

    /**
     * @brief Clear the plan member variables
     */
    void clearPlan();

    /**
     * @brief Update the body plan with the current plan
     * @param[in] t Time of state in trajectory
     * @param[in] body_state Body state
     * @param[in] body_plan_msg Body plan message
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

    /// Algorithm for planner to run (rrt-connect or rrt-star-connect)
    std::string algorithm_;

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
    std::vector<double> start_state_;

    /// Robot goal state
    std::vector<double> goal_state_;
    
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
