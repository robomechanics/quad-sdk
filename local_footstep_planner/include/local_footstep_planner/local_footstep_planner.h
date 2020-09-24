#ifndef LOCAL_FOOTSTEP_PLANNER_H
#define LOCAL_FOOTSTEP_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <spirit_msgs/BodyPlan.h>
#include <spirit_msgs/Footstep.h>
#include <spirit_msgs/FootstepPlan.h>
#include <spirit_utils/fast_terrain_map.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

//! A local footstep planning class for spirit
/*!
   FootstepPlanner is a container for all of the logic utilized in the local footstep planning node.
   The implementation must provide a clean and high level interface to the core algorithm
*/
class LocalFootstepPlanner {
  public:
    /**
     * @brief Constructor for LocalFootstepPlanner Class
     * @param Node handle
     * @return Constructed object of type LocalFootstepPlanner
     */
    LocalFootstepPlanner(ros::NodeHandle nh);

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
     * @brief Callback function to handle new body plan data
     * @param[in] nav_msgs::Path contining map data
     */
    void bodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg);

    /**
     * @brief Update the footstep plan with the current plan
     */
    void updatePlan();

    /**
     * @brief Publish the current footstep plan
     */
    void publishPlan();

    /// Subscriber for terrain map messages
    ros::Subscriber terrain_map_sub_;

    /// Subscriber for body plan messages
    ros::Subscriber body_plan_sub_;

    /// Publisher for footstep plan messages
    ros::Publisher footstep_plan_pub_;

    /// Nodehandle to pub to and sub from
    ros::NodeHandle nh_;

    /// Update rate for sending and receiving data;
    double update_rate_;

    /// Handle for the map frame
    std::string map_frame_;

    /// Struct for terrain map data
    FastTerrainMap terrain_;

    /// Typedef for body state array (TODO replace with utils def) 
    typedef std::array<double, 9> BodyState;

    /// Typedef for footstep state array (TODO replace with utils def)
    typedef std::array<double, 5> FootstepState;

    /// Std vector containing robot body plan
    std::vector<BodyState> body_plan_;

    /// Std vector containing robot footstep plan
    std::vector<FootstepState> footstep_plan_;

    /// Std vector containing time data
    std::vector<double> t_plan_;

};


#endif // LOCAL_FOOTSTEP_PLANNER_H
