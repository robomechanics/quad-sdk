#ifndef LOCAL_FOOTSTEP_PLANNER_H
#define LOCAL_FOOTSTEP_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>

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
    void bodyPlanCallback(const nav_msgs::Path::ConstPtr& msg);

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

    /// Typedef for ground struct (TODO replace with utils def)
    typedef struct
    {
      std::vector<double> x_data;
      std::vector<double> y_data;
      std::vector<std::vector<double>> z_data;
      std::vector<std::vector<double>> dx_data;
      std::vector<std::vector<double>> dy_data;
      std::vector<std::vector<double>> dz_data;
      int x_size;
      int y_size;
      int z_size;
    } Terrain;

    /// Typedef for state array (TODO replace with utils def) 
    typedef std::array<double, 8> State;

    /// Struct for terrain map data
    Terrain terrain_;

    /// Std vector containing robot body plan
    std::vector<State> body_plan_;

};


#endif // LOCAL_FOOTSTEP_PLANNER_H
