#ifndef LOCAL_FOOTSTEP_PLANNER_H
#define LOCAL_FOOTSTEP_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <spirit_msgs/BodyPlan.h>
#include <spirit_msgs/FootState.h>
#include <spirit_msgs/MultiFootState.h>
#include <spirit_msgs/MultiFootPlanContinuous.h>
#include <spirit_msgs/FootPlanDiscrete.h>
#include <spirit_msgs/MultiFootPlanDiscrete.h>
#include <spirit_utils/fast_terrain_map.h>
#include <spirit_utils/function_timer.h>
#include <spirit_utils/math_utils.h>
#include <spirit_utils/ros_utils.h>
#include <spirit_utils/kinematics.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

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
     * @brief Callback function to handle new robot state data
     * @param[in] msg the message contining robot state data
     */
    void robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg);

    /**
     * @brief Compute the amount of time until the next flight phase
     * @param[in] t Current time
     * @return Time until flight
     */
    double computeTimeUntilNextFlight(double t);

    /**
     * @brief Update the discrete footstep plan with the current plan
     */
    void updateDiscretePlan();

    /**
     * @brief Update the continuous foot plan to match the discrete
     */
    void updateContinuousPlan();

    /**
     * @brief Publish the current footstep plan
     */
    void publishDiscretePlan();

    /**
     * @brief Publish the continuous foot plan to match the discrete
     */
    void publishContinuousPlan();

    /**
     * @brief Wait until map and plan messages have been received and processed
     */
    void waitForData();

    /// Subscriber for terrain map messages
    ros::Subscriber terrain_map_sub_;

    /// Subscriber for body plan messages
    ros::Subscriber body_plan_sub_;

    /// Subscriber for body plan messages
    ros::Subscriber robot_state_sub_;

    /// Publisher for discrete foot plan messages
    ros::Publisher foot_plan_discrete_pub_;

    /// Publisher for continuous
    ros::Publisher foot_plan_continuous_pub_;

    /// Nodehandle to pub to and sub from
    ros::NodeHandle nh_;

    /// Topic name for terrain map (needed to ensure data has been received)
    std::string terrain_map_topic_;

    /// Topic name for robot state data (needed to ensure data has been received)
    std::string body_plan_topic_;

    /// Update rate for sending and receiving data;
    double update_rate_;

    /// Handle for the map frame
    std::string map_frame_;

    /// Struct for terrain map data
    FastTerrainMap terrain_;

    /// Define the body state data structure
    typedef std::vector<double> BodyState;

    /// Define the footstep state data structure
    typedef std::vector<double> FootstepState;
    
    /// Std vector containing robot body plan
    std::vector<BodyState> body_plan_;

    /// Std vector containing robot GRFs
    std::vector<Eigen::Vector3d> grf_plan_;

    /// Std vector containing primitive ids for the plan
    std::vector<int> primitive_id_plan_;

    /// Std vector containing robot footstep plan
    std::vector<std::vector<FootstepState> > footstep_plan_;

    /// Std vector containing time data
    std::vector<double> t_plan_;

    /// ROS Timestamp of plan (should match body plan)
    ros::Time plan_timestamp_;

    /// Current body plan
    spirit_msgs::BodyPlan::ConstPtr body_plan_msg_;

    /// Current robot state
    spirit_msgs::RobotState::ConstPtr robot_state_msg_;

    /// Current continuous footstep plan
    spirit_msgs::MultiFootPlanContinuous multi_foot_plan_continuous_msg_;

    /// Boolean for whether or not to replan to accomodate an updated body plan
    bool update_flag_;

    /// Number of feet
    const int num_feet_ = 4;

    /// Weighting on the projection of the grf
    double grf_weight_;

    /// Maximum horizon with which to plan footsteps
    double max_footstep_horizon_;

    /// Number of cycles to plan
    int num_cycles_;

    /// Ground clearance
    double period_;

    /// Ground clearance
    double ground_clearance_;

    /// Interpolation timestep for swing leg
    double interp_dt_;

    /// Primitive ids - FLIGHT
    const int FLIGHT = 0;

    /// Primitive ids - STANCE
    const int STANCE = 1;

    /// Primitive ids - CONNECT_STANCE
    const int CONNECT_STANCE = 2;

};


#endif // LOCAL_FOOTSTEP_PLANNER_H
