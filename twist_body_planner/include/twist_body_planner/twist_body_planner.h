#ifndef TWIST_BODY_PLANNER_H
#define TWIST_BODY_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <spirit_msgs/BodyPlan.h>
#include <spirit_msgs/RobotState.h>


//! A twist body planning class for legged robots
/*!
   TwistBodyPlanner is a container for all of the logic utilized in the twist body planning node.
   This algorithm requires a desired twist, and will publish the resulting body plan as a BodyPlan 
   message over a topic.
*/
class TwistBodyPlanner {
  public:
    /**
     * @brief Constructor for TwistBodyPlanner Class
     * @param[in] nh Node handle
     * @return Constructed object of type TwistBodyPlanner
     */
    TwistBodyPlanner(ros::NodeHandle nh);

    /**
     * @brief Get the interpolated body trajectory and store in private member
     */
    void plan();

    /**
     * @brief Primary work function in class, called in node file for this component
     */
    void spin();

  private:

    typedef std::vector<double> State;
    typedef std::vector<double> Twist;

    /**
     * @brief Callback function to handle new desired twist data
     * @param[in] msg the message contining twist data
     */
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    /**
     * @brief Callback function to handle new robot state data
     * @param[in] msg the message contining robot state data
     */
    void robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg);

    /**
     * @brief Clear the plan member variables
     */
    void clearPlan();

    /**
     * @brief Update the body plan with the current plan
     * @param[in] t Time of state in trajectory
     * @param[in] body_state Body state
     * @param[in] body_wrench Wrench applied to body
     * @param[in] body_plan_msg Body plan message
     */
    void addStateWrenchToMsg(double t, State body_state, spirit_msgs::BodyPlan& body_plan_msg);

    /**
     * @brief Publish the current body plan
     */
    void publishPlan();

    /// Nodehandle to pub to and sub from
    ros::NodeHandle nh_;

    /// Subscriber for commanded twist messages
    ros::Subscriber cmd_vel_sub_;

    /// Subscriber for robot state messages
    ros::Subscriber robot_state_sub_;

    /// Publisher for body plan messages
    ros::Publisher body_plan_pub_;

    /// Update rate for sending and receiving data;
    double update_rate_;

    /// Handle for the map frame
    std::string map_frame_;

    /// Std vector containing the interpolated robot body plan
    std::vector< State > body_plan_;

    /// Std vector containing the interpolated time data
    std::vector<double> t_plan_;

    /// Robot starting state
    State start_state_;

    /// Robot starting state
    Twist cmd_vel_;

    /// Horizon length for the body plan
    double horizon_length_;

    /// Time of the most recent cmd_vel data
    ros::Time last_cmd_vel_msg_time_;

    /// Threshold for waiting for cmd_vel data
    double last_cmd_vel_msg_time_max_;

    /// Plan timestamp
    ros::Time plan_timestamp_;

};

#endif // TWIST_BODY_PLANNER_H
