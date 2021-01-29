#ifndef BODY_FORCE_ESTIMATOR_H
#define BODY_FORCE_ESTIMATOR_H

#include <ros/ros.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/BodyForceEstimate.h>

//! Estimates body contact forces
/*!
   BodyForceEstimator is a container for all logic used in estimating force from contacts distrbuted across all links of the robot.
   It requires robot state estimates and motor commands and exposes an update method.
*/
class BodyForceEstimator {
  public:
    /**
     * @brief Constructor for BodyForceEstimator Class
     * @param[in] nh ROS NodeHandle to publish and subscribe from
     * @return Constructed object of type BodyForceEstimator
     */
    BodyForceEstimator(ros::NodeHandle nh);

    /**
     * @brief Calls ros spinOnce and pubs data at set frequency
     */
    void spin();

    /**
     * @brief Callback function to handle new state estimates
     * @param[in] Robot state message contining position and velocity for each joint and robot body
     */
    void robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg);

    /**
     * @brief Publish body force force estimates
     */
    void publishBodyForce();

    /// ROS subscriber for the robot state
    ros::Subscriber robot_state_sub_;

    /// ROS publisher for body force force estimates
    ros::Publisher body_force_pub_;

    /// Nodehandle to pub to and sub from
    ros::NodeHandle nh_;

    /// Update rate for sending and receiving data;
    double update_rate_;
};

#endif // BODY_FORCE_ESTIMATOR_H
