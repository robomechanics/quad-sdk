#ifndef LEG_OVERRIDE_H
#define LEG_OVERRIDE_H

#include <ros/ros.h>
#include <spirit_msgs/LegOverride.h>
#include <spirit_msgs/BodyForceEstimate.h>

//! A template class for spirit
/*!
   PackageTemplate is a container for all of the logic utilized in the template node.
   The implementation must provide a clean and high level interface to the core algorithm
*/
class LegOverrider {
public:
    /**
     * @brief Constructor for PackageTemplate Class
     * @param[in] nh ROS NodeHandle to publish and subscribe from
     * @return Constructed object of type PackageTemplate
     */
    LegOverrider(ros::NodeHandle nh);

    /**
     * @brief Calls ros spinOnce and pubs data at set frequency
     */
    void spin();

    /**
     * @brief Callback function to handle new body force estimates
     * @param[in] Body force estimates for each joint
     */
    void bodyForceCallback(const spirit_msgs::BodyForceEstimate::ConstPtr& msg);

    /**
     * @brief Publish leg overrides
     */
    void publishLegOverride();


private:
    /// ROS subscriber for the body force estimate
    ros::Subscriber body_force_sub_;

    /// ROS Publisher
    ros::Publisher leg_override_pub_;

    /// Nodehandle to pub to and sub from
    ros::NodeHandle nh_;

    /// Update rate for sending and receiving data;
    double update_rate_;

    spirit_msgs::BodyForceEstimate::ConstPtr last_body_force_estimate_msg_;
};

#endif // LEG_OVERRIDE
