#ifndef LEG_OVERRIDE_H
#define LEG_OVERRIDE_H

#include <quad_msgs/BodyForceEstimate.h>
#include <quad_msgs/LegOverride.h>
#include <ros/ros.h>

//! A template class for quad
/*!
   PackageTemplate is a container for all of the logic utilized in the template
   node. The implementation must provide a clean and high level interface to the
   core algorithm
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
  void bodyForceCallback(const quad_msgs::BodyForceEstimate::ConstPtr& msg);

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

  quad_msgs::BodyForceEstimate::ConstPtr last_body_force_estimate_msg_;
};

#endif  // LEG_OVERRIDE
