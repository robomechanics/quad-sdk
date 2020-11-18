#ifndef SPIRIT_ROS_UTILS_H
#define SPIRIT_ROS_UTILS_H

#include <ros/ros.h>
#include <std_msgs/Header.h>

namespace spirit_utils{
  /**
   * @brief Gets the relative age of a timestamped header
   * @param[in] header ROS Header that we wish to compute the age of
   * @return Age in ms (compared to ros::Time::now())
   */
  double getROSMessageAgeInMs(std_msgs::Header &header);
  /**
   * @brief Gets the relative age of a timestamped header
   * @param[in] header ROS Header that we wish to compute the age of
   * @param[in] t_compare ROS time we wish to compare to
   * @return Age in ms (compared to t_compare)
   */
  double getROSMessageAgeInMs(std_msgs::Header &header, ros::Time &t_compare);
}

#endif