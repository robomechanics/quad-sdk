#ifndef SPIRIT_ROS_UTILS_H
#define SPIRIT_ROS_UTILS_H

#include <ros/ros.h>
#include <std_msgs/Header.h>

namespace spirit_utils{
  /**
   * @brief Gets the relative age of a timestamped header
   * @param[in] header ROS Header that we wish to compute the age of
   * @param[in] t_compare ROS time we wish to compare to
   * @return Age in ms (compared to t_compare)
   */
  inline double getROSMessageAgeInMs(std_msgs::Header header, ros::Time t_compare)
  {
    return (header.stamp - t_compare).toSec()*1000.0;
  }

  /**
   * @brief Gets the relative age of a timestamped header
   * @param[in] header ROS Header that we wish to compute the age of
   * @return Age in ms (compared to ros::Time::now())
   */
  inline double getROSMessageAgeInMs(std_msgs::Header header)
  {
    ros::Time t_compare = ros::Time::now();
    return spirit_utils::getROSMessageAgeInMs(header,t_compare);
  }

  /**
   * @brief Load ros parameter into class variable
   * @param[in] nh ROS nodehandle
   * @param[in] paramName string storing key of param in rosparam server
   * @param[in] varName address of variable to store loaded param
   * @return boolean success
   */
  template <class ParamType>
  inline bool loadROSParam(ros::NodeHandle nh, std::string paramName, ParamType &varName)
  {
    if(!nh.getParam(paramName, varName))
    {
      ROS_ERROR("Can't find param %s from parameter server", paramName.c_str());
      return false;
    }
    return true;
  }

  /**
   * @brief Load ros parameter into class variable
   * @param[in] nh ROS nodehandle
   * @param[in] paramName string storing key of param in rosparam server
   * @param[in] varName address of variable to store loaded param
   * @param[in] defaultVal default value to use if rosparam server doesn't contain key
   * @return boolean (true if found rosparam, false if loaded default)
   */
  template <class ParamType>
  inline bool loadROSParamDefault(ros::NodeHandle nh, std::string paramName, ParamType &varName, ParamType defaultVal)
  {
    if(!nh.getParam(paramName, varName))
    {
      varName = defaultVal;
      ROS_INFO("Can't find param %s on rosparam server, loading default value.",paramName.c_str());
      return false;
    }
    return true;
  }
}

#endif