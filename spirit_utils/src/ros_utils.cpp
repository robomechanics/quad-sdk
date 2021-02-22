#include <spirit_utils/ros_utils.h>

double spirit_utils::getROSMessageAgeInMs(std_msgs::Header &header)
{
  ros::Time t_compare = ros::Time::now();
  return spirit_utils::getROSMessageAgeInMs(header,t_compare);
}

double spirit_utils::getROSMessageAgeInMs(std_msgs::Header &header, ros::Time &t_compare)
{
  return (header.stamp - t_compare).toSec()*1000.0;
}

bool spirit_utils::loadROSParam(std::string paramName, ParamType &varName)
{
  
}
