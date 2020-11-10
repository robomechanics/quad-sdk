#include <ros/ros.h>
#include <std_msgs/Header.h>

namespace spirit_utils{
  double getROSMessageAgeInMs(std_msgs::Header &header);
  double getROSMessageAgeInMs(std_msgs::Header &header, ros::Time &t_compare);
}