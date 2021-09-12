#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_utils/robot_proxy.h"

TEST(RobotProxyTest, testTrue) {
  ros::NodeHandle nh;
  RobotProxy robot_proxy(nh);
  EXPECT_EQ(1 + 1, 2);
}