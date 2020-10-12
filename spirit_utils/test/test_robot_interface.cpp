#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_utils/robot_interface.h"

TEST(RobotInterfaceTest, testTrue) {
	ros::NodeHandle nh;
	RobotInterface robot_interface(nh);
	EXPECT_EQ(1 + 1, 2);
}