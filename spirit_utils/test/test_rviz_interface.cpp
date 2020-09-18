#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_utils/rviz_interface.h"

TEST(RVizInterfaceTest, testTrue) {
	ros::NodeHandle nh;
	RVizInterface rviz_interface(nh);
	EXPECT_EQ(1 + 1, 2);
}

// int main(int argc, char** argv) {
// 	testing::InitGoogleTest(&argc, argv);
// 	ros::init(argc, argv, "tester");

// 	return RUN_ALL_TESTS();
// }
