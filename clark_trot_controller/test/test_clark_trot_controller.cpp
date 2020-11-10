#include <ros/ros.h>
#include <gtest/gtest.h>

#include "clark_trot_controller/clark_trot_controller.h"

TEST(ClarkTrotController, testTrue) {
	ros::NodeHandle nh;
	ClarkTrotController clark_trot_controller(nh);
	EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "clark_trot_controller_tester");

	return RUN_ALL_TESTS();
}
