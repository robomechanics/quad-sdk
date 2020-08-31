#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_controller.h"

TEST(SpiritController, testTrue) {
	ros::NodeHandle nh;
	SpiritController spirit_controller(nh);
	EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "controller_tester");

	return RUN_ALL_TESTS();
}
