#include <ros/ros.h>
#include <gtest/gtest.h>

#include "controller/controller.h"

TEST(Controller, testTrue) {
	Controller controller();
	EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "controller_tester");

	return RUN_ALL_TESTS();
}
