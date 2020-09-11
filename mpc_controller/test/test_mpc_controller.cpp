#include <ros/ros.h>
#include <gtest/gtest.h>

#include "mpc_controller/mpc_controller.h"

TEST(MPCController, testTrue) {
	ros::NodeHandle nh;
	MPCController mpc_controller(nh);
	EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mpc_controller_tester");

	return RUN_ALL_TESTS();
}
