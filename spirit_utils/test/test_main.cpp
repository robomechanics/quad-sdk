#include <ros/ros.h>
#include <gtest/gtest.h>

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "spirit_utils_tester");

	return RUN_ALL_TESTS();
}
