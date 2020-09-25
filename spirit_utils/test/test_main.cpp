#include <ros/ros.h>
#include <gtest/gtest.h>

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "tester");

	return RUN_ALL_TESTS();
}
