#include <ros/ros.h>
#include <gtest/gtest.h>

#include "contact_detection/contact_detection.h"

TEST(ContactDetection, testTrue) {
	ros::NodeHandle nh;
	ContactDetection cd(nh);
	EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "contact_deetection_tester");

	return RUN_ALL_TESTS();
}
