#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_utils/class1.h"

TEST(Class1, testTrue) {
	Class1 c1;
	EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "tester");

	return RUN_ALL_TESTS();
}
