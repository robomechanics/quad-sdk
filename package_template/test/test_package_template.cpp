#include <ros/ros.h>
#include <gtest/gtest.h>

#include "package_template/package_template.h"

TEST(PackageTemplate, testTrue) {
	PackageTemplate package_template();
	EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "tester");

	return RUN_ALL_TESTS();
}
