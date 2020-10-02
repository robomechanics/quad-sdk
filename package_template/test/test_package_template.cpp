#include <ros/ros.h>
#include <gtest/gtest.h>

#include "package_template/package_template.h"

TEST(PackageTemplate, testTrue) {
	ros::NodeHandle nh;
	PackageTemplate package_template(nh);
	EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "package_template_tester");

	return RUN_ALL_TESTS();
}
