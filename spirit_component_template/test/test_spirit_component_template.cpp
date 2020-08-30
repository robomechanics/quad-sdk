#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_component_template.h"

TEST(SpiritComponentTest, testTrue) {
	ros::NodeHandle nh;
	SpiritComponentTemplate spirit_component(nh);
	EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "tester");

	return RUN_ALL_TESTS();
}
