#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_estimator.h"

TEST(SpiritEstimator, testTrue) {
	ros::NodeHandle nh;
	SpiritEstimator spirit_estimator(nh);
	EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "estimator_tester");

	return RUN_ALL_TESTS();
}
