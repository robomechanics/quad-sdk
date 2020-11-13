#include <ros/ros.h>
#include <gtest/gtest.h>

#include "body_force_estimator/body_force_estimator.h"

TEST(BodyContactEstimator, testTrue) {
	ros::NodeHandle nh;
	BodyContactEstimator body_force_estimator(nh);
	EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "body_force_estimator_tester");

	return RUN_ALL_TESTS();
}
