#include <ros/ros.h>
#include <gtest/gtest.h>

#include "estimator/estimator.h"

TEST(Estimator, testTrue) {
	Estimator estimator();
	EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "estimator_tester");

	return RUN_ALL_TESTS();
}
