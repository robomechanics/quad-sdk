#include <ros/ros.h>
#include <gtest/gtest.h>

#include "local_footstep_planner/local_footstep_planner.h"

TEST(LocalFootstepPlannerTest, testTrue) {
	ros::NodeHandle nh;
	LocalFootstepPlanner local_footstep_planner(nh);
	EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "footstep_plannertester");

	return RUN_ALL_TESTS();
}
