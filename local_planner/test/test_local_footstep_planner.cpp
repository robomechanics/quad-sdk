#include <gtest/gtest.h>
#include <ros/ros.h>

#include "local_planner/local_footstep_planner.h"

TEST(LocalFootstepPlannerTest, testTrue) {
  LocalFootstepPlanner local_footstep_planner();
  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "local_footstep_planner_tester");

  return RUN_ALL_TESTS();
}
