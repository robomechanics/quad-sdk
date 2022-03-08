#include <gtest/gtest.h>
#include <ros/ros.h>

#include "global_footstep_planner/global_footstep_planner.h"

TEST(GlobalFootstepPlannerTest, testTrue) {
  ros::NodeHandle nh;
  GlobalFootstepPlanner global_footstep_planner(nh);
  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "global_footstep_planner_tester");

  return RUN_ALL_TESTS();
}
