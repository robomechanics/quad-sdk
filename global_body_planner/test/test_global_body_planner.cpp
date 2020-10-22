#include <ros/ros.h>
#include <gtest/gtest.h>

#include "global_body_planner/global_body_planner.h"

TEST(GlobalBodyPlannerTest, testTrue) {
  ros::NodeHandle nh;
  GlobalBodyPlanner global_body_planner(nh);
  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "global_body_planner_tester");

  return RUN_ALL_TESTS();
}
