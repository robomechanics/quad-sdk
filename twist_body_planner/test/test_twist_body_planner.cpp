#include <gtest/gtest.h>
#include <ros/ros.h>

#include "twist_body_planner/twist_body_planner.h"

TEST(TwistBodyPlannerTest, testTrue) {
  ros::NodeHandle nh;
  TwistBodyPlanner twist_body_planner(nh);
  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "twist_body_planner_tester");

  return RUN_ALL_TESTS();
}
