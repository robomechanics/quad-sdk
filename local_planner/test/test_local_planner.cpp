#include <ros/ros.h>
#include <gtest/gtest.h>

#include "local_planner/local_planner.h"

TEST(LocalPlannerTest, testTrue) {
  ros::NodeHandle nh;
  LocalPlanner local_planner(nh);
  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "local_planner_tester");

  return RUN_ALL_TESTS();
}
