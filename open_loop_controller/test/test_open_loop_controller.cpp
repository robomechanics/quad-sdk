#include <gtest/gtest.h>
#include <ros/ros.h>

#include "open_loop_controller/open_loop_controller.h"

TEST(OpenLoopController, testTrue) {
  ros::NodeHandle nh;
  OpenLoopController open_loop_controller(nh);
  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "open_loop_controller_tester");

  return RUN_ALL_TESTS();
}
