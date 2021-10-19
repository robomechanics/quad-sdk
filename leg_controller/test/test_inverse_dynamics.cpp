#include <ros/ros.h>
#include <gtest/gtest.h>

#include "leg_controller/inverse_dynamics.h"
#include "leg_controller/leg_controller.h"

TEST(InverseDynamics, testConstructor) {
  InverseDynamicsController inverse_dynamics_controller();
  EXPECT_EQ(1 + 1, 2);
}

TEST(LegController, testConstructor) {
  ros::NodeHandle nh;
  LegController leg_controller(nh);
  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "leg_controller_test");

  return RUN_ALL_TESTS();
}
