#include <ros/ros.h>
#include <gtest/gtest.h>

#include "inverse_dynamics/inverse_dynamics.h"

TEST(InverseDynamicsTest, testTrue) {
  ros::NodeHandle nh;
  InverseDynamics inverse_dynamics(nh);
  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "inverse_dynamics_test");

  return RUN_ALL_TESTS();
}
