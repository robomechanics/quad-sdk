#include <ros/ros.h>
#include <gtest/gtest.h>

#include "leg_controller/inverse_dynamics.h"
#include "leg_controller/grf_pid_controller.h"
#include "leg_controller/joint_controller.h"
#include "leg_controller/leg_controller_interface.h"


TEST(LegController, testConstructorLegControllerInterface) {
  ros::NodeHandle nh;
  LegControllerInterface leg_controller(nh);
  EXPECT_EQ(1 + 1, 2);
}

TEST(LegController, testConstructorInverseDynamicsController) {
  InverseDynamicsController inverse_dynamics_controller;
  EXPECT_EQ(1 + 1, 2);
}

TEST(JointController, testConstructorJointController) {
  JointController joint_controller;
  EXPECT_EQ(1 + 1, 2);
}

TEST(GrfPidController, testConstructorGrfPidController) {
  GrfPidController grf_pid_controller;
  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "leg_controller_test");

  return RUN_ALL_TESTS();
}
