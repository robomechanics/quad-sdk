#include <ros/ros.h>
#include <gtest/gtest.h>

#include "robot_driver/inverse_dynamics.h"
#include "robot_driver/grf_pid_controller.h"
#include "robot_driver/joint_controller.h"
#include "robot_driver/robot_driver.h"


TEST(LegController, testConstructorRobotDriver) {
  ros::NodeHandle nh;
  RobotDriver robot_driver(nh);
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
  ros::init(argc, argv, "robot_driver_test");

  return RUN_ALL_TESTS();
}
