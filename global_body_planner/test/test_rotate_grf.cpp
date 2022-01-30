#include "global_body_planner/planning_utils.h"
#include <ros/ros.h>
#include <gtest/gtest.h>

TEST(GlobalBodyPlannerTest, testRotateGRF) {

  Eigen::Vector3d f, surf_norm, f_test;
  f << 0, 0, 1;
  surf_norm << -1.0/sqrt(2.0), 0, 1/sqrt(2.0);
  f_test = planning_utils::rotateGRF(surf_norm, f);

  // std::cout << f_test << std::endl;
  // std::cout << surf_norm << std::endl;

  EXPECT_TRUE(f_test.isApprox(surf_norm));

}