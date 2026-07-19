#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "global_body_planner/global_body_planner_test_fixture.hpp"
#include "test_helpers.hpp"

using global_body_planner_test::makeConstraint;
using global_body_planner_test::makeState;

TEST(GlobalBodyPlannerConstraintTest, ObbIntersectHandlesOverlapAndGaps) {
  const Eigen::Vector3d half_extents(0.5, 0.25, 0.2);

  EXPECT_TRUE(planning_utils::obbIntersect(
      Eigen::Vector3d(0.0, 0.0, 0.3), 0.0, half_extents,
      Eigen::Vector3d(0.4, 0.0, 0.3), 0.0, half_extents));
  EXPECT_FALSE(planning_utils::obbIntersect(
      Eigen::Vector3d(0.0, 0.0, 0.3), 0.0, half_extents,
      Eigen::Vector3d(2.0, 0.0, 0.3), 0.0, half_extents));
  EXPECT_FALSE(planning_utils::obbIntersect(
      Eigen::Vector3d(0.0, 0.0, 0.3), 0.0, half_extents,
      Eigen::Vector3d(0.0, 0.0, 1.0), 0.0, half_extents));
}

TEST(GlobalBodyPlannerConstraintTest, ObbIntersectTreatsTouchingAsOverlap) {
  const Eigen::Vector3d half_extents(0.5, 0.25, 0.2);

  EXPECT_TRUE(planning_utils::obbIntersect(
      Eigen::Vector3d(0.0, 0.0, 0.3), 0.0, half_extents,
      Eigen::Vector3d(1.0, 0.0, 0.3), 0.0, half_extents));
}

TEST_F(GlobalBodyPlannerTestFixture,
       DynamicConstraintsHonorTimeWindowsAndStaticFallback) {
  planner_config_.dynamic_constraints.push_back(makeConstraint(
      Eigen::Vector3d(0.0, 0.0, 0.3), 0.0, Eigen::Vector3d(0.5, 0.25, 0.2),
      1.0, 2.0));
  const planning_utils::State s = makeState(0.0, 0.0, 0.3, 1.0, 0.0, 0.0);

  EXPECT_FALSE(planning_utils::failsRobotConstraint(s, 0.5, planner_config_));
  EXPECT_TRUE(planning_utils::failsRobotConstraint(s, 1.0, planner_config_));
  EXPECT_TRUE(planning_utils::failsRobotConstraint(s, 2.0, planner_config_));
  EXPECT_FALSE(planning_utils::failsRobotConstraint(s, 2.5, planner_config_));
  EXPECT_TRUE(planning_utils::failsRobotConstraint(
      s, std::numeric_limits<double>::quiet_NaN(), planner_config_));
}

TEST_F(GlobalBodyPlannerTestFixture, ValidityStatsCountConstraintRejections) {
  planner_config_.dynamic_constraints.push_back(makeConstraint(
      Eigen::Vector3d(0.0, 0.0, 0.3), 0.0, Eigen::Vector3d(0.5, 0.25, 0.2),
      0.0, 10.0));
  const planning_utils::State s = makeState(0.0, 0.0, 0.3, 1.0, 0.0, 0.0);
  double max_valid_z = 0.0;

  planning_utils::resetValidityStats();
  EXPECT_FALSE(planning_utils::isValidState(s, planner_config_, LEAP_STANCE,
                                            max_valid_z, 1.0));
  const planning_utils::ValidityStats stats =
      planning_utils::getValidityStats();

  EXPECT_EQ(stats.total, 1);
  EXPECT_EQ(stats.constraint_rejects, 1);
}
