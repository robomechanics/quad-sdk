#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "global_body_planner/global_body_planner_test_fixture.hpp"

namespace {

planning_utils::State makeState(double x, double y, double z, double vx,
                                double vy, double vz) {
  planning_utils::State s;
  s.pos << x, y, z;
  s.vel << vx, vy, vz;
  return s;
}

planning_utils::TimedPoseConstraint makeConstraint(
    const Eigen::Vector3d& pos, double yaw,
    const Eigen::Vector3d& half_extents, double t_start, double t_end) {
  planning_utils::TimedPoseConstraint c;
  c.pos = pos;
  c.yaw = yaw;
  c.half_extents = half_extents;
  c.t_start = t_start;
  c.t_end = t_end;
  return c;
}

}  // namespace

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

TEST_F(GlobalBodyPlannerTestFixture, ValidStateRejectsMapVelocityAndHeight) {
  double max_valid_z = 0.0;

  EXPECT_FALSE(planning_utils::isValidState(
      makeState(20.0, 0.0, 0.3, 0.0, 0.0, 0.0), planner_config_,
      LEAP_STANCE, max_valid_z));

  EXPECT_FALSE(planning_utils::isValidState(
      makeState(0.0, 0.0, 0.3, planner_config_.v_max + 1.0, 0.0, 0.0),
      planner_config_, LEAP_STANCE, max_valid_z));

  EXPECT_FALSE(planning_utils::isValidState(
      makeState(0.0, 0.0, planner_config_.h_min * 0.5, 0.0, 0.0, 0.0),
      planner_config_, LEAP_STANCE, max_valid_z));

  EXPECT_FALSE(planning_utils::isValidState(
      makeState(0.0, 0.0, planner_config_.h_max + 1.0, 0.0, 0.0, 0.0),
      planner_config_, CONNECT, max_valid_z));
}

TEST_F(GlobalBodyPlannerTestFixture, ValidStateRejectsLowTraversability) {
  for (grid_map::GridMapIterator it(terrain_grid_map_); !it.isPastEnd();
       ++it) {
    terrain_grid_map_.at("traversability", *it) = 0.0;
  }
  terrain_.loadDataFromGridMap(terrain_grid_map_);
  planner_config_.terrain = terrain_;
  planner_config_.terrain_grid_map = terrain_grid_map_;

  const planning_utils::State s = makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0);

  EXPECT_FALSE(planning_utils::isValidState(s, planner_config_, LEAP_STANCE));
  EXPECT_TRUE(planning_utils::isValidState(s, planner_config_, FLIGHT));
}

TEST_F(GlobalBodyPlannerTestFixture,
       StateActionPairUsesDynamicConstraintTimeWindows) {
  planning_utils::State s0 = makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  planning_utils::State s1 = makeState(1.0, 0.0, 0.3, 0.0, 0.0, 0.0);

  planning_utils::StateActionResult connect_result;
  GBPL gbpl;
  ASSERT_EQ(gbpl.attemptConnect(s0, s1, 2.0, connect_result, planner_config_,
                                FORWARD),
            REACHED);

  planner_config_.dynamic_constraints.push_back(makeConstraint(
      Eigen::Vector3d(0.5, 0.0, 0.3), 0.0, Eigen::Vector3d(0.5, 0.25, 0.2),
      0.8, 1.2));

  planning_utils::StateActionResult result;
  EXPECT_TRUE(planning_utils::isValidStateActionPair(
      s0, connect_result.a_new, result, planner_config_, 10.0));
  EXPECT_FALSE(planning_utils::isValidStateActionPair(
      s0, connect_result.a_new, result, planner_config_, 0.0));
  EXPECT_GT(result.t_new, 0.0);
  EXPECT_LT(result.t_new, connect_result.a_new.t_s_leap);
}

TEST_F(GlobalBodyPlannerTestFixture, ActionValidityRejectsBadForcesAndTimes) {
  planning_utils::Action a;
  a.grf_0 << 0.0, 0.0, 1.0;
  a.grf_f << 0.0, 0.0, 1.0;
  a.t_s_leap = 0.2;
  a.t_f = 0.0;
  a.t_s_land = 0.0;
  a.dz_0 = 0.0;
  a.dz_f = 0.0;

  EXPECT_TRUE(planning_utils::isValidAction(a, planner_config_));

  a.t_s_leap = 0.0;
  EXPECT_FALSE(planning_utils::isValidAction(a, planner_config_));
  a.t_s_leap = 0.2;

  a.grf_0 << planner_config_.grf_max, 0.0, planner_config_.grf_max;
  EXPECT_FALSE(planning_utils::isValidAction(a, planner_config_));
  a.grf_0 << 1.0, 0.0, 1.0;
  EXPECT_FALSE(planning_utils::isValidAction(a, planner_config_));
}

TEST(GlobalBodyPlannerUtilsTest, StateConversionsDistancesAndDirectionFlip) {
  planning_utils::State s = makeState(1.0, 2.0, 3.0, 0.4, -0.5, 0.6);
  planning_utils::FullState full =
      planning_utils::stateToFullState(s, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0);
  EXPECT_TRUE(planning_utils::fullStateToState(full).isApprox(s));

  Eigen::VectorXd eig = planning_utils::fullStateToEigen(full);
  planning_utils::FullState round_trip;
  planning_utils::eigenToFullState(eig, round_trip);
  EXPECT_TRUE(round_trip.pos.isApprox(full.pos));
  EXPECT_TRUE(round_trip.vel.isApprox(full.vel));

  std::vector<double> values{1.0, 2.0, 3.0, 0.1, 0.2, 0.3,
                             0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
  planning_utils::FullState from_vector;
  planning_utils::vectorToFullState(values, from_vector);
  EXPECT_NEAR(from_vector.pos.x(), 1.0, 1e-9);
  EXPECT_NEAR(from_vector.ang_vel.z(), 0.9, 1e-9);

  planning_utils::State s2 = makeState(4.0, 6.0, 3.0, 1.4, -0.5, 0.6);
  EXPECT_NEAR(planning_utils::poseDistance(s, s2), 5.0, 1e-9);
  EXPECT_GT(planning_utils::stateDistance(s, s2),
            planning_utils::poseDistance(s, s2));

  planning_utils::flipDirection(s);
  EXPECT_NEAR(s.vel.x(), -0.4, 1e-9);
  EXPECT_NEAR(s.vel.y(), 0.5, 1e-9);
}

TEST(GlobalBodyPlannerUtilsTest, ActionDirectionFlipSwapsLeapAndLanding) {
  planning_utils::Action a;
  a.grf_0 << 1.0, 2.0, 3.0;
  a.grf_f << 4.0, 5.0, 6.0;
  a.t_s_leap = 0.1;
  a.t_f = 0.2;
  a.t_s_land = 0.3;
  a.dz_0 = -1.0;
  a.dz_f = 2.0;

  planning_utils::flipDirection(a);

  EXPECT_TRUE(a.grf_0.isApprox(Eigen::Vector3d(4.0, 5.0, 6.0)));
  EXPECT_TRUE(a.grf_f.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_NEAR(a.t_s_leap, 0.3, 1e-9);
  EXPECT_NEAR(a.t_s_land, 0.1, 1e-9);
  EXPECT_NEAR(a.dz_0, -2.0, 1e-9);
  EXPECT_NEAR(a.dz_f, 1.0, 1e-9);
}

TEST(GlobalBodyPlannerUtilsTest, VectorPoseDistanceRejectsShortInputs) {
  EXPECT_THROW(planning_utils::poseDistance(std::vector<double>{1.0},
                                            std::vector<double>{1.0, 2.0}),
               std::runtime_error);
}
