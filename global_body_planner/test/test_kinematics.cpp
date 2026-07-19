#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "global_body_planner/gbpl.hpp"
#include "global_body_planner/global_body_planner_test_fixture.hpp"
#include "test_helpers.hpp"

using global_body_planner_test::kTol;
using global_body_planner_test::makeState;

TEST(GlobalBodyPlannerKinematicsTest, RotateGrfAlignsWithSurfaceNormal) {
  Eigen::Vector3d f;
  f << 0.0, 0.0, 1.0;
  Eigen::Vector3d surface_normal;
  surface_normal << -1.0 / std::sqrt(2.0), 0.0, 1.0 / std::sqrt(2.0);

  const Eigen::Vector3d rotated = planning_utils::rotateGRF(surface_normal, f);

  EXPECT_TRUE(rotated.isApprox(surface_normal));
}

TEST_F(GlobalBodyPlannerTestFixture, ConnectActionMatchesInterpolation) {
  planning_utils::State s1 = makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  planning_utils::State s2 = s1;
  s2.pos[0] += 1.0;

  planning_utils::StateActionResult result;
  GBPL gbpl;
  const int connect_result =
      gbpl.attemptConnect(s1, s2, 2.0, result, planner_config_, FORWARD);

  EXPECT_EQ(connect_result, REACHED);
  EXPECT_TRUE(s2.isApprox(result.s_new));
  EXPECT_NEAR(result.length, 1.0, kTol);
  EXPECT_DOUBLE_EQ(result.t_new, 2.0);

  std::vector<planning_utils::State> interp_plan;
  std::vector<planning_utils::GRF> interp_grf;
  std::vector<double> interp_t;
  std::vector<int> interp_primitive_id;
  std::vector<double> interp_length{0.0};

  const planning_utils::State applied =
      planning_utils::applyAction(s1, result.a_new, planner_config_);
  const planning_utils::State interpolated = planning_utils::interpStateActionPair(
      s1, result.a_new, 0.0, 0.03, interp_plan, interp_grf, interp_t,
      interp_primitive_id, interp_length, planner_config_);

  EXPECT_TRUE(s2.isApprox(applied));
  EXPECT_TRUE(s2.isApprox(interpolated));
}

TEST_F(GlobalBodyPlannerTestFixture, ConnectActionWorksOnElevatedTerrain) {
  updateTerrainHeight(5.0);

  planning_utils::State s1 = makeState(0.0, 0.0, 5.3, 0.0, 0.0, 0.0);
  planning_utils::State s2 = s1;
  s2.pos[0] += 1.0;

  planning_utils::StateActionResult result;
  GBPL gbpl;
  const int connect_result =
      gbpl.attemptConnect(s1, s2, 2.0, result, planner_config_, FORWARD);

  EXPECT_EQ(connect_result, REACHED);
  EXPECT_TRUE(s2.isApprox(result.s_new));
  EXPECT_NEAR(result.length, 1.0, kTol);
  EXPECT_DOUBLE_EQ(result.t_new, 2.0);
}

TEST_F(GlobalBodyPlannerTestFixture, ConnectActionTracksSlopeHeight) {
  const double grade = 0.1;
  const double slope = std::atan(grade);
  updateTerrainSlope(grade);

  planning_utils::State s1 = makeState(0.0, 0.0, 0.3, 1.0, 0.0, 0.0);
  planning_utils::setDz(s1, planner_config_);

  planning_utils::State s2 = s1;
  s2.pos[0] += std::cos(slope);
  s2.pos[2] = 0.3 + std::sin(slope);
  s2.vel[2] = 0.0;

  planning_utils::StateActionResult result;
  GBPL gbpl;
  const int connect_result =
      gbpl.attemptConnect(s1, s2, 2.0, result, planner_config_, FORWARD);

  EXPECT_EQ(connect_result, REACHED);
  EXPECT_TRUE(s2.isApprox(result.s_new));
  EXPECT_NEAR(result.length, 1.0, kTol);
  EXPECT_DOUBLE_EQ(result.t_new, 2.0);
}
