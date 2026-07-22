#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "global_body_planner/gbpl.hpp"
#include "global_body_planner/global_body_planner_test_fixture.hpp"

namespace {

constexpr double kTol = 1e-6;

planning_utils::State makeState(double x, double y, double z, double vx,
                                double vy, double vz) {
  planning_utils::State s;
  s.pos << x, y, z;
  s.vel << vx, vy, vz;
  return s;
}

}  // namespace

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

TEST_F(GlobalBodyPlannerTestFixture, ApplyFlightUsesBallisticDynamics) {
  planning_utils::State s = makeState(0.0, 0.0, 1.0, 1.0, 2.0, 3.0);

  const planning_utils::State out =
      planning_utils::applyFlight(s, 0.5, planner_config_);

  EXPECT_NEAR(out.pos.x(), 0.5, kTol);
  EXPECT_NEAR(out.pos.y(), 1.0, kTol);
  EXPECT_NEAR(out.pos.z(), 1.0 + 1.5 - 0.5 * planner_config_.g * 0.25, kTol);
  EXPECT_NEAR(out.vel.x(), 1.0, kTol);
  EXPECT_NEAR(out.vel.y(), 2.0, kTol);
  EXPECT_NEAR(out.vel.z(), 3.0 - planner_config_.g * 0.5, kTol);
}

TEST_F(GlobalBodyPlannerTestFixture, GrfAndAccelerationAgree) {
  planning_utils::Action a;
  a.grf_0 << 0.1, 0.0, 0.3;
  a.grf_f << 0.2, 0.0, 0.3;
  a.t_s_leap = 1.0;
  a.t_f = 0.0;
  a.t_s_land = 0.0;
  a.dz_0 = 0.0;
  a.dz_f = 0.0;

  const planning_utils::GRF grf =
      planning_utils::getGRF(a, 0.5, CONNECT, planner_config_);
  const Eigen::Vector3d accel =
      planning_utils::getAcceleration(a, 0.5, CONNECT, planner_config_);

  EXPECT_TRUE(accel.isApprox(grf / planner_config_.mass +
                             planner_config_.g_vec));
}

TEST_F(GlobalBodyPlannerTestFixture, RandomLeapActionProducesValidAction) {
  std::srand(1);
  planning_utils::State s = makeState(0.0, 0.0, planner_config_.h_nom, 0.5,
                                      0.0, 0.0);
  planning_utils::setDz(s, planner_config_);
  planning_utils::Action a;

  ASSERT_TRUE(planning_utils::getRandomLeapAction(
      s, Eigen::Vector3d(0.0, 0.0, 1.0), a, planner_config_));
  EXPECT_TRUE(planning_utils::isValidAction(a, planner_config_));
  EXPECT_GT(a.t_s_leap, 0.0);
  EXPECT_GT(a.t_f, 0.0);
  EXPECT_GT(a.t_s_land, 0.0);
}

TEST_F(GlobalBodyPlannerTestFixture, InterpolationMatchesApplyActionEndpoint) {
  planning_utils::State s1 = makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  planning_utils::State s2 = s1;
  s2.pos[0] += 1.0;

  planning_utils::StateActionResult result;
  GBPL gbpl;
  ASSERT_EQ(gbpl.attemptConnect(s1, s2, 2.0, result, planner_config_, FORWARD),
            REACHED);

  std::vector<planning_utils::State> interp_plan;
  std::vector<planning_utils::GRF> interp_grf;
  std::vector<double> interp_t;
  std::vector<int> primitive_ids;
  std::vector<double> interp_length{0.0};

  const planning_utils::State endpoint =
      planning_utils::interpStateActionPair(
          s1, result.a_new, 0.0, planner_config_.dt, interp_plan, interp_grf,
          interp_t, primitive_ids, interp_length, planner_config_);

  EXPECT_TRUE(endpoint.isApprox(
      planning_utils::applyAction(s1, result.a_new, planner_config_)));
  EXPECT_FALSE(interp_plan.empty());
  EXPECT_EQ(interp_plan.size(), interp_grf.size());
  EXPECT_EQ(interp_plan.size(), interp_t.size());
  EXPECT_EQ(interp_plan.size(), primitive_ids.size());
}
