#include <gtest/gtest.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>

#include "global_body_planner/gbpl.h"
#include "global_body_planner/global_body_planner_test_fixture.h"
#include "global_body_planner/planner_class.h"
#include "global_body_planner/planning_utils.h"

const int N = 1000;
const double kinematics_tol = 1e-6;

TEST_F(GlobalBodyPlannerTestFixture, testLeapAction) {
  StateActionResult result;

  // Generate a new state and center it above the origin
  State s;
  s.pos << 0, 0, 0.3;
  s.vel << 1.0, 0, 0;

  // Generate a random action
  Eigen::Vector3d surf_norm = getSurfaceNormalFiltered(s, planner_config_);
  Action a;

  for (int i = 0; i < N; i++) {
    bool is_valid_forward = false;
    while (!is_valid_forward) {
      if (getRandomLeapAction(s, surf_norm, a, planner_config_)) {
        is_valid_forward =
            isValidStateActionPair(s, a, result, planner_config_);
      }
    }

    // Check validity
    EXPECT_TRUE(is_valid_forward);

    // Apply the action to the state
    double t0 = 0;
    double dt = 0.03;
    std::vector<State> interp_plan;
    std::vector<GRF> interp_GRF;
    std::vector<double> interp_t;
    std::vector<int> interp_primitive_id;
    std::vector<double> interp_length;
    interp_length.push_back(0);

    // Propogate the state forwards in two different ways
    State s_final_1 = interpStateActionPair(
        s, a, t0, dt, interp_plan, interp_GRF, interp_t, interp_primitive_id,
        interp_length, planner_config_);
    State s_final_2 = applyAction(s, a, planner_config_);

    // Make sure propagations are equal
    EXPECT_TRUE(s_final_1.isApprox(s_final_2));
    EXPECT_TRUE(s_final_1.isApprox(result.s_new));

    // Reverse the state and action
    flipDirection(s_final_1);
    flipDirection(a);

    // Propagate the reverse state and action
    bool is_valid_reverse =
        isValidStateActionPair(s_final_1, a, result, planner_config_);
    State s_init_1 = interpStateActionPair(
        s_final_1, a, t0, dt, interp_plan, interp_GRF, interp_t,
        interp_primitive_id, interp_length, planner_config_);
    State s_init_2 = applyAction(s_final_1, a, planner_config_);

    // Make sure propagations are equal
    flipDirection(s);
    EXPECT_TRUE(s.isApprox(result.s_new));
    EXPECT_TRUE(s.isApprox(s_init_1));
    EXPECT_TRUE(s.isApprox(s_init_2));

    if (!s.isApprox(result.s_new)) {
      std::cout << "Reverse:" << std::endl;
      std::cout << "is_valid_reverse = " << ((is_valid_reverse) ? 1 : 0)
                << std::endl;
      printState(s);
      printState(result.s_new);
      printState(s_init_1);
      printState(s_init_2);
      printState(s_final_1);
      printAction(a);
      return;
    }
  }
}

TEST_F(GlobalBodyPlannerTestFixture, testUnitConnectAction) {
  // Test connect
  State s1, s2;
  double dist = 1.0;
  double t_s = 2.0;
  s1.pos << 0, 0, 0.3;
  s1.vel << 0, 0, 0;
  s2 = s1;
  s2.pos[0] = s1.pos[0] + dist;
  StateActionResult result;

  GBPL gbpl;
  int connect_result =
      gbpl.attemptConnect(s1, s2, t_s, result, planner_config_, FORWARD);

  // Make sure the connection is made and that the length and time match
  EXPECT_TRUE(connect_result == REACHED);
  EXPECT_TRUE(s2.isApprox(result.s_new));
  EXPECT_TRUE(abs(result.length - dist) <= kinematics_tol);
  EXPECT_TRUE(result.t_new == t_s);

  // Make sure that applying this new action yields the correct state
  double t0 = 0;
  double dt = 0.03;
  std::vector<State> interp_plan;
  std::vector<GRF> interp_GRF;
  std::vector<double> interp_t;
  std::vector<int> interp_primitive_id;
  std::vector<double> interp_length;
  interp_length.push_back(0);

  State s_final_1 = applyAction(s1, result.a_new, planner_config_);
  State s_final_2 = interpStateActionPair(
      s1, result.a_new, t0, dt, interp_plan, interp_GRF, interp_t,
      interp_primitive_id, interp_length, planner_config_);

  EXPECT_TRUE(s2.isApprox(s_final_1));
  EXPECT_TRUE(s2.isApprox(s_final_2));
}

TEST_F(GlobalBodyPlannerTestFixture, testUnitConnectActionElevatedTerrain) {
  // Create planner and configuration
  double height = 5;
  updateTerrainHeight(height);

  // Test connect
  State s1, s2;
  double dist = 1.0;
  double t_s = 2.0;
  s1.pos << 0, 0, (0.3 + height);
  s1.vel << 0, 0, 0;
  s2 = s1;
  s2.pos[0] = s1.pos[0] + dist;
  StateActionResult result;

  GBPL gbpl;
  int connect_result =
      gbpl.attemptConnect(s1, s2, t_s, result, planner_config_, FORWARD);

  // Make sure the connection is made and that the length and time match
  EXPECT_TRUE(connect_result == REACHED);
  EXPECT_TRUE(s2.isApprox(result.s_new));
  EXPECT_TRUE(abs(result.length - dist) <= kinematics_tol);
  EXPECT_TRUE(result.t_new == t_s);

  // Make sure that applying this new action yields the correct state
  double t0 = 0;
  double dt = 0.03;
  std::vector<State> interp_plan;
  std::vector<GRF> interp_GRF;
  std::vector<double> interp_t;
  std::vector<int> interp_primitive_id;
  std::vector<double> interp_length;
  interp_length.push_back(0);

  State s_final_1 = applyAction(s1, result.a_new, planner_config_);
  State s_final_2 = interpStateActionPair(
      s1, result.a_new, t0, dt, interp_plan, interp_GRF, interp_t,
      interp_primitive_id, interp_length, planner_config_);

  EXPECT_TRUE(s2.isApprox(s_final_1));
  EXPECT_TRUE(s2.isApprox(s_final_2));
}

TEST_F(GlobalBodyPlannerTestFixture, testUnitConnectActionSlope) {
  double grade = 0.1;
  double slope = atan(grade);
  updateTerrainSlope(grade);

  State s1;
  double vel = 1.0;

  s1.pos << 0, 0, 0.3;
  s1.vel << 0, vel, 0;
  EXPECT_TRUE(abs(getDzFromState(s1, planner_config_)) < kinematics_tol);

  s1.pos << 0, 0, 0.3;
  s1.vel << vel, 0, 0;
  EXPECT_TRUE(abs(getDzFromState(s1, planner_config_) - vel * grade) <
              kinematics_tol);

  // Test connect
  State s2;
  double dist = 1.0;
  double t_s = 2.0;
  s1.pos << 0, 0, 0.3;
  s1.vel << vel, 0, 0;
  setDz(s1, planner_config_);
  s2 = s1;
  s2.pos[0] = s1.pos[0] + dist * cos(slope);
  s2.pos[2] = 0.3 + dist * sin(slope);
  s2.vel[2] = 0;
  StateActionResult result;

  GBPL gbpl;
  int connect_result =
      gbpl.attemptConnect(s1, s2, t_s, result, planner_config_, FORWARD);

  // // Make sure the connection is made and that the length and time match
  EXPECT_TRUE(connect_result == REACHED);
  EXPECT_TRUE(s2.isApprox(result.s_new));
  EXPECT_TRUE(abs(result.length - dist) <= kinematics_tol);
  EXPECT_TRUE(result.t_new == t_s);
}

TEST_F(GlobalBodyPlannerTestFixture, testUnitLeapActionSlope) {
  double grade = planner_config_.mu;
  double slope = atan(grade);
  updateTerrainSlope(grade);

  // Turn off friction
  planner_config_.mu = 0;

  State s;
  s.pos << 0, 0, 0.3;
  s.vel << 0.5, 0, 0;
  setDz(s, planner_config_);
  EXPECT_TRUE(isValidState(s, planner_config_, LEAP_STANCE));
  EXPECT_TRUE(getPitchFromState(s, planner_config_) == -slope);

  // Initialise some data
  Eigen::Vector3d surf_norm = getSurfaceNormalFiltered(s, planner_config_);
  Action a;
  StateActionResult result;

  // Generate a specific leap action
  a.t_s_leap = 0.15;
  a.t_f = 1e-6;
  a.t_s_land = 0.15;
  a.dz_0 = getDzFromState(s, planner_config_);
  a.dz_f = 0;
  a.grf_0.setZero();
  a.grf_f.setZero();
  refineAction(s, a, planner_config_);

  // Check that the action is valid and yields a state of the correct height
  // wrt the terrain
  bool is_valid_forward = isValidStateActionPair(s, a, result, planner_config_);
  EXPECT_TRUE(is_valid_forward);
  EXPECT_TRUE(abs(result.s_new.pos[2] -
                  (result.s_new.pos[0] * grade + planner_config_.h_nom)) <
              kinematics_tol);
}
