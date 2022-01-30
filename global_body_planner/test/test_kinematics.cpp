#include <ros/ros.h>
#include <gtest/gtest.h>

#include "global_body_planner/planning_utils.h"
#include "global_body_planner/planner_class.h"
#include "global_body_planner/rrt_connect.h"
#include <quad_utils/ros_utils.h>

const int N = 1000;

TEST(GlobalBodyPlannerTest, testLeapAction) {

  // Create planner and configuration
  PlannerClass P(FORWARD);
  PlannerConfig planner_config;
  planner_config.G_VEC << 0, 0, -planner_config.G_CONST;
  planner_config.terrain.loadFlat();
  const double kinematics_tol = 1e-6;
  StateActionResult result;

  // Generate a new state and center it above the origin
  State s;
  s.pos << 0,0,0.3;
  s.vel << 1.0,0,0;

  // Generate a random action
  Eigen::Vector3d surf_norm = planner_config.terrain.getSurfaceNormalFilteredEigen(s.pos[0],s.pos[1]);
  Action a;

  for (int i = 0; i < N; i++) {

    bool is_valid_forward = false;
    while (!is_valid_forward) {
      if (getRandomLeapAction(s, surf_norm, a, planner_config)) {
        is_valid_forward = isValidStateActionPair(s,a,result,planner_config);
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
    State s_final_1 = interpStateActionPair(s, a, t0, dt, interp_plan, interp_GRF, interp_t, 
      interp_primitive_id, interp_length, planner_config);
    State s_final_2 = applyAction(s,a,planner_config);

    // Make sure propagations are equal
    EXPECT_TRUE(s_final_1.isApprox(s_final_2));
    EXPECT_TRUE(s_final_1.isApprox(result.s_new));

    // Reverse the state and action
    flipDirection(s_final_1);
    flipDirection(a);

    // Propagate the reverse state and action
    bool is_valid_reverse = isValidStateActionPair(s_final_1,a,result,planner_config);
    State s_init_1 = interpStateActionPair(s_final_1, a, t0, dt, interp_plan, interp_GRF, interp_t, 
      interp_primitive_id, interp_length, planner_config);
    State s_init_2 = applyAction(s_final_1,a,planner_config);

    // Make sure propagations are equal
    flipDirection(s);
    EXPECT_TRUE(s.isApprox(result.s_new));
    EXPECT_TRUE(s.isApprox(s_init_1));
    EXPECT_TRUE(s.isApprox(s_init_2));
  }
}

TEST(GlobalBodyPlannerTest, testConnectActionUnit) {

  // Create planner and configuration
  PlannerClass P(FORWARD);
  PlannerConfig planner_config;
  planner_config.G_VEC << 0, 0, -planner_config.G_CONST;
  planner_config.terrain.loadFlat();
  const double kinematics_tol = 1e-6;

  // Test connect
  State s1, s2;
  double dist = 1.0;
  double t_s = 2.0;
  s1.pos << 0,0,0.3;
  s1.vel << 0,0,0;
  s2 = s1;
  s2.pos[0] = s1.pos[0] + dist;
  StateActionResult result;

  RRTConnectClass rrt;
  int connect_result = rrt.attemptConnect(s1, s2, t_s, result, planner_config, FORWARD);

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

  State s_final_1 = applyAction(s1, result.a_new, planner_config);
  State s_final_2 = interpStateActionPair(s1, result.a_new, t0, dt, interp_plan, interp_GRF,
    interp_t, interp_primitive_id, interp_length, planner_config);

  EXPECT_TRUE(s2.isApprox(s_final_1));
  EXPECT_TRUE(s2.isApprox(s_final_2));
}

TEST(GlobalBodyPlannerTest, testConnectActionRandom) {


  // Create planner and configuration
  PlannerClass P(FORWARD);
  PlannerConfig planner_config;
  planner_config.G_VEC << 0, 0, -planner_config.G_CONST;
  planner_config.terrain.loadFlat();
  const double kinematics_tol = 1e-6;
  RRTConnectClass rrt;
  StateActionResult result;

  for (int i = 0; i < N; i++) {
    State s1 = P.randomState(planner_config);
    while (!isValidState(s1,planner_config,CONNECT))
      s1 = P.randomState(planner_config);

    State s2 = P.randomState(planner_config);
    while (!isValidState(s2,planner_config,CONNECT))
      s2 = P.randomState(planner_config);

    int connect_result = rrt.attemptConnect(s1, s2, result, planner_config, FORWARD);
    EXPECT_TRUE(connect_result != TRAPPED);
  }
}