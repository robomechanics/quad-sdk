#include <ros/ros.h>
#include <gtest/gtest.h>

#include "global_body_planner/planning_utils.h"
#include "global_body_planner/planner_class.h"
#include "global_body_planner/rrt_connect.h"
#include <quad_utils/ros_utils.h>

const int N = 1000;
const double kinematics_tol = 1e-6;

TEST(GlobalBodyPlannerTest, testIsValidStateTime) {

  // Create planner and configuration
  PlannerClass P(FORWARD);
  PlannerConfig planner_config;
  planner_config.loadVectors();
  planner_config.terrain.loadFlat();

  std::vector<State> state_vec(N);

  for (int i = 0; i < N; i++) {
    state_vec[i] = P.randomState(planner_config);
  }

  quad_utils::FunctionTimer timer("isValidState()");
  for (int i = 0; i < N; i++) {
    isValidState(state_vec[i], planner_config, LEAP_STANCE);
  }
  double avg_duration = timer.report(N);

  EXPECT_TRUE(avg_duration <= 5e-6);

}

TEST(GlobalBodyPlannerTest, testGetRandomLeapActionTime) {

  // Create planner and configuration
  PlannerClass P(FORWARD);
  PlannerConfig planner_config;
  planner_config.loadVectors();
  planner_config.terrain.loadFlat();
  Eigen::Vector3d surf_norm = planner_config.terrain.getSurfaceNormalFilteredEigen(0, 0);

  std::vector<State> state_vec(N);
  std::vector<Action> action_vec(N);

  for (int i = 0; i < N; i++) {
    State s = P.randomState(planner_config);
    while (!isValidState(s,planner_config,LEAP_STANCE))
      s = P.randomState(planner_config);

    state_vec[i] = s;
  }

  quad_utils::FunctionTimer timer("getRandomLeapAction()");
  for (int i = 0; i < N; i++) {
    getRandomLeapAction(state_vec[i], surf_norm, action_vec[i], planner_config);
  }
  double avg_duration = timer.report(N);

  EXPECT_TRUE(avg_duration <= 5e-6);

}

TEST(GlobalBodyPlannerTest, testIsValidStateActionPairTime) {

  // Create planner and configuration
  PlannerClass P(FORWARD);
  PlannerConfig planner_config;
  planner_config.loadVectors();
  planner_config.terrain.loadFlat();
  Eigen::Vector3d surf_norm = planner_config.terrain.getSurfaceNormalFilteredEigen(0, 0);

  std::vector<State> state_vec(N);
  std::vector<Action> action_vec(N);
  StateActionResult result;

  for (int i = 0; i < N; i++) {
    Action a;
    State s = P.randomState(planner_config);
    bool is_valid = getRandomLeapAction(s,surf_norm,a,planner_config);

    while (!is_valid || !isValidState(s,planner_config,LEAP_STANCE)) {
      s = P.randomState(planner_config);
      is_valid = getRandomLeapAction(s,surf_norm,a,planner_config);

    }

    state_vec[i] = s;
    action_vec[i] = a;
  }

  quad_utils::FunctionTimer timer("isValidStateActionPair()");
  for (int i = 0; i < N; i++) {
    isValidStateActionPair(state_vec[i], action_vec[i], result, planner_config);
  }
  double avg_duration = timer.report(N);

  EXPECT_TRUE(avg_duration <= 5e-5);

}

TEST(GlobalBodyPlannerTest, testValidStateActionPairAccuracy) {

  // Create planner and configuration
  PlannerClass P(FORWARD);
  PlannerConfig planner_config;
  planner_config.loadVectors();
  planner_config.terrain.loadFlat();
  Eigen::Vector3d surf_norm = planner_config.terrain.getSurfaceNormalFilteredEigen(0, 0);

  std::vector<State> state_vec(N);
  std::vector<Action> action_vec(N);
  std::vector<bool> state_valid(N);
  std::vector<bool> state_action_valid_coarse(N);
  StateActionResult result;

  int count_valid_state = 0;
  int count_valid_coarse = 0;
  int count_valid = 0;

  // Sample states
  for (int i = 0; i < N; i++) {
    state_vec[i] = P.randomState(planner_config);
    state_valid[i] = isValidState(state_vec[i],planner_config,LEAP_STANCE);
  }

  // Get random leaps
  for (int i = 0; i < N; i++) {
    if (state_valid[i]) {
      state_action_valid_coarse[i] = getRandomLeapAction(state_vec[i], surf_norm, action_vec[i], planner_config);
    }
  }

  // Check validity
  for (int i = 0; i < N; i++) {
    if (state_valid[i]) {
      count_valid_state++;
      if (state_valid[i] && state_action_valid_coarse[i]) {
        count_valid_coarse++;
        if (isValidStateActionPair(state_vec[i], action_vec[i],result, planner_config)) {
          count_valid++;
        }
      }
    }
  }

  std::cout << "Valid states/sampled states = " << (double)count_valid_state/N << std::endl;
  std::cout << "Valid coarse pairs/sampled states = " << (double)count_valid_coarse/N << std::endl;
  std::cout << "Valid pairs/sampled states = " << (double)count_valid/N << std::endl;
  std::cout << "Valid coarse pairs/valid states = " << (double)count_valid_coarse/count_valid_state << std::endl;
  std::cout << "Valid pairs/Valid coarse pairs = " << (double)count_valid/count_valid_coarse << std::endl;

  EXPECT_TRUE((double)count_valid_coarse/N >= 0.8);
  EXPECT_TRUE((double)count_valid/count_valid_coarse >= 0.8);
}

TEST(GlobalBodyPlannerTest, testValidStateActionPairRate) {

  // Create planner and configuration
  PlannerClass P(FORWARD);
  PlannerConfig planner_config;
  planner_config.loadVectors();
  planner_config.terrain.loadFlat();
  Eigen::Vector3d surf_norm;

  StateActionResult result;
  Action a;

  int count_valid = 0;

  // Find valid state action pair
  quad_utils::FunctionTimer timer("testValidStateActionPairRate");
  while (count_valid<N) {
    State s = P.randomState(planner_config);
    surf_norm = planner_config.terrain.getSurfaceNormalFilteredEigen(s.pos[0], s.pos[1]);
    bool is_valid_forward = false;
    while (!is_valid_forward) {
      if (getRandomLeapAction(s, surf_norm, a, planner_config)) {
        is_valid_forward = isValidStateActionPair(s,a,result,planner_config);
      }
    }
    count_valid++;
  }
  double avg_duration = timer.report(N);

  EXPECT_TRUE(avg_duration <= 5e-5);
}