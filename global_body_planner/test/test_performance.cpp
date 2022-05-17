#include <gtest/gtest.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>

#include "global_body_planner/fast_global_motion_planner.h"
#include "global_body_planner/global_body_planner_test_fixture.h"
#include "global_body_planner/planner_class.h"
#include "global_body_planner/planning_utils.h"

const int N = 1000;
const double kinematics_tol = 1e-6;

TEST_F(GlobalBodyPlannerTestFixture, testIsValidStateTime) {
  std::vector<State> state_vec(N);

  for (int i = 0; i < N; i++) {
    state_vec[i] = planner_->randomState(planner_config_);
  }

  quad_utils::FunctionTimer timer("isValidState()");
  for (int i = 0; i < N; i++) {
    isValidState(state_vec[i], planner_config_, LEAP_STANCE);
  }
  double avg_duration = timer.reportStatistics(N);

  EXPECT_LE(avg_duration, 5e-6);
}

TEST_F(GlobalBodyPlannerTestFixture, testGetRandomLeapActionTime) {
  State s_origin;
  s_origin.pos.setZero();
  Eigen::Vector3d surf_norm =
      getSurfaceNormalFiltered(s_origin, planner_config_);

  std::vector<State> state_vec(N);
  std::vector<Action> action_vec(N);

  for (int i = 0; i < N; i++) {
    State s = planner_->randomState(planner_config_);
    while (!isValidState(s, planner_config_, LEAP_STANCE))
      s = planner_->randomState(planner_config_);

    state_vec[i] = s;
  }

  quad_utils::FunctionTimer timer("getRandomLeapAction()");
  for (int i = 0; i < N; i++) {
    getRandomLeapAction(state_vec[i], surf_norm, action_vec[i],
                        planner_config_);
  }
  double avg_duration = timer.reportStatistics(N);

  EXPECT_LE(avg_duration, 5e-5);
}

TEST_F(GlobalBodyPlannerTestFixture, testIsValidStateActionPairTime) {
  State s_origin;
  s_origin.pos.setZero();
  Eigen::Vector3d surf_norm =
      getSurfaceNormalFiltered(s_origin, planner_config_);

  std::vector<State> state_vec(N);
  std::vector<Action> action_vec(N);
  StateActionResult result;

  for (int i = 0; i < N; i++) {
    Action a;
    State s = planner_->randomState(planner_config_);
    bool is_valid = getRandomLeapAction(s, surf_norm, a, planner_config_);

    while (!is_valid || !isValidState(s, planner_config_, LEAP_STANCE)) {
      s = planner_->randomState(planner_config_);
      is_valid = getRandomLeapAction(s, surf_norm, a, planner_config_);
    }

    state_vec[i] = s;
    action_vec[i] = a;
  }

  quad_utils::FunctionTimer timer("isValidStateActionPair()");
  for (int i = 0; i < N; i++) {
    isValidStateActionPair(state_vec[i], action_vec[i], result,
                           planner_config_);
  }
  double avg_duration = timer.reportStatistics(N);

  EXPECT_LE(avg_duration, 1e-4);
}

TEST_F(GlobalBodyPlannerTestFixture, testValidStateActionPairAccuracy) {
  State s_origin;
  s_origin.pos.setZero();
  Eigen::Vector3d surf_norm =
      getSurfaceNormalFiltered(s_origin, planner_config_);

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
    state_vec[i] = planner_->randomState(planner_config_);
    state_valid[i] = isValidState(state_vec[i], planner_config_, LEAP_STANCE);
  }

  // Get random leaps
  for (int i = 0; i < N; i++) {
    if (state_valid[i]) {
      state_action_valid_coarse[i] = getRandomLeapAction(
          state_vec[i], surf_norm, action_vec[i], planner_config_);
    }
  }

  // Check validity
  for (int i = 0; i < N; i++) {
    if (state_valid[i]) {
      count_valid_state++;
      if (state_valid[i] && state_action_valid_coarse[i]) {
        count_valid_coarse++;
        if (isValidStateActionPair(state_vec[i], action_vec[i], result,
                                   planner_config_)) {
          count_valid++;
        }
      }
    }
  }

  std::cout << "Valid states/sampled states = " << (double)count_valid_state / N
            << std::endl;
  std::cout << "Valid coarse pairs/sampled states = "
            << (double)count_valid_coarse / N << std::endl;
  std::cout << "Valid pairs/sampled states = " << (double)count_valid / N
            << std::endl;
  std::cout << "Valid coarse pairs/valid states = "
            << (double)count_valid_coarse / count_valid_state << std::endl;
  std::cout << "Valid pairs/Valid coarse pairs = "
            << (double)count_valid / count_valid_coarse << std::endl;

  EXPECT_GE((double)count_valid_coarse / N, 0.6);
  EXPECT_GE((double)count_valid / count_valid_coarse, 0.9);
}

TEST_F(GlobalBodyPlannerTestFixture, testValidStateActionPairRate) {
  Eigen::Vector3d surf_norm;

  StateActionResult result;
  Action a;

  int count_valid = 0;

  // Find valid state action pair
  quad_utils::FunctionTimer timer("testValidStateActionPairRate");
  while (count_valid < N) {
    State s = planner_->randomState(planner_config_);
    if (!isValidState(s, planner_config_, LEAP_STANCE)) {
      continue;
    }
    surf_norm = getSurfaceNormalFiltered(s, planner_config_);
    bool is_valid_forward = false;
    if (getRandomLeapAction(s, surf_norm, a, planner_config_)) {
      if (isValidStateActionPair(s, a, result, planner_config_)) {
        count_valid++;
      };
    }
  }
  double avg_duration = timer.reportStatistics(N);

  EXPECT_TRUE(avg_duration <= 5e-5);
}
