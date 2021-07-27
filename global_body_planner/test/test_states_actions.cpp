#include <ros/ros.h>
#include <gtest/gtest.h>

#include "global_body_planner/action.h"
#include "global_body_planner/walk_action.h"
#include "global_body_planner/leap_action.h"

TEST(GlobalBodyPlannerTest, testActions) {

  int N = 1000000;
  std::vector<planning_utils::State> eig_vec;
  std::vector<planning_utils::WalkAction> action_vec;
  std::chrono::time_point<std::chrono::steady_clock> t_start = std::chrono::steady_clock::now();
  for (int i = 0; i < N; i++) {
    planning_utils::State test;
    eig_vec.push_back(test);
  }
  std::chrono::time_point<std::chrono::steady_clock> t_eigen = std::chrono::steady_clock::now();

  for (int i = 0; i < N; i++) {
    planning_utils::WalkAction action;
    action_vec.push_back(action);
  }
  std::chrono::time_point<std::chrono::steady_clock> t_action = std::chrono::steady_clock::now();

  planning_utils::WalkAction walk_action;
  planning_utils::LeapAction leap_action;
  walk_action.print();
  leap_action.print();
  
  std::chrono::duration<double> duration_eigen = std::chrono::duration_cast<std::chrono::duration<double>>(t_eigen - t_start);
  std::chrono::duration<double> duration_action = std::chrono::duration_cast<std::chrono::duration<double>>(t_action - t_eigen);
  printf("Eigen constructor took %3e s\n", duration_eigen.count()/N);
  printf("Action constructor took %3e s\n", duration_action.count()/N);

  EXPECT_EQ(1 + 1, 2);
}