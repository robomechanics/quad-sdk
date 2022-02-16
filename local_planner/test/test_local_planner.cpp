#include <gtest/gtest.h>
#include <ros/ros.h>

#include "local_planner/local_planner.h"

TEST(LocalPlannerTest, baseCase) {
  ros::NodeHandle nh;
  LocalPlanner local_planner(nh);

  // Should be false without proper inputs
  auto plan_check_1 = local_planner.computeLocalPlan();
  EXPECT_FALSE(plan_check_1);
}

TEST(LocalFootStepPlanner, baseCase) {
  LocalFootstepPlanner footstep_planner;

  int N;
  ros::param::get("/nmpc_controller/leg/horizon_length", N);
  double dt = 0.03;
  double period = 0.36;
  std::vector<double> duty_cycles = {0.3, 0.3, 0.3, 0.3};
  std::vector<double> phase_offsets = {0.0, 0.5, 0.5, 0.0};
  double ground_clearance = 0.07;
  double hip_clearance = 0.1;
  double standing_error_threshold = 0.03;
  double foothold_search_radius = 0.2;
  double foothold_obj_threshold = 0.6;
  double grf_weight = 0.5;
  auto quadKD = std::make_shared<quad_utils::QuadKD>();
  std::string obj_fun_layer = "traversability";

  footstep_planner.setTemporalParams(dt, period, N, duty_cycles, phase_offsets);
  footstep_planner.setSpatialParams(
      ground_clearance, hip_clearance, standing_error_threshold, grf_weight,
      quadKD, foothold_search_radius, foothold_obj_threshold, obj_fun_layer);

  Eigen::MatrixXd body_plan(N, 12);
  body_plan.fill(0);
  body_plan.col(2).fill(0.3);

  Eigen::MatrixXd ref_body_plan(N + 1, 12);
  ref_body_plan.fill(0);
  ref_body_plan.col(2).fill(0.3);

  std::vector<std::vector<bool>> contact_schedule;
  contact_schedule.resize(N);
  for (int i = 0; i < N; i++) {
    contact_schedule.at(i).resize(4);
    if (i % 12 < 6) {
      contact_schedule.at(i) = {true, false, false, true};
    } else {
      contact_schedule.at(i) = {false, true, true, false};
    }
  }

  Eigen::MatrixXd grf_plan(N, 12);
  // Fill every third column with something
  grf_plan.fill(0);

  Eigen::MatrixXd foot_positions_world(N, 12);
  foot_positions_world.fill(0);
  footstep_planner.computeFootPositions(body_plan, grf_plan, contact_schedule,
                                        ref_body_plan, foot_positions_world);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "local_planner_tester");

  return RUN_ALL_TESTS();
}
