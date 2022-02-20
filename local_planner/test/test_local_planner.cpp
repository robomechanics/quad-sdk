#include <gtest/gtest.h>
#include <ros/ros.h>

#include "local_planner/local_planner.h"

TEST(LocalPlannerTest, noInputCase) {
  ros::NodeHandle nh;
  LocalPlanner local_planner(nh);

  // Should be false without proper inputs
  auto plan_check_1 = local_planner.computeLocalPlan();
  EXPECT_FALSE(plan_check_1);
}

TEST(LocalFootStepPlanner, baseCase) {
  int N;
  ros::param::get("/nmpc_controller/leg/horizon_length", N);
  int current_plan_index = 0;
  double dt = 0.03;
  double first_element_duration = dt;
  int period = 12;
  int num_feet = 4;
  std::vector<double> duty_cycles = {0.3, 0.3, 0.3, 0.3};
  std::vector<double> phase_offsets = {0.0, 0.5, 0.5, 0.0};
  double ground_clearance = 0.07;
  double hip_clearance = 0.1;
  double standing_error_threshold = 0.03;
  double foothold_search_radius = 0.2;
  double foothold_obj_threshold = 0.6;
  double grf_weight = 0.0;
  std::string obj_fun_layer = "traversability";
  auto quadKD = std::make_shared<quad_utils::QuadKD>();

  LocalFootstepPlanner footstep_planner;
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
    if (i % period < period / 2) {
      contact_schedule.at(i) = {true, false, false, true};
    } else {
      contact_schedule.at(i) = {false, true, true, false};
    }
  }

  Eigen::MatrixXd grf_plan(N, 12);
  grf_plan.fill(0.3);

  Eigen::MatrixXd foot_positions_world(N, 12);
  foot_positions_world.fill(0);

  Eigen::MatrixXd foot_velocities_world(N, 12);
  foot_velocities_world.fill(0);

  Eigen::MatrixXd foot_accelerations_world(N, 12);
  foot_accelerations_world.fill(0);

  Eigen::VectorXd current_foot_positions_world(12);
  current_foot_positions_world << 0.2263, 0.17098, 0.02, -0.2263, 0.17098, 0.02,
      0.2263, -0.17098, 0.02, -0.2263, -0.17098, 0.02;

  Eigen::VectorXd current_foot_velocities_world(12);
  current_foot_velocities_world.fill(0);

  quad_msgs::MultiFootState past_footholds_msg;
  past_footholds_msg.feet.resize(num_feet);
  past_footholds_msg.traj_index = current_plan_index;
  for (int i = 0; i < num_feet; i++) {
    quad_utils::eigenToFootStateMsg(
        current_foot_positions_world.segment(3 * i, 3),
        current_foot_velocities_world.segment(3 * i, 3),
        past_footholds_msg.feet[i]);
    past_footholds_msg.feet[i].traj_index = past_footholds_msg.traj_index;
  }

  grid_map::GridMap map({"z", "traversability"});
  map.setGeometry(grid_map::Length(10, 10), 0.01);

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    map.getPosition(*it, position);
    map.at("z", *it) = 0;
    map.at("traversability", *it) = 1;
  }

  footstep_planner.setTerrainMap(map);

  footstep_planner.computeFootPlan(
      current_plan_index, contact_schedule, body_plan, grf_plan, ref_body_plan,
      current_foot_positions_world, current_foot_velocities_world,
      first_element_duration, past_footholds_msg, foot_positions_world,
      foot_velocities_world, foot_accelerations_world);

  EXPECT_EQ(foot_positions_world(N - 1, 0), -foot_positions_world(N - 1, 3));
  EXPECT_EQ(foot_positions_world(N - 1, 1), foot_positions_world(N - 1, 4));
  EXPECT_EQ(foot_positions_world(N - 1, 3), -foot_positions_world(N - 1, 6));
  EXPECT_EQ(foot_positions_world(N - 1, 4), -foot_positions_world(N - 1, 7));
  EXPECT_EQ(foot_positions_world(N - 1, 6), -foot_positions_world(N - 1, 9));
  EXPECT_EQ(foot_positions_world(N - 1, 7), foot_positions_world(N - 1, 10));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "local_planner_tester");

  return RUN_ALL_TESTS();
}
