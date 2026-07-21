#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>
#include <vector>

#include "local_planner/local_footstep_planner.hpp"

namespace {

constexpr double kTol = 1e-6;

grid_map::GridMap makeTerrain(double height = 0.0, double traversability = 1.0) {
  grid_map::GridMap map({"z_inpainted", "z_smooth", "normal_vectors_x",
                         "normal_vectors_y", "normal_vectors_z",
                         "smooth_normal_vectors_x", "smooth_normal_vectors_y",
                         "smooth_normal_vectors_z", "traversability"});
  map.setGeometry(grid_map::Length(4.0, 4.0), 0.1);
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    map.at("z_inpainted", *it) = height;
    map.at("z_smooth", *it) = height;
    map.at("normal_vectors_x", *it) = 0.0;
    map.at("normal_vectors_y", *it) = 0.0;
    map.at("normal_vectors_z", *it) = 1.0;
    map.at("smooth_normal_vectors_x", *it) = 0.0;
    map.at("smooth_normal_vectors_y", *it) = 0.0;
    map.at("smooth_normal_vectors_z", *it) = 1.0;
    map.at("traversability", *it) = traversability;
  }
  return map;
}

LocalFootstepPlanner makePlanner() {
  auto node = std::make_shared<rclcpp::Node>("local_footstep_planner_test");
  LocalFootstepPlanner planner(node);
  planner.setTemporalParams(0.1, 4, 6, {0.5, 0.5, 0.5, 0.5},
                            {0.0, 0.5, 0.5, 0.0});
  planner.setSpatialParams(0.07, 0.1, 0.45, 0.03, nullptr, 0.25, 0.6,
                           "traversability", 0.02);
  const auto terrain_grid = makeTerrain();
  FastTerrainMap terrain;
  terrain.loadDataFromGridMap(terrain_grid);
  planner.updateMap(terrain_grid);
  planner.updateMap(terrain);
  return planner;
}

}  // namespace

TEST(LocalFootstepPlannerTest, ContactScheduleTilesGaitAndStandMode) {
  LocalFootstepPlanner planner = makePlanner();
  Eigen::MatrixXd body_plan = Eigen::MatrixXd::Zero(6, 12);
  Eigen::VectorXi primitives = Eigen::VectorXi::Zero(6);
  std::vector<std::vector<bool>> schedule;

  planner.computeContactSchedule(0, body_plan, primitives, STEP, schedule);

  ASSERT_EQ(schedule.size(), 6u);
  EXPECT_EQ(schedule[0], (std::vector<bool>{true, false, false, true}));
  EXPECT_EQ(schedule[1], (std::vector<bool>{true, false, false, true}));
  EXPECT_EQ(schedule[2], (std::vector<bool>{false, true, true, false}));
  EXPECT_EQ(schedule[3], (std::vector<bool>{false, true, true, false}));
  EXPECT_EQ(schedule[4], schedule[0]);

  planner.computeContactSchedule(0, body_plan, primitives, STAND, schedule);
  for (const auto& row : schedule) {
    EXPECT_EQ(row, (std::vector<bool>{true, true, true, true}));
  }
}

TEST(LocalFootstepPlannerTest, ContactScheduleOverridesFlightAndLanding) {
  LocalFootstepPlanner planner = makePlanner();
  Eigen::MatrixXd body_plan = Eigen::MatrixXd::Zero(6, 12);
  Eigen::VectorXi primitives = Eigen::VectorXi::Zero(6);
  primitives(1) = 2;
  primitives(2) = 3;
  std::vector<std::vector<bool>> schedule;

  planner.computeContactSchedule(0, body_plan, primitives, STEP, schedule);

  EXPECT_EQ(schedule[1], (std::vector<bool>{false, false, false, false}));
  EXPECT_EQ(schedule[2], (std::vector<bool>{true, true, true, true}));
}

TEST(LocalFootstepPlannerTest, FootPositionsTransformFromWorldToBody) {
  LocalFootstepPlanner planner = makePlanner();
  Eigen::VectorXd body = Eigen::VectorXd::Zero(12);
  body << 1.0, 2.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Eigen::VectorXd feet_world(12);
  feet_world << 1.2, 2.3, 0.0, 0.8, 2.3, 0.0, 1.2, 1.7, 0.0, 0.8, 1.7, 0.0;
  Eigen::VectorXd feet_body = Eigen::VectorXd::Zero(12);

  planner.getFootPositionsBodyFrame(body, feet_world, feet_body);

  EXPECT_NEAR(feet_body[0], 0.2, kTol);
  EXPECT_NEAR(feet_body[1], 0.3, kTol);
  EXPECT_NEAR(feet_body[2], -0.5, kTol);

  Eigen::MatrixXd body_plan = Eigen::MatrixXd::Zero(6, 12);
  body_plan.row(0) = body;
  body_plan.row(1) = body;
  Eigen::MatrixXd feet_plan(6, 12);
  feet_plan.row(0) = feet_world;
  feet_plan.row(1) = feet_world;
  Eigen::MatrixXd feet_body_plan = Eigen::MatrixXd::Zero(6, 12);

  planner.getFootPositionsBodyFrame(body_plan, feet_plan, feet_body_plan);

  EXPECT_NEAR(feet_body_plan(1, 0), 0.2, kTol);
  EXPECT_NEAR(feet_body_plan(1, 2), -0.5, kTol);
}

TEST(LocalFootstepPlannerTest, TerrainQueriesAndFutureBodyPlan) {
  LocalFootstepPlanner planner = makePlanner();
  Eigen::VectorXd body = Eigen::VectorXd::Zero(12);
  body[0] = 1.0;
  body[1] = 2.0;
  body[2] = 0.3;
  body[6] = 0.5;
  body[7] = -0.2;

  const Eigen::VectorXd future = planner.computeFutureBodyPlan(3.0, body);

  EXPECT_NEAR(planner.getTerrainHeight(0.0, 0.0), 0.0, kTol);
  EXPECT_NEAR(planner.getTerrainSlope(0.0, 0.0, 1.0, 0.0), 0.0, kTol);
  EXPECT_NEAR(future[0], 1.15, kTol);
  EXPECT_NEAR(future[1], 1.94, kTol);
  EXPECT_NEAR(future[5], body[5], kTol);
}

TEST(LocalFootstepPlannerTest, CubicHermiteSplineMatchesEndpoints) {
  LocalFootstepPlanner planner = makePlanner();
  double pos = 0.0;
  double vel = 0.0;
  double acc = 0.0;

  planner.cubicHermiteSpline(1.0, 0.0, 3.0, 0.0, 0.0, 2.0, pos, vel, acc);
  EXPECT_NEAR(pos, 1.0, kTol);
  EXPECT_NEAR(vel, 0.0, kTol);

  planner.cubicHermiteSpline(1.0, 0.0, 3.0, 0.0, 1.0, 2.0, pos, vel, acc);
  EXPECT_NEAR(pos, 3.0, kTol);
  EXPECT_NEAR(vel, 0.0, kTol);
}

TEST(LocalFootstepPlannerTest, FootholdSearchUsesValidTerrainAndToeRadius) {
  LocalFootstepPlanner planner = makePlanner();
  const Eigen::Vector3d nominal(0.0, 0.0, 0.0);
  const Eigen::Vector3d previous(0.0, 0.0, 0.0);

  const Eigen::Vector3d foothold =
      planner.getNearestValidFoothold(nominal, previous);

  EXPECT_NEAR(foothold.x(), 0.0, 0.11);
  EXPECT_NEAR(foothold.y(), 0.0, 0.11);
  EXPECT_NEAR(foothold.z(), 0.02, kTol);
}

TEST(LocalFootstepPlannerTest, FootPlanMessagesContainTouchdownsAndTimestamps) {
  LocalFootstepPlanner planner = makePlanner();
  std::vector<std::vector<bool>> schedule = {
      {false, true, true, true},
      {true, true, true, true},
      {true, false, true, true},
      {true, true, true, true},
  };
  Eigen::MatrixXd positions = Eigen::MatrixXd::Zero(4, 12);
  Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(4, 12);
  Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(4, 12);
  positions(1, 0) = 0.4;
  positions(3, 3) = -0.2;

  quad_msgs::msg::MultiFootPlanDiscrete footholds;
  quad_msgs::msg::MultiFootPlanContinuous continuous;
  continuous.header.stamp.sec = 10;
  footholds.header = continuous.header;

  planner.loadFootPlanMsgs(schedule, 7, 0.05, positions, velocities,
                           accelerations, footholds, continuous);

  ASSERT_EQ(continuous.states.size(), 4u);
  EXPECT_EQ(continuous.states[0].traj_index, 7);
  EXPECT_EQ(continuous.states[3].traj_index, 10);
  EXPECT_NEAR(
      (rclcpp::Time(continuous.states[1].header.stamp) -
       rclcpp::Time(continuous.states[0].header.stamp))
          .seconds(),
      0.05, kTol);
  ASSERT_EQ(footholds.feet.size(), 4u);
  ASSERT_EQ(footholds.feet[0].footholds.size(), 1u);
  ASSERT_EQ(footholds.feet[1].footholds.size(), 1u);
  EXPECT_NEAR(footholds.feet[0].footholds.front().position.x, 0.4, kTol);
}
