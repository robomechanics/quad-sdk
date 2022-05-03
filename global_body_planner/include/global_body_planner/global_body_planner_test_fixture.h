#ifndef GLOBAL_BODY_PLANNER_TEST_H
#define GLOBAL_BODY_PLANNER_TEST_H

#include <gtest/gtest.h>
#include <quad_utils/fast_terrain_map.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include "global_body_planner/planner_class.h"
#include "global_body_planner/planning_utils.h"
#include "global_body_planner/rrt_connect.h"

//! A test fixture for the global body planning class
/*!
   GlobalBodyPlannerTest is a container for all of the logic utilized in the
   global body planning node. This algorithm requires an height map of the
   terrain as a GridMap message type, and will publish the global body plan as a
   BodyPlan message over a topic. It will also publish the discrete states used
   by the planner (from which the full path is interpolated).
*/
class GlobalBodyPlannerTestFixture : public ::testing::Test {
 public:
  /**
   * @brief Constructor for GlobalBodyPlannerTestFixture Class
   * @return Constructed object of type GlobalBodyPlannerTestFixture
   */
  GlobalBodyPlannerTestFixture() : planner_(FORWARD) {
    grid_map::GridMap map({"z_inpainted", "z_smooth", "normal_vectors_x",
                           "normal_vectors_y", "normal_vectors_z",
                           "smooth_normal_vectors_x", "smooth_normal_vectors_y",
                           "smooth_normal_vectors_z", "traversability"});
    map.setGeometry(grid_map::Length(10, 10), 0.05);
    terrain_grid_map_ = map;

    double slope = 0;
    updateTerrainSlope(slope);

    // Create planner and configuration
    planner_config_.loadEigenVectorsFromParams();
  }

  /**
   * @brief Constructor for GlobalBodyPlannerTestFixture Class
   * @param[in] grade Grade of the new terrain in x direction (rise/run)
   */
  void updateTerrainSlope(double grade) {
    double slope = atan(grade);
    Eigen::Vector3d normal_vec;
    normal_vec << -sin(slope), 0, cos(slope);

    for (grid_map::GridMapIterator it(terrain_grid_map_); !it.isPastEnd();
         ++it) {
      grid_map::Position position;
      terrain_grid_map_.getPosition(*it, position);
      terrain_grid_map_.at("z_inpainted", *it) = position.x() * grade;
      terrain_grid_map_.at("z_smooth", *it) =
          terrain_grid_map_.at("z_inpainted", *it);
      terrain_grid_map_.at("normal_vectors_x", *it) = normal_vec.x();
      terrain_grid_map_.at("normal_vectors_y", *it) = normal_vec.y();
      terrain_grid_map_.at("normal_vectors_z", *it) = normal_vec.z();
      terrain_grid_map_.at("smooth_normal_vectors_x", *it) = normal_vec.x();
      terrain_grid_map_.at("smooth_normal_vectors_y", *it) = normal_vec.y();
      terrain_grid_map_.at("smooth_normal_vectors_z", *it) = normal_vec.z();
      terrain_grid_map_.at("traversability", *it) = 1.0;
    }

    terrain_.loadDataFromGridMap(terrain_grid_map_);
    planner_config_.terrain = terrain_;
    planner_config_.terrain_grid_map = terrain_grid_map_;
  }

  /**
   * @brief Constructor for GlobalBodyPlannerTestFixture Class
   * @param[in] grade Grade of the new terrain in x direction (rise/run)
   */
  void updateTerrainHeight(double height) {
    Eigen::Vector3d normal_vec;
    normal_vec << 0, 0, 0;

    for (grid_map::GridMapIterator it(terrain_grid_map_); !it.isPastEnd();
         ++it) {
      grid_map::Position position;
      terrain_grid_map_.getPosition(*it, position);
      terrain_grid_map_.at("z_inpainted", *it) = height;
      terrain_grid_map_.at("z_smooth", *it) =
          terrain_grid_map_.at("z_inpainted", *it);
      terrain_grid_map_.at("normal_vectors_x", *it) = normal_vec.x();
      terrain_grid_map_.at("normal_vectors_y", *it) = normal_vec.y();
      terrain_grid_map_.at("normal_vectors_z", *it) = normal_vec.z();
      terrain_grid_map_.at("smooth_normal_vectors_x", *it) = normal_vec.x();
      terrain_grid_map_.at("smooth_normal_vectors_y", *it) = normal_vec.y();
      terrain_grid_map_.at("smooth_normal_vectors_z", *it) = normal_vec.z();
      terrain_grid_map_.at("traversability", *it) = 1.0;
    }

    terrain_.loadDataFromGridMap(terrain_grid_map_);
    planner_config_.terrain = terrain_;
    planner_config_.terrain_grid_map = terrain_grid_map_;
  }

  /// Planner class
  PlannerClass planner_;

  /// Planner configuration
  PlannerConfig planner_config_;

  /// Flat terrain map
  grid_map::GridMap terrain_grid_map_;

  /// Sloped terrain map
  FastTerrainMap terrain_;
};

#endif  // GLOBAL_BODY_PLANNER_TEST_H
