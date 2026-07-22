#include <gtest/gtest.h>

#include <grid_map_core/grid_map_core.hpp>

#include "quad_utils/fast_terrain_map.hpp"

namespace {
constexpr double kTol = 1e-9;
}

TEST(FastTerrainMapTest, DefaultAndGeneratedMaps) {
  FastTerrainMap map;
  EXPECT_TRUE(map.isEmpty());

  map.loadFlatElevated(1.25);
  EXPECT_FALSE(map.isEmpty());
  EXPECT_TRUE(map.isInRange(0.0, 0.0));
  EXPECT_FALSE(map.isInRange(5.0, 5.0));
  EXPECT_NEAR(map.getGroundHeight(0.0, 0.0), 1.25, kTol);
  EXPECT_NEAR(map.getGroundHeightFiltered(0.0, 0.0), 1.25, kTol);

  const auto normal = map.getSurfaceNormal(0.0, 0.0);
  EXPECT_NEAR(normal[0], 0.0, kTol);
  EXPECT_NEAR(normal[1], 0.0, kTol);
  EXPECT_NEAR(normal[2], 1.0, kTol);
  EXPECT_TRUE(map.getSurfaceNormalFilteredEigen(0.0, 0.0)
                  .isApprox(Eigen::Vector3d::UnitZ()));

  map.loadSlope(0.5);
  EXPECT_NEAR(map.getGroundHeight(2.0, 0.0), 1.0, kTol);
  const double slope = std::atan(0.5);
  const auto slope_normal = map.getSurfaceNormal(1.0, 0.0);
  EXPECT_NEAR(slope_normal[0], -std::sin(slope), kTol);
  EXPECT_NEAR(slope_normal[1], 0.0, kTol);
  EXPECT_NEAR(slope_normal[2], std::cos(slope), kTol);

  map.loadStep(0.4);
  EXPECT_NEAR(map.getGroundHeight(-1.0, 0.0), 0.0, kTol);
  EXPECT_NEAR(map.getGroundHeight(1.0, 0.0), 0.4, kTol);
}

TEST(FastTerrainMapTest, CustomDataAndGridMapImport) {
  FastTerrainMap map;
  const std::vector<double> xs{0.0, 1.0};
  const std::vector<double> ys{0.0, 1.0};
  const std::vector<std::vector<double>> z{{0.0, 1.0}, {2.0, 3.0}};
  const std::vector<std::vector<double>> nx{{0.0, 0.0}, {0.2, 0.2}};
  const std::vector<std::vector<double>> ny{{0.0, 0.3}, {0.0, 0.3}};
  const std::vector<std::vector<double>> nz{{1.0, 0.9}, {0.8, 0.7}};
  const std::vector<std::vector<double>> z_filt{{10.0, 11.0}, {12.0, 13.0}};

  map.loadData(2, 2, xs, ys, z, nx, ny, nz, z_filt, nx, ny, nz);
  EXPECT_EQ(map.getXData(), xs);
  EXPECT_EQ(map.getYData(), ys);
  EXPECT_NEAR(map.getGroundHeight(0.5, 0.5), 1.5, kTol);
  EXPECT_NEAR(map.getGroundHeightFiltered(0.5, 0.5), 11.5, kTol);
  const auto normal = map.getSurfaceNormalFiltered(0.5, 0.5);
  EXPECT_NEAR(normal[0], 0.1, kTol);
  EXPECT_NEAR(normal[1], 0.15, kTol);
  EXPECT_NEAR(normal[2], 0.85, kTol);

  grid_map::GridMap grid({"z_inpainted", "normal_vectors_x",
                          "normal_vectors_y", "normal_vectors_z", "z_smooth",
                          "smooth_normal_vectors_x",
                          "smooth_normal_vectors_y",
                          "smooth_normal_vectors_z"});
  grid.setGeometry(grid_map::Length(2.0, 2.0), 1.0,
                   grid_map::Position(0.0, 0.0));
  for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    grid.getPosition(*it, position);
    grid.at("z_inpainted", *it) = position.x() + 2.0 * position.y();
    grid.at("normal_vectors_x", *it) = 0.1;
    grid.at("normal_vectors_y", *it) = 0.2;
    grid.at("normal_vectors_z", *it) = 0.9;
    grid.at("z_smooth", *it) = 5.0;
    grid.at("smooth_normal_vectors_x", *it) = 0.0;
    grid.at("smooth_normal_vectors_y", *it) = 0.0;
    grid.at("smooth_normal_vectors_z", *it) = 1.0;
  }

  map.loadDataFromGridMap(grid);
  EXPECT_FALSE(map.isEmpty());
  EXPECT_NEAR(map.getGroundHeightFiltered(0.0, 0.0), 5.0, kTol);
  EXPECT_TRUE(map.getSurfaceNormalFilteredEigen(0.0, 0.0)
                  .isApprox(Eigen::Vector3d::UnitZ()));
}

TEST(FastTerrainMapTest, GridMapImportFallsBackWhenOptionalLayersAreMissing) {
  grid_map::GridMap grid({"z_inpainted"});
  grid.setGeometry(grid_map::Length(2.0, 2.0), 1.0,
                   grid_map::Position(0.0, 0.0));
  for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    grid.getPosition(*it, position);
    grid.at("z_inpainted", *it) = position.x() + position.y();
  }

  FastTerrainMap map;
  map.loadDataFromGridMap(grid);

  EXPECT_FALSE(map.isEmpty());
  EXPECT_NEAR(map.getGroundHeight(0.0, 0.0), 0.0, kTol);
  EXPECT_NEAR(map.getGroundHeightFiltered(0.0, 0.0), 0.0, kTol);

  const auto raw_normal = map.getSurfaceNormal(0.0, 0.0);
  EXPECT_NEAR(raw_normal[0], 0.0, kTol);
  EXPECT_NEAR(raw_normal[1], 0.0, kTol);
  EXPECT_NEAR(raw_normal[2], 1.0, kTol);
  EXPECT_TRUE(map.getSurfaceNormalFilteredEigen(0.0, 0.0)
                  .isApprox(Eigen::Vector3d::UnitZ()));
}

TEST(FastTerrainMapTest, ProjectToMap) {
  FastTerrainMap map;
  map.loadFlat();

  const Eigen::Vector3d projected =
      map.projectToMap(Eigen::Vector3d(0.0, 0.0, 1.0),
                       Eigen::Vector3d(0.0, 0.0, -1.0));
  EXPECT_NEAR(projected.x(), 0.0, kTol);
  EXPECT_NEAR(projected.y(), 0.0, kTol);
  EXPECT_NEAR(projected.z(), 0.0, kTol);

  const Eigen::Vector3d off_map =
      map.projectToMap(Eigen::Vector3d(4.0, 4.0, 1.0),
                       Eigen::Vector3d(1.0, 0.0, -1.0));
  EXPECT_LT(off_map.z(), -1e100);
}
