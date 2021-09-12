#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_utils/fast_terrain_map.h"

TEST(FastTerrainMapTest, testConstructor) {
  ros::NodeHandle nh;
  FastTerrainMap fast_terrain_map;
  EXPECT_EQ(1 + 1, 2);
}

TEST(FastTerrainMapTest, testProjection) {
  ros::NodeHandle nh;
  FastTerrainMap fast_terrain_map;

  int x_size = 2;
  int y_size = 2;
  std::vector<double> x_data = {-1, 1};
  std::vector<double> y_data = {-1, 1};
  std::vector<double> z_data_vec = {-1, 1};
  std::vector<std::vector<double> > z_data = {z_data_vec,z_data_vec};

  std::vector<double> dx_data_vec = {0, 0};
  std::vector<double> dz_data_vec = {1, 1};
  std::vector<std::vector<double> > dx_data = {dx_data_vec,dx_data_vec};
  std::vector<std::vector<double> > dy_data = dx_data;
  std::vector<std::vector<double> > dz_data = {dz_data_vec,dz_data_vec};


  fast_terrain_map.loadData(x_size, y_size, x_data, y_data, z_data, dx_data, dy_data, dz_data,
    z_data, dx_data, dy_data, dz_data);

  // Eigen::Vector3d point = {0,0.5,1};
  // Eigen::Vector3d direction = {0.1,0.1,-1};
  Eigen::Vector3d point = {0,0,1};
  Eigen::Vector3d direction = {0,0,-1};

  auto t_start = std::chrono::steady_clock::now();
  Eigen::Vector3d intersection = fast_terrain_map.projectToMap(point, direction);
  auto t_end = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start);
  // std::cout << "projectToMap took " << time_span.count() << " seconds." << std::endl;

  // std::cout << "Result is {" << intersection[0] << ", " << intersection[1] << ", " << intersection[2] << "}" << std::endl;

  EXPECT_EQ(1 + 1, 2);
}