#include <gtest/gtest.h>
#include <ros/ros.h>

#include "quad_utils/fast_terrain_map.h"

TEST(FastTerrainMapTest, testSpeedComparison) {
  // Define map parameters
  double res = 0.01;
  double x_length = 12.0;
  double y_length = 5.0;
  double x_center = 4.0;
  double y_center = 0.0;
  double x_origin = x_center - 0.5 * x_length;
  double y_origin = y_center - 0.5 * y_length;

  // Create GridMap
  grid_map::GridMap grid_map_obj(
      {"z", "nx", "ny", "nz", "z_filt", "nx_filt", "ny_filt", "nz_filt"});
  // grid_map_obj.setBasicLayers({"z","nx","ny","nz","z_filt","nx_filt","ny_filt","nz_filt"});
  grid_map_obj.setFrameId("map");
  grid_map_obj.setGeometry(grid_map::Length(x_length, y_length), res,
                           grid_map::Position(x_center, y_center));
  int x_size = grid_map_obj.getSize()(0);
  int y_size = grid_map_obj.getSize()(1);
  // printf("Created map with size %f x %f m (%i x %i cells).\n",
  //   grid_map_obj.getLength().x(), grid_map_obj.getLength().y(), x_size,
  //   y_size);

  // Load grid map with random noise
  for (grid_map::GridMapIterator it(grid_map_obj); !it.isPastEnd(); ++it) {
    grid_map_obj.at("z", *it) = 0.1 * ((double)rand() / RAND_MAX);
    grid_map_obj.at("z_filt", *it) = grid_map_obj.at("z", *it);

    grid_map_obj.at("nx", *it) = 0.0;
    grid_map_obj.at("ny", *it) = 0.0;
    grid_map_obj.at("nz", *it) = 1.0;

    grid_map_obj.at("nx_filt", *it) = 0.0;
    grid_map_obj.at("ny_filt", *it) = 0.0;
    grid_map_obj.at("nz_filt", *it) = 1.0;
  }

  // Load data into fast terrain map obj
  FastTerrainMap fast_terrain_map;
  fast_terrain_map.loadDataFromGridMap(grid_map_obj);

  // Initialize testing parameters
  const int N = 10001;
  grid_map::Position pos;
  double x, y, z;
  std::chrono::time_point<std::chrono::steady_clock> start_time,
      intermediate_time_1, intermediate_time_2, stop_time;
  std::chrono::duration<double> elapsed;
  std::vector<double> timings(N - 1);

  // Start GridMap nearest neighbor
  double gm_nn_total_elapsed = 0;
  start_time = std::chrono::steady_clock::now();
  for (int i = 0; i < N; i++) {
    // Generate random test point
    x = (x_length - res) * ((double)rand() / RAND_MAX) + x_origin + 0.5 * res;
    y = (y_length - res) * ((double)rand() / RAND_MAX) + y_origin + 0.5 * res;

    // Query
    pos = {x, y};
    intermediate_time_1 = std::chrono::steady_clock::now();
    z += grid_map_obj.atPosition(
             "z", pos, grid_map::InterpolationMethods::INTER_NEAREST) /
         N;
    intermediate_time_2 = std::chrono::steady_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
        intermediate_time_2 - intermediate_time_1);

    if (i > 0) {
      timings[i - 1] = (double)elapsed.count();
    } else {
      start_time = std::chrono::steady_clock::now();
    }
  }

  stop_time = std::chrono::steady_clock::now();
  elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      stop_time - start_time);
  double gm_nn_time = (double)elapsed.count();
  for (int i = 0; i < timings.size(); i++) {
    gm_nn_total_elapsed += timings[i];
    if (fmod(log10(i), 1) == 0) {
      // printf("Duration of nn iteration %d = %.3fus\n", i, 1e6*timings[i]);
    }
  }

  // Start GridMap linear
  double gm_lin_total_elapsed = 0;
  z = 0;
  start_time = std::chrono::steady_clock::now();
  for (int i = 0; i < N; i++) {
    // Generate random test point
    x = (x_length - res) * ((double)rand() / RAND_MAX) + x_origin + 0.5 * res;
    y = (y_length - res) * ((double)rand() / RAND_MAX) + y_origin + 0.5 * res;

    // Query
    pos = {x, y};
    intermediate_time_1 = std::chrono::steady_clock::now();
    z += grid_map_obj.atPosition("z", pos,
                                 grid_map::InterpolationMethods::INTER_LINEAR) /
         N;
    intermediate_time_2 = std::chrono::steady_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
        intermediate_time_2 - intermediate_time_1);

    if (i > 0) {
      timings[i - 1] = (double)elapsed.count();
    } else {
      start_time = std::chrono::steady_clock::now();
    }
  }

  stop_time = std::chrono::steady_clock::now();
  elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      stop_time - start_time);
  double gm_lin_time = (double)elapsed.count();
  for (int i = 0; i < timings.size(); i++) {
    gm_lin_total_elapsed += timings[i];
    if (fmod(log10(i), 1) == 0) {
      // printf("Duration of lin iteration %d = %.3fus\n", i, 1e6*timings[i]);
    }
  }

  // Start FastTerrainMap linear
  double ftm_lin_total_elapsed = 0;
  z = 0;
  start_time = std::chrono::steady_clock::now();
  for (int i = 0; i < N; i++) {
    // Generate random test point
    x = (x_length - res) * ((double)rand() / RAND_MAX) + x_origin + 0.5 * res;
    y = (y_length - res) * ((double)rand() / RAND_MAX) + y_origin + 0.5 * res;

    // Query
    pos = {x, y};
    intermediate_time_1 = std::chrono::steady_clock::now();
    z += fast_terrain_map.getGroundHeight(x, y) / N;
    intermediate_time_2 = std::chrono::steady_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
        intermediate_time_2 - intermediate_time_1);

    if (i > 0) {
      timings[i - 1] = (double)elapsed.count();
    } else {
      start_time = std::chrono::steady_clock::now();
    }
  }

  stop_time = std::chrono::steady_clock::now();
  elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      stop_time - start_time);
  double ftm_lin_time = (double)elapsed.count();
  for (int i = 0; i < timings.size(); i++) {
    ftm_lin_total_elapsed += timings[i];
    if (fmod(log10(i), 1) == 0) {
      // printf("Duration of lin iteration %d = %.3fus\n", i, 1e6*timings[i]);
    }
  }

  printf("GridMap NN avg (total duration/iterations) = %.3fus\n",
         (double)1e6 * gm_nn_time / N);
  // printf("GridMap NN avg (sum of individual/iterations) = %.3fus\n",
  // (double)1e6*gm_nn_total_elapsed/timings.size());
  printf("GridMap linear avg (total duration/iterations) = %.3fus\n",
         (double)1e6 * gm_lin_time / N);
  // printf("GridMap linear avg (sum of individual/iterations) = %.3fus\n",
  // (double)1e6*gm_lin_total_elapsed/timings.size());
  printf("FastTerrainMap linear avg (total duration/iterations) = %.3fus\n",
         (double)1e6 * ftm_lin_time / N);
  // printf("FastTerrainMap linear avg (sum of individual/iterations) =
  // %.3fus\n", (double)1e6*ftm_lin_total_elapsed/timings.size()); printf("z avg
  // = %.3f\n", (double)z);

  EXPECT_EQ(1 + 1, 2);
}

TEST(FastTerrainMapTest, testConstructor) {
  FastTerrainMap fast_terrain_map;
  EXPECT_EQ(1 + 1, 2);
}

TEST(FastTerrainMapTest, testProjection) {
  FastTerrainMap fast_terrain_map;

  int x_size = 2;
  int y_size = 2;
  std::vector<double> x_data = {-1, 1};
  std::vector<double> y_data = {-1, 1};
  std::vector<double> z_data_vec = {-1, 1};
  std::vector<std::vector<double>> z_data = {z_data_vec, z_data_vec};

  std::vector<double> dx_data_vec = {0, 0};
  std::vector<double> dz_data_vec = {1, 1};
  std::vector<std::vector<double>> dx_data = {dx_data_vec, dx_data_vec};
  std::vector<std::vector<double>> dy_data = dx_data;
  std::vector<std::vector<double>> dz_data = {dz_data_vec, dz_data_vec};

  fast_terrain_map.loadData(x_size, y_size, x_data, y_data, z_data, dx_data,
                            dy_data, dz_data, z_data, dx_data, dy_data,
                            dz_data);

  // Eigen::Vector3d point = {0,0.5,1};
  // Eigen::Vector3d direction = {0.1,0.1,-1};
  Eigen::Vector3d point = {0, 0, 1};
  Eigen::Vector3d direction = {0, 0, -1};

  auto t_start = std::chrono::steady_clock::now();
  Eigen::Vector3d intersection =
      fast_terrain_map.projectToMap(point, direction);
  auto t_end = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_span =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_end -
                                                                t_start);
  // std::cout << "projectToMap took " << time_span.count() << " seconds." <<
  // std::endl;

  // std::cout << "Result is {" << intersection[0] << ", " << intersection[1] <<
  // ", " << intersection[2] << "}" << std::endl;

  EXPECT_EQ(1 + 1, 2);
}

TEST(FastTerrainMapTest, testSlope) {
  FastTerrainMap map;
  double grade = 0.5;
  map.loadSlope(grade);

  Eigen::Vector3d normal;
  normal << -sin(atan(grade)), 0, cos(atan(grade));

  double x = -2;
  double y = 0;
  double z = grade * x;
  EXPECT_TRUE(abs(map.getGroundHeight(x, y) - z) < 1e-6);

  x = 2;
  y = 0;
  z = grade * x;
  EXPECT_TRUE(abs(map.getGroundHeight(x, y) - z) < 1e-6);

  x = 0;
  y = 2;
  z = grade * x;
  EXPECT_TRUE(abs(map.getGroundHeight(x, y) - z) < 1e-6);

  EXPECT_TRUE(map.getSurfaceNormalFilteredEigen(x, y).isApprox(normal));
}

TEST(FastTerrainMapTest, testStep) {
  FastTerrainMap map;
  double height = 0.2;
  map.loadStep(height);

  Eigen::Vector3d normal;
  normal << 0, 0, 1;

  double x = -2;
  double y = 0;
  double z = (x > 0) ? height : 0;
  EXPECT_TRUE(abs(map.getGroundHeight(x, y) - z) < 1e-6);

  x = 2;
  y = 0;
  z = (x > 0) ? height : 0;
  EXPECT_TRUE(abs(map.getGroundHeight(x, y) - z) < 1e-6);

  x = 0;
  y = 2;
  z = (x > 0) ? height : 0;
  EXPECT_TRUE(abs(map.getGroundHeight(x, y) - z) < 1e-6);

  EXPECT_TRUE(map.getSurfaceNormalFilteredEigen(x, y).isApprox(normal));
}
