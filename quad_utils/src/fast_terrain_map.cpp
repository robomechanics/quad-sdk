#include "quad_utils/fast_terrain_map.h"

#include <grid_map_core/grid_map_core.hpp>
#include <iostream>

FastTerrainMap::FastTerrainMap() {}

void FastTerrainMap::loadData(int x_size, int y_size,
                              std::vector<double> x_data,
                              std::vector<double> y_data,
                              std::vector<std::vector<double>> z_data,
                              std::vector<std::vector<double>> nx_data,
                              std::vector<std::vector<double>> ny_data,
                              std::vector<std::vector<double>> nz_data,
                              std::vector<std::vector<double>> z_data_filt,
                              std::vector<std::vector<double>> nx_data_filt,
                              std::vector<std::vector<double>> ny_data_filt,
                              std::vector<std::vector<double>> nz_data_filt) {
  // Load the data into the object's private variables
  x_size_ = x_size;
  y_size_ = y_size;
  x_data_ = x_data;
  y_data_ = y_data;

  x_diff_ = x_data_[1] - x_data_[0];
  y_diff_ = y_data_[1] - y_data_[0];

  z_data_ = z_data;
  nx_data_ = nx_data;
  ny_data_ = ny_data;
  nz_data_ = nz_data;

  z_data_filt_ = z_data_filt;
  nx_data_filt_ = nx_data_filt;
  ny_data_filt_ = ny_data_filt;
  nz_data_filt_ = nz_data_filt;
}

void FastTerrainMap::loadFlat() {
  int x_size = 2;
  int y_size = 2;
  std::vector<double> x_data = {-5, 5};
  std::vector<double> y_data = {-5, 5};
  std::vector<double> z_data_vec = {0, 0};
  std::vector<double> nz_data_vec = {1, 1};
  std::vector<std::vector<double>> z_data = {z_data_vec, z_data_vec};
  std::vector<std::vector<double>> nx_data = z_data;
  std::vector<std::vector<double>> ny_data = z_data;
  std::vector<std::vector<double>> nz_data = {nz_data_vec, nz_data_vec};
  std::vector<std::vector<double>> z_data_filt = z_data;
  std::vector<std::vector<double>> nx_data_filt = nx_data;
  std::vector<std::vector<double>> ny_data_filt = ny_data;
  std::vector<std::vector<double>> nz_data_filt = nz_data;

  this->loadData(x_size, y_size, x_data, y_data, z_data, nx_data, ny_data,
                 nz_data, z_data_filt, nx_data_filt, ny_data_filt,
                 nz_data_filt);
}

void FastTerrainMap::loadFlatElevated(double height) {
  int x_size = 2;
  int y_size = 2;
  std::vector<double> x_data = {-5, 5};
  std::vector<double> y_data = {-5, 5};
  std::vector<double> z_data_vec = {height, height};
  std::vector<double> nx_data_vec = {0, 0};
  std::vector<double> nz_data_vec = {1, 1};
  std::vector<std::vector<double>> z_data = {z_data_vec, z_data_vec};
  std::vector<std::vector<double>> nx_data = {nx_data_vec, nx_data_vec};
  std::vector<std::vector<double>> ny_data = nx_data;
  std::vector<std::vector<double>> nz_data = {nz_data_vec, nz_data_vec};
  std::vector<std::vector<double>> z_data_filt = z_data;
  std::vector<std::vector<double>> nx_data_filt = nx_data;
  std::vector<std::vector<double>> ny_data_filt = ny_data;
  std::vector<std::vector<double>> nz_data_filt = nz_data;

  this->loadData(x_size, y_size, x_data, y_data, z_data, nx_data, ny_data,
                 nz_data, z_data_filt, nx_data_filt, ny_data_filt,
                 nz_data_filt);
}

void FastTerrainMap::loadSlope(double grade) {
  double slope = atan(grade);
  int x_size = 2;
  int y_size = 2;
  double length = 5;
  std::vector<double> x_data = {-length, length};
  std::vector<double> y_data = {-length, length};
  std::vector<double> z_data_vec_1 = {-length * grade, -length * grade};
  std::vector<double> z_data_vec_2 = {length * grade, length * grade};
  std::vector<double> nx_data_vec = {-sin(slope), -sin(slope)};
  std::vector<double> ny_data_vec = {0, 0};
  std::vector<double> nz_data_vec = {cos(slope), cos(slope)};
  std::vector<std::vector<double>> z_data = {z_data_vec_1, z_data_vec_2};
  std::vector<std::vector<double>> nx_data = {nx_data_vec, nx_data_vec};
  std::vector<std::vector<double>> ny_data = {ny_data_vec, ny_data_vec};
  std::vector<std::vector<double>> nz_data = {nz_data_vec, nz_data_vec};
  std::vector<std::vector<double>> z_data_filt = {z_data_vec_1, z_data_vec_2};
  std::vector<std::vector<double>> nx_data_filt = {nx_data_vec, nx_data_vec};
  std::vector<std::vector<double>> ny_data_filt = {ny_data_vec, ny_data_vec};
  std::vector<std::vector<double>> nz_data_filt = {nz_data_vec, nz_data_vec};

  this->loadData(x_size, y_size, x_data, y_data, z_data, nx_data, ny_data,
                 nz_data, z_data_filt, nx_data_filt, ny_data_filt,
                 nz_data_filt);
}

void FastTerrainMap::loadStep(double height) {
  double res = 0.05;
  double length = 2;
  int x_size = length * 2 / res + 1;
  int y_size = x_size;

  std::vector<double> x_data;
  std::vector<double> y_data;
  std::vector<std::vector<double>> z_data(x_size);
  std::vector<std::vector<double>> nx_data(x_size);
  std::vector<std::vector<double>> ny_data(x_size);
  std::vector<std::vector<double>> nz_data(x_size);
  std::vector<std::vector<double>> z_data_filt(x_size);
  std::vector<std::vector<double>> nx_data_filt(x_size);
  std::vector<std::vector<double>> ny_data_filt(x_size);
  std::vector<std::vector<double>> nz_data_filt(x_size);

  for (int i = 0; i < x_size; i++) {
    double x = i * res - length;
    x_data.push_back(i * res - length);
    y_data.push_back(i * res - length);

    z_data[i].resize(y_size);
    nx_data[i].resize(y_size);
    ny_data[i].resize(y_size);
    nz_data[i].resize(y_size);
    z_data_filt[i].resize(y_size);
    nx_data_filt[i].resize(y_size);
    ny_data_filt[i].resize(y_size);
    nz_data_filt[i].resize(y_size);

    for (int j = 0; j < y_size; j++) {
      double y = j * res - length;
      z_data[i][j] = (x > 0) ? height : 0;
      nx_data[i][j] = 0;
      ny_data[i][j] = 0;
      nz_data[i][j] = 1;
    }
  }

  z_data_filt = z_data;
  nx_data_filt = nx_data;
  ny_data_filt = ny_data;
  nz_data_filt = nz_data;

  this->loadData(x_size, y_size, x_data, y_data, z_data, nx_data, ny_data,
                 nz_data, z_data_filt, nx_data_filt, ny_data_filt,
                 nz_data_filt);
}

void FastTerrainMap::loadDataFromGridMap(const grid_map::GridMap map) {
  // Initialize the data structures for the map
  int x_size = map.getSize()(0);
  int y_size = map.getSize()(1);
  std::vector<double> x_data(x_size);
  std::vector<double> y_data(y_size);
  std::vector<std::vector<double>> z_data(x_size);
  std::vector<std::vector<double>> nx_data(x_size);
  std::vector<std::vector<double>> ny_data(x_size);
  std::vector<std::vector<double>> nz_data(x_size);
  std::vector<std::vector<double>> z_data_filt(x_size);
  std::vector<std::vector<double>> nx_data_filt(x_size);
  std::vector<std::vector<double>> ny_data_filt(x_size);
  std::vector<std::vector<double>> nz_data_filt(x_size);

  // Load the x and y data coordinates
  for (int i = 0; i < x_size; i++) {
    grid_map::Index index = {(x_size - 1) - i, 0};
    grid_map::Position position;
    map.getPosition(index, position);
    x_data[i] = position.x();
  }
  for (int i = 0; i < y_size; i++) {
    grid_map::Index index = {0, (y_size - 1) - i};
    grid_map::Position position;
    map.getPosition(index, position);
    y_data[i] = position.y();
  }

  // Loop through the map and get the height and slope info
  for (int i = 0; i < x_size; i++) {
    for (int j = 0; j < y_size; j++) {
      grid_map::Index index = {(x_size - 1) - i, (y_size - 1) - j};
      double height = (double)map.at("z_inpainted", index);
      z_data[i].push_back(height);

      if (map.exists("normal_vectors_x") == true) {
        double nx = (double)map.at("normal_vectors_x", index);
        double ny = (double)map.at("normal_vectors_y", index);
        double nz = (double)map.at("normal_vectors_z", index);
        nx_data[i].push_back(nx);
        ny_data[i].push_back(ny);
        nz_data[i].push_back(nz);
      } else {
        nx_data[i].push_back(0.0);
        ny_data[i].push_back(0.0);
        nz_data[i].push_back(1.0);
      }

      if (map.exists("z_smooth") == true) {
        double z_filt = (double)map.at("z_smooth", index);
        double nx_filt = (double)map.at("smooth_normal_vectors_x", index);
        double ny_filt = (double)map.at("smooth_normal_vectors_y", index);
        double nz_filt = (double)map.at("smooth_normal_vectors_z", index);
        z_data_filt[i].push_back(z_filt);
        nx_data_filt[i].push_back(nx_filt);
        ny_data_filt[i].push_back(ny_filt);
        nz_data_filt[i].push_back(nz_filt);
      } else {
        z_data_filt[i].push_back(height);
        nx_data_filt[i].push_back(0.0);
        ny_data_filt[i].push_back(0.0);
        nz_data_filt[i].push_back(1.0);
      }
    }
  }

  // Update the private terrain member
  x_size_ = x_size;
  y_size_ = y_size;
  x_data_ = x_data;
  y_data_ = y_data;

  x_diff_ = x_data_[1] - x_data_[0];
  y_diff_ = y_data_[1] - y_data_[0];

  z_data_ = z_data;
  nx_data_ = nx_data;
  ny_data_ = ny_data;
  nz_data_ = nz_data;

  z_data_filt_ = z_data_filt;
  nx_data_filt_ = nx_data_filt;
  ny_data_filt_ = ny_data_filt;
  nz_data_filt_ = nz_data_filt;
}

bool FastTerrainMap::isInRange(const double x, const double y) const {
  double epsilon = 0.5;
  if (((x - epsilon) >= x_data_.front()) && ((x + epsilon) <= x_data_.back()) &&
      ((y - epsilon) >= y_data_.front()) && ((y + epsilon) <= y_data_.back())) {
    return true;
  } else {
    return false;
  }
}

double FastTerrainMap::getGroundHeight(const double x, const double y) const {
  // quad_utils::FunctionTimer timer(__FUNCTION__);
  int ix = getXIndex(x);
  int iy = getYIndex(y);

  double x1 = x_data_[ix];
  double x2 = x_data_[ix + 1];
  double y1 = y_data_[iy];
  double y2 = y_data_[iy + 1];

  // Perform bilinear interpolation
  double fx1y1 = z_data_[ix][iy];
  double fx1y2 = z_data_[ix][iy + 1];
  double fx2y1 = z_data_[ix + 1][iy];
  double fx2y2 = z_data_[ix + 1][iy + 1];
  double height = 1.0 / ((x2 - x1) * (y2 - y1)) *
                  (fx1y1 * (x2 - x) * (y2 - y) + fx2y1 * (x - x1) * (y2 - y) +
                   fx1y2 * (x2 - x) * (y - y1) + fx2y2 * (x - x1) * (y - y1));

  // timer.reportStatistics();
  return height;
}

double FastTerrainMap::getGroundHeightFiltered(const double x,
                                               const double y) const {
  // quad_utils::FunctionTimer timer(__FUNCTION__);

  int ix = getXIndex(x);
  int iy = getYIndex(y);
  double x1 = x_data_[ix];
  double x2 = x_data_[ix + 1];
  double y1 = y_data_[iy];
  double y2 = y_data_[iy + 1];

  // Perform bilinear interpolation
  double fx1y1 = z_data_filt_[ix][iy];
  double fx1y2 = z_data_filt_[ix][iy + 1];
  double fx2y1 = z_data_filt_[ix + 1][iy];
  double fx2y2 = z_data_filt_[ix + 1][iy + 1];
  double height = 1.0 / ((x2 - x1) * (y2 - y1)) *
                  (fx1y1 * (x2 - x) * (y2 - y) + fx2y1 * (x - x1) * (y2 - y) +
                   fx1y2 * (x2 - x) * (y - y1) + fx2y2 * (x - x1) * (y - y1));

  // timer.reportStatistics();
  return height;
}

std::array<double, 3> FastTerrainMap::getSurfaceNormal(const double x,
                                                       const double y) const {
  std::array<double, 3> surf_norm;

  int ix = getXIndex(x);
  int iy = getYIndex(y);
  double x1 = x_data_[ix];
  double x2 = x_data_[ix + 1];
  double y1 = y_data_[iy];
  double y2 = y_data_[iy + 1];

  double fx_x1y1 = nx_data_[ix][iy];
  double fx_x1y2 = nx_data_[ix][iy + 1];
  double fx_x2y1 = nx_data_[ix + 1][iy];
  double fx_x2y2 = nx_data_[ix + 1][iy + 1];

  surf_norm[0] =
      1.0 / ((x2 - x1) * (y2 - y1)) *
      (fx_x1y1 * (x2 - x) * (y2 - y) + fx_x2y1 * (x - x1) * (y2 - y) +
       fx_x1y2 * (x2 - x) * (y - y1) + fx_x2y2 * (x - x1) * (y - y1));

  double fy_x1y1 = ny_data_[ix][iy];
  double fy_x1y2 = ny_data_[ix][iy + 1];
  double fy_x2y1 = ny_data_[ix + 1][iy];
  double fy_x2y2 = ny_data_[ix + 1][iy + 1];

  surf_norm[1] =
      1.0 / ((x2 - x1) * (y2 - y1)) *
      (fy_x1y1 * (x2 - x) * (y2 - y) + fy_x2y1 * (x - x1) * (y2 - y) +
       fy_x1y2 * (x2 - x) * (y - y1) + fy_x2y2 * (x - x1) * (y - y1));

  double fz_x1y1 = nz_data_[ix][iy];
  double fz_x1y2 = nz_data_[ix][iy + 1];
  double fz_x2y1 = nz_data_[ix + 1][iy];
  double fz_x2y2 = nz_data_[ix + 1][iy + 1];

  surf_norm[2] =
      1.0 / ((x2 - x1) * (y2 - y1)) *
      (fz_x1y1 * (x2 - x) * (y2 - y) + fz_x2y1 * (x - x1) * (y2 - y) +
       fz_x1y2 * (x2 - x) * (y - y1) + fz_x2y2 * (x - x1) * (y - y1));
  return surf_norm;
}

std::array<double, 3> FastTerrainMap::getSurfaceNormalFiltered(
    const double x, const double y) const {
  std::array<double, 3> surf_norm;

  int ix = getXIndex(x);
  int iy = getYIndex(y);
  double x1 = x_data_[ix];
  double x2 = x_data_[ix + 1];
  double y1 = y_data_[iy];
  double y2 = y_data_[iy + 1];

  double fx_x1y1 = nx_data_filt_[ix][iy];
  double fx_x1y2 = nx_data_filt_[ix][iy + 1];
  double fx_x2y1 = nx_data_filt_[ix + 1][iy];
  double fx_x2y2 = nx_data_filt_[ix + 1][iy + 1];

  surf_norm[0] =
      1.0 / ((x2 - x1) * (y2 - y1)) *
      (fx_x1y1 * (x2 - x) * (y2 - y) + fx_x2y1 * (x - x1) * (y2 - y) +
       fx_x1y2 * (x2 - x) * (y - y1) + fx_x2y2 * (x - x1) * (y - y1));

  double fy_x1y1 = ny_data_filt_[ix][iy];
  double fy_x1y2 = ny_data_filt_[ix][iy + 1];
  double fy_x2y1 = ny_data_filt_[ix + 1][iy];
  double fy_x2y2 = ny_data_filt_[ix + 1][iy + 1];

  surf_norm[1] =
      1.0 / ((x2 - x1) * (y2 - y1)) *
      (fy_x1y1 * (x2 - x) * (y2 - y) + fy_x2y1 * (x - x1) * (y2 - y) +
       fy_x1y2 * (x2 - x) * (y - y1) + fy_x2y2 * (x - x1) * (y - y1));

  double fz_x1y1 = nz_data_filt_[ix][iy];
  double fz_x1y2 = nz_data_filt_[ix][iy + 1];
  double fz_x2y1 = nz_data_filt_[ix + 1][iy];
  double fz_x2y2 = nz_data_filt_[ix + 1][iy + 1];

  surf_norm[2] =
      1.0 / ((x2 - x1) * (y2 - y1)) *
      (fz_x1y1 * (x2 - x) * (y2 - y) + fz_x2y1 * (x - x1) * (y2 - y) +
       fz_x1y2 * (x2 - x) * (y - y1) + fz_x2y2 * (x - x1) * (y - y1));
  return surf_norm;
}

Eigen::Vector3d FastTerrainMap::getSurfaceNormalFilteredEigen(
    const double x, const double y) const {
  Eigen::Vector3d surf_norm;

  int ix = getXIndex(x);
  int iy = getYIndex(y);
  double x1 = x_data_[ix];
  double x2 = x_data_[ix + 1];
  double y1 = y_data_[iy];
  double y2 = y_data_[iy + 1];

  double fx_x1y1 = nx_data_filt_[ix][iy];
  double fx_x1y2 = nx_data_filt_[ix][iy + 1];
  double fx_x2y1 = nx_data_filt_[ix + 1][iy];
  double fx_x2y2 = nx_data_filt_[ix + 1][iy + 1];

  surf_norm[0] =
      1.0 / ((x2 - x1) * (y2 - y1)) *
      (fx_x1y1 * (x2 - x) * (y2 - y) + fx_x2y1 * (x - x1) * (y2 - y) +
       fx_x1y2 * (x2 - x) * (y - y1) + fx_x2y2 * (x - x1) * (y - y1));

  double fy_x1y1 = ny_data_filt_[ix][iy];
  double fy_x1y2 = ny_data_filt_[ix][iy + 1];
  double fy_x2y1 = ny_data_filt_[ix + 1][iy];
  double fy_x2y2 = ny_data_filt_[ix + 1][iy + 1];

  surf_norm[1] =
      1.0 / ((x2 - x1) * (y2 - y1)) *
      (fy_x1y1 * (x2 - x) * (y2 - y) + fy_x2y1 * (x - x1) * (y2 - y) +
       fy_x1y2 * (x2 - x) * (y - y1) + fy_x2y2 * (x - x1) * (y - y1));

  double fz_x1y1 = nz_data_filt_[ix][iy];
  double fz_x1y2 = nz_data_filt_[ix][iy + 1];
  double fz_x2y1 = nz_data_filt_[ix + 1][iy];
  double fz_x2y2 = nz_data_filt_[ix + 1][iy + 1];

  surf_norm[2] =
      1.0 / ((x2 - x1) * (y2 - y1)) *
      (fz_x1y1 * (x2 - x) * (y2 - y) + fz_x2y1 * (x - x1) * (y2 - y) +
       fz_x1y2 * (x2 - x) * (y - y1) + fz_x2y2 * (x - x1) * (y - y1));
  return surf_norm;
}

Eigen::Vector3d FastTerrainMap::projectToMap(const Eigen::Vector3d point,
                                             const Eigen::Vector3d direction) {
  // quad_utils::FunctionTimer timer(__FUNCTION__);

  Eigen::Vector3d direction_norm = direction;
  direction_norm.normalize();
  Eigen::Vector3d result = point;
  Eigen::Vector3d new_point = point;
  Eigen::Vector3d old_point = point;
  double step_size = 0.01;
  double clearance = 0;
  while (clearance >= 0) {
    old_point = new_point;
    for (int i = 0; i < 3; i++) {
      new_point[i] += direction_norm[i] * step_size;
    }
    if (isInRange(new_point[0], new_point[1])) {
      clearance = new_point[2] - getGroundHeight(new_point[0], new_point[1]);
    } else {
      result = {old_point[0], old_point[1],
                -std::numeric_limits<double>::max()};
      ROS_WARN_THROTTLE(0.5, "Tried to project to a point off the map.");
      return result;
    }
  }

  result = {old_point[0], old_point[1],
            getGroundHeight(old_point[0], old_point[1])};

  // timer.reportStatistics();
  return result;
}

std::vector<double> FastTerrainMap::getXData() const { return x_data_; }

std::vector<double> FastTerrainMap::getYData() const { return y_data_; }

bool FastTerrainMap::isEmpty() const {
  if (x_size_ == 0 || y_size_ == 0) {
    return true;
  } else {
    return false;
  }
}
