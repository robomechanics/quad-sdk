#include "spirit_utils/fast_terrain_map.h"
#include <grid_map_core/grid_map_core.hpp>

#include <chrono>
#include <iostream>

FastTerrainMap::FastTerrainMap(){

}

void FastTerrainMap::loadData(int x_size,
  int y_size,
  std::vector<double> x_data,
  std::vector<double> y_data,
  std::vector<std::vector<double>> z_data,
  std::vector<std::vector<double>> dx_data,
  std::vector<std::vector<double>> dy_data,
  std::vector<std::vector<double>> dz_data){

  // Load the data into the object's private variables
  x_size_ = x_size;
  y_size_ = y_size;
  x_data_ = x_data;
  y_data_ = y_data;
  z_data_ = z_data;
  dx_data_ = dx_data;
  dy_data_ = dy_data;
  dz_data_ = dz_data;
}

void FastTerrainMap::loadDataFromGridMap(grid_map::GridMap map){
  // Initialize the data structures for the map
  int x_size = map.getSize()(0);
  int y_size = map.getSize()(1);
  std::vector<double> x_data(x_size);
  std::vector<double> y_data(y_size);
  std::vector<std::vector<double> > z_data(x_size);
  std::vector<std::vector<double> > dx_data(x_size);
  std::vector<std::vector<double> > dy_data(x_size);
  std::vector<std::vector<double> > dz_data(x_size);

  // Load the x and y data coordinates
  for (int i=0; i<x_size; i++)
  {
    grid_map::Index index = {(x_size-1)-i, 0};
    grid_map::Position position;
    map.getPosition(index, position);
    x_data[i] = position.x();
  }
  for (int i=0; i<y_size; i++)
  {
    grid_map::Index index = {0, (y_size-1)-i};
    grid_map::Position position;
    map.getPosition(index, position);
    y_data[i] = position.y();;
  }

  // Loop through the map and get the height and slope info
  for (int i=0; i<x_size; i++)
  {
    for (int j=0; j<y_size; j++)
    {
      grid_map::Index index = {(x_size-1)-i, (y_size-1)-j};
      double height = (double) map.at("elevation", index);
      z_data[i].push_back(height);

      if (map.exists("dx") == true) {
        double dx = (double) map.at("dx", index);
        double dy = (double) map.at("dy", index);
        double dz = (double) map.at("dz", index);
        dx_data[i].push_back(dx);
        dy_data[i].push_back(dy);
        dz_data[i].push_back(dz);
      } else {
        dx_data[i].push_back(0.0);
        dy_data[i].push_back(0.0);
        dz_data[i].push_back(1.0);
      }
    }
  }

  // Update the private terrain member
  x_size_ = x_size;
  y_size_ = y_size;
  x_data_ = x_data;
  y_data_ = y_data;
  z_data_ = z_data;
  dx_data_ = dx_data;
  dy_data_ = dy_data;
  dz_data_ = dz_data;
}

bool FastTerrainMap::isInRange(const double x, const double y) {

  if ((x >= x_data_.front()) && (x <= x_data_.back()) && (y >= y_data_.front()) && (y <= y_data_.back())) {
    return true;
  } else {
    return false;
  }
}

double FastTerrainMap::getGroundHeight(const double x, const double y) {
  // auto t_start = std::chrono::steady_clock::now();
  double x1, x2, y1, y2;

  // Find the correct x values to interpolate between
  int ix=0; int iy = 0;
  for(int i=0;i<x_size_;i++)
  {
      if(x_data_[i]<=x && x<x_data_[i+1])
      {
          x1 = x_data_[i];
          x2 = x_data_[i+1];
          ix = i;
          break;
      }
  }

  // Find the correct y values to interpolate between
  for(int i=0;i<y_size_;i++)
  {
      if(y_data_[i]<=y && y<y_data_[i+1])
      {
          y1 = y_data_[i];
          y2 = y_data_[i+1];
          iy = i;
          break;
      }
  }

  // Perform bilinear interpolation
  double fx1y1 = z_data_[ix][iy];
  double fx1y2 = z_data_[ix][iy+1];
  double fx2y1 = z_data_[ix+1][iy];
  double fx2y2 = z_data_[ix+1][iy+1];
  double height = 1.0/((x2-x1)*(y2-y1))*(fx1y1*(x2-x)*(y2-y) + fx2y1*(x-x1)*(y2-y) + fx1y2*(x2-x)*(y-y1) + fx2y2*(x-x1)*(y-y1));

  // auto t_end = std::chrono::steady_clock::now();
  // std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start);
  // std::cout << "Ground height check took " << time_span.count() << " seconds." << std::endl;
  return height;
}

std::array<double, 3> FastTerrainMap::getSurfaceNormal(double x, double y)
{
    std::array<double, 3> surf_norm;

    double x1, x2, y1, y2;

    int ix=0; int iy = 0;
    for(int i=0;i<x_size_;i++)
    {
        if(x_data_[i]<=x && x<x_data_[i+1])
        {
            x1 = x_data_[i];
            x2 = x_data_[i+1];
            ix = i;
            break;
        }
    }

    for(int i=0;i<y_size_;i++)
    {
        if(y_data_[i]<=y && y<y_data_[i+1])
        {
            y1 = y_data_[i];
            y2 = y_data_[i+1];
            iy = i;
            break;
        }
    }

    double fx_x1y1 = dx_data_[ix][iy];
    double fx_x1y2 = dx_data_[ix][iy+1];
    double fx_x2y1 = dx_data_[ix+1][iy];
    double fx_x2y2 = dx_data_[ix+1][iy+1];

    surf_norm[0] = 1.0/((x2-x1)*(y2-y1))*(fx_x1y1*(x2-x)*(y2-y) + fx_x2y1*(x-x1)*(y2-y) + fx_x1y2*(x2-x)*(y-y1) + fx_x2y2*(x-x1)*(y-y1));

    double fy_x1y1 = dy_data_[ix][iy];
    double fy_x1y2 = dy_data_[ix][iy+1];
    double fy_x2y1 = dy_data_[ix+1][iy];
    double fy_x2y2 = dy_data_[ix+1][iy+1];

    surf_norm[1] = 1.0/((x2-x1)*(y2-y1))*(fy_x1y1*(x2-x)*(y2-y) + fy_x2y1*(x-x1)*(y2-y) + fy_x1y2*(x2-x)*(y-y1) + fy_x2y2*(x-x1)*(y-y1));

    double fz_x1y1 = dz_data_[ix][iy];
    double fz_x1y2 = dz_data_[ix][iy+1];
    double fz_x2y1 = dz_data_[ix+1][iy];
    double fz_x2y2 = dz_data_[ix+1][iy+1];

    surf_norm[2] = 1.0/((x2-x1)*(y2-y1))*(fz_x1y1*(x2-x)*(y2-y) + fz_x2y1*(x-x1)*(y2-y) + fz_x1y2*(x2-x)*(y-y1) + fz_x2y2*(x-x1)*(y-y1));
    return surf_norm;
}

std::vector<double> FastTerrainMap::getXData() {
  return x_data_;
}

std::vector<double> FastTerrainMap::getYData() {
  return y_data_;
}