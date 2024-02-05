#ifndef FAST_TERRAIN_MAP_H
#define FAST_TERRAIN_MAP_H

#include <grid_map_core/grid_map_core.hpp>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <chrono>
#include <quad_utils/function_timer.h>

//! A terrain map class built for fast and efficient sampling
/*!
   FastTerrainMap is a class built for lightweight and efficient sampling of the terrain for height and slope.
*/
class FastTerrainMap {
 public:
  /**
   * @brief Constructor for FastTerrainMap Class
   * @return Constructed object of type FastTerrainMap
   */
  FastTerrainMap();

  /**
   * @brief Load data from a grid_map::GridMap object into a FastTerrainMap
   * object
   * @param[in] int The number of elements in the x direction
   * @param[in] int The number of elements in the xy direction
   * @param[in] std::vector<double> The vector of x data
   * @param[in] std::vector<double> The vector of y data
   * @param[in] std::vector<std::vector<double>> The nested vector of z data at
   * each [x,y] location
   * @param[in] std::vector<std::vector<double>> The nested vector of the x
   * component of the gradient at each [x,y] location
   * @param[in] std::vector<std::vector<double>> The nested vector of the y
   * component of the gradient at each [x,y] location
   * @param[in] std::vector<std::vector<double>> The nested vector of the z
   * component of the gradient at each [x,y] location
   */
  void loadData(int x_size, int y_size, std::vector<double> x_data,
                std::vector<double> y_data,
                std::vector<std::vector<double>> z_data,
                std::vector<std::vector<double>> nx_data,
                std::vector<std::vector<double>> ny_data,
                std::vector<std::vector<double>> nz_data,
                std::vector<std::vector<double>> z_data_filt,
                std::vector<std::vector<double>> nx_data_filt,
                std::vector<std::vector<double>> ny_data_filt,
                std::vector<std::vector<double>> nz_data_filt);

  /**
   * @brief Load in a default terrain map 10x10m, four corners with flat terrain
   */
  void loadFlat();

  /**
   * @brief Load in a default terrain map 10x10m, four corners with elevated
   * terrain
   * @param[in] height Height of elevated terrain
   */
  void loadFlatElevated(double height);

  /**
   * @brief Load in a default terrain map 10x10m, four corners with sloped
   * terrain
   * @param[in] grade Grade of terrain data (grade = tan(slope))
   */
  void loadSlope(double grade);

  /**
   * @brief Load in a terrain map with a step at x = 0
   * @param[in] height Height of step
   */
  void loadStep(double height);

  /**
   * @brief Load data from a grid_map::GridMap object into a FastTerrainMap
   * object
   * @param[in] grid_map::GridMap object with map data
   */
  void loadDataFromGridMap(const grid_map::GridMap map);

  /**
   * @brief Check if map data is defined at a requested location
   * @param[in] double x location
   * @param[in] double y location
   * @return bool location [x,y] is or is not in range
   */
  bool isInRange(const double x, const double y) const;

  /**
   * @brief Return the ground height at a requested location
   * @param[in] double x location
   * @param[in] double y location
   * @return double ground height at location [x,y]
   */
  double getGroundHeight(const double x, const double y) const;

  /**
   * @brief Return the surface normal at a requested location
   * @param[in] double x location
   * @param[in] double y location
   * @return std::array<double, 3> surface normal at location [x,y]
   */
  std::array<double, 3> getSurfaceNormal(const double x, const double y) const;

  /**
   * @brief Return the filtered ground height at a requested location
   * @param[in] double x location
   * @param[in] double y location
   * @return double ground height at location [x,y]
   */
  double getGroundHeightFiltered(const double x, const double y) const;

  /**
   * @brief Return the filtered surface normal at a requested location
   * @param[in] double x location
   * @param[in] double y location
   * @return std::array<double, 3> surface normal at location [x,y]
   */
  std::array<double, 3> getSurfaceNormalFiltered(const double x,
                                                 const double y) const;

  /**
   * @brief Return the filtered surface normal at a requested location
   * @param[in] double x location
   * @param[in] double y location
   * @return std::array<double, 3> surface normal at location [x,y]
   */
  Eigen::Vector3d getSurfaceNormalFilteredEigen(const double x,
                                                const double y) const;

  /**
   * @brief Return the (approximate) intersection of the height map and a
   * vector. Returned point lies exactly on the map but not entirely on the
   * vector.
   * @param[in] point The point at which the vector originates
   * @param[in] direction The direction along which to project the point
   */
  Eigen::Vector3d projectToMap(const Eigen::Vector3d point,
                               const Eigen::Vector3d direction);

  /**
   * @brief Return the vector of x_data of the map
   * @return std::vector<double> of x locations in the grid
   */
  std::vector<double> getXData() const;

  /**
   * @brief Return the vector of y_data of the map
   * @return std::vector<double> of y locations in the grid
   */
  std::vector<double> getYData() const;

  /**
   * @brief Determine if the map is empty
   * @return boolean for map emptiness (true = empty)
   */
  bool isEmpty() const;

 private:
  /**
   * @brief Return the x index
   * @param[in] x X location of the point
   * @return X index of location
   */
  inline int getXIndex(const double x) const {
    return std::max(
        std::min((int)floor((x - x_data_[0]) / x_diff_), x_size_ - 2), 0);
  }

  /**
   * @brief Return the y index
   * @param[in] y Y location of the point
   * @return Y index of location
   */
  inline int getYIndex(const double y) const {
    return std::max(
        std::min((int)floor((y - y_data_[0]) / y_diff_), y_size_ - 2), 0);
  }

  /// The number of elements in the x direction
  int x_size_ = 0;

  /// The number of elements in the y direction
  int y_size_ = 0;

  /// Distance between nodes in x
  double x_diff_;

  /// Distance between nodes in y
  double y_diff_;

  /// The vector of x data
  std::vector<double> x_data_;

  /// The vector of y data
  std::vector<double> y_data_;

  /// The nested vector of z data at each [x,y] location
  std::vector<std::vector<double>> z_data_;

  /// The nested vector of the x component of the gradient at each [x,y]
  /// location
  std::vector<std::vector<double>> nx_data_;

  /// The nested vector of the y component of the gradient at each [x,y]
  /// location
  std::vector<std::vector<double>> ny_data_;

  /// The nested vector of the z component of the gradient at each [x,y]
  /// location
  std::vector<std::vector<double>> nz_data_;

  /// The nested vector of filtered z data at each [x,y] location
  std::vector<std::vector<double>> z_data_filt_;

  /// The nested vector of the x component of the filtered gradient at each
  /// [x,y] location
  std::vector<std::vector<double>> nx_data_filt_;

  /// The nested vector of the y component of the filtered gradient at each
  /// [x,y] location
  std::vector<std::vector<double>> ny_data_filt_;

  /// The nested vector of the z component of the filtered gradient at each
  /// [x,y] location
  std::vector<std::vector<double>> nz_data_filt_;
};

#endif // FAST_TERRAIN_MAP_H
