#ifndef FAST_TERRAIN_MAP_H
#define FAST_TERRAIN_MAP_H

#include <grid_map_core/grid_map_core.hpp>

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
     * @brief Load data from a grid_map::GridMap object into a FastTerrainMap object
     * @param[in] int The number of elements in the x direction
     * @param[in] int The number of elements in the xy direction
     * @param[in] std::vector<double> The vector of x data
     * @param[in] std::vector<double> The vector of y data
     * @param[in] std::vector<std::vector<double>> The nested vector of z data at each [x,y] location
     * @param[in] std::vector<std::vector<double>> The nested vector of the x component of the gradient at each [x,y] location
     * @param[in] std::vector<std::vector<double>> The nested vector of the y component of the gradient at each [x,y] location
     * @param[in] std::vector<std::vector<double>> The nested vector of the z component of the gradient at each [x,y] location
     */
    void loadData(int x_size,
      int y_size,
      std::vector<double> x_data,
      std::vector<double> y_data,
      std::vector<std::vector<double>> z_data,
      std::vector<std::vector<double>> dx_data,
      std::vector<std::vector<double>> dy_data,
      std::vector<std::vector<double>> dz_data);

    /**
     * @brief Load data from a grid_map::GridMap object into a FastTerrainMap object
     * @param[in] grid_map::GridMap object with map data
     */
    void loadDataFromGridMap(grid_map::GridMap map);

    /**
     * @brief Return the ground height at a requested location
     * @param[in] double x location
     * @param[in] double y location
     * @return double ground height at location [x,y]
     */
    double getGroundHeight(const double x,const double y);

    /**
     * @brief Return the surface normal at a requested location
     * @param[in] double x location
     * @param[in] double y location
     * @return std::array<double, 3> surface normal at location [x,y]
     */
    std::array<double, 3> getSurfaceNormal(const double x,const double y);

    /**
     * @brief Return the vector of x_data of the map
     * @return std::vector<double> of x locations in the grid
     */
    std::vector<double> getXData();

    /**
     * @brief Return the vector of y_data of the map
     * @return std::vector<double> of y locations in the grid
     */
    std::vector<double> getYData();

  private:

    /// The number of elements in the x direction
    int x_size_;

    /// The number of elements in the y direction
    int y_size_;

    /// The vector of x data
    std::vector<double> x_data_;

    /// The vector of y data
    std::vector<double> y_data_;

    /// The nested vector of z data at each [x,y] location
    std::vector<std::vector<double>> z_data_;

    /// The nested vector of the x component of the gradient at each [x,y] location
    std::vector<std::vector<double>> dx_data_;

    /// The nested vector of the y component of the gradient at each [x,y] location
    std::vector<std::vector<double>> dy_data_;

    /// The nested vector of the z component of the gradient at each [x,y] location
    std::vector<std::vector<double>> dz_data_;

};

#endif // FAST_TERRAIN_MAP_H
