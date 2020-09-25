#ifndef FAST_INTERPOLATOR_H
#define FAST_INTERPOLATOR_H

#include <vector>
#include <algorithm>
#include <functional>
#include <string>
#include <stdexcept>
#include <iostream>

//! A custom interpolation library
/*!
   Implements linear interpolation
*/
class FastInterpolator {
public:
  /**
   * @brief Constructor for FastInterpolate class
   * @return Constructed object of type FastInterpolator
   */
  FastInterpolator();

  /**
   * @brief Linear interpolation with single interpolated point
   * @param[in] X vector of sorted inputs to f(X) where f is underlying function
   * @param[in] Y vector of outputs of f(X) = Y
   * @param[in] Xq point to interpolate at
   * @return interpolated value at Xq
   */
  double interp1(std::vector<double> X, std::vector<double> Y, double Xq);

  /**
   * @brief Linear interpolation with vector of input points
   * @param[in] X vector of sorted inputs to f(X) where f is underlying function
   * @param[in] Y vector of outputs of f(X) = Y
   * @param[in] Xq vector of points to interpolate at
   * @return vector of interpolated values at Xq
   */
  std::vector<double> interp1(std::vector<double> X, std::vector<double> Y, std::vector<double> Xq);

};


#endif // INTERPOLATOR_H