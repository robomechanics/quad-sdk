#ifndef QUAD_MATH_UTILS_H
#define QUAD_MATH_UTILS_H

// Just include ros to access a bunch of other functions, fuck good code
#include <nav_msgs/Odometry.h>
#include <quad_msgs/MultiFootPlanContinuous.h>
#include <quad_msgs/MultiFootState.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/RobotState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "quad_utils/function_timer.h"
#include "quad_utils/quad_kd.h"

namespace math_utils {

/**
 * @brief Linearly interpolate data (a + t*(b-a)). DOES NOT CHECK FOR
 * EXTRAPOLATION.
 * @param[in] a
 * @param[in] b
 * @param[in] t
 * @return Double for interpolated value.
 */
inline double lerp(double a, double b, double t) { return (a + t * (b - a)); }

/**
 * @brief Wrap to [0,2*pi)
 * @param[in] val value to wrap
 * @return Wrapped value
 */
inline double wrapTo2Pi(double val) {
  return fmod(2 * M_PI + fmod(val, 2 * M_PI), 2 * M_PI);
}

/**
 * @brief Wrap to [-pi,pi)
 * @param[in] val value to wrap
 * @return Wrapped value
 */
inline double wrapToPi(double val) {
  return -M_PI + wrapTo2Pi(val + M_PI);
  // double new_val = fmod(val + M_PI, 2*M_PI);
  // while (new_val < 0) {
  //   new_val += 2*M_PI;
  // }
  // return new_val-M_PI;
}

/**
 * @brief Wrap data to [-pi,pi)
 * @param[in] data data to wrap
 * @return Wrapped data
 */
inline std::vector<double> wrapToPi(const std::vector<double> &data) {
  std::vector<double> data_wrapped = data;
  for (int i = 0; i < data.size(); i++) {
    data_wrapped[i] = wrapToPi(data[i]);
  }
  return data_wrapped;
}

/**
 * @brief Interpolate data from column vectors contained in a matrix (vector of
 * row vectors) provided an input vector and query point
 * @param[in] input_vec Input vector
 * @param[in] output_mat Collection of row vectors such that each row
 * corresponds to exactly one element in the input vector
 * @param[in] input_val Query point
 * @return Vector of interpolated values
 */
std::vector<double> interpMat(
    const std::vector<double> &input_vec,
    const std::vector<std::vector<double>> &output_mat,
    const double query_point);

/**
 * @brief Interpolate data from column vectors contained in a matrix (vector of
 * row vectors) provided an input vector and query point
 * @param[in] input_vec Input vector
 * @param[in] output_mat Collection of row vectors such that each row
 * corresponds to exactly one element in the input vector
 * @param[in] input_val Query point
 * @return Vector of interpolated values
 */
Eigen::Vector3d interpVector3d(const std::vector<double> &input_vec,
                               const std::vector<Eigen::Vector3d> &output_mat,
                               const double query_point);

/**
 * @brief Interpolate data from Eigen::Vector3d contained in a matrix (vector of
 * row vectors) provided an input vector and query point
 * @param[in] input_vec Input vector
 * @param[in] output_mat Collection of row vectors such that each row
 * corresponds to exactly one element in the input vector
 * @param[in] input_val Query point
 * @return Vector of interpolated values
 */
std::vector<Eigen::Vector3d> interpMatVector3d(
    const std::vector<double> &input_vec,
    const std::vector<std::vector<Eigen::Vector3d>> &output_mat,
    const double query_point);

/**
 * @brief Obtain the correct int within a parameterized vector of ints
 * @param[in] input_vec Input vector
 * @param[in] output_vec Output vector of ints
 * @param[in] input_val Query point
 * @return Correct output int corresponsing to the query point
 */
int interpInt(const std::vector<double> &input_vec,
              std::vector<int> &output_vec, const double query_point);

/**
 * @brief Filter a stl vector with a moving average window.
 * @param[in] data Input vector
 * @param[in] window_size the width of the moving window. If even, function will
 * add one to maintain symmetry
 * @return Vector of filtered values
 */
std::vector<double> movingAverageFilter(const std::vector<double> &data,
                                        int window_size);

/**
 * @brief Differentiate an input vector with the central difference method
 * @param[in] data Input vector
 * @param[in] dt The (constant) timestep between values in data.
 * @return Vector of differentiated signal
 */
std::vector<double> centralDiff(const std::vector<double> &data, double dt);

/**
 * @brief Wrap a given scalar to within PI of a given target by adding or
 * subtracting 2*PI
 *
 * @tparam ScalarType
 * @param[in] val Value to be wrapped to val_target
 * @param[in] val_target Target to wrap towards
 */
template <typename ScalarType>
void wrapToTarget(ScalarType &val, const ScalarType &val_target = 0.0) {
  while (val_target - val > M_PI) {
    val += 2.0 * M_PI;
  }
  while (val_target - val < -M_PI) {
    val -= 2.0 * M_PI;
  }
}

/**
 * @brief Unwrap a phase variable by filtering out differences > pi
 * @param[in] vec Input vector containing a wrapped signal
 * @return Flag for if the vector was modified by unwrapping
 */
template <typename VecType>
bool unwrapVector(VecType &vec) {
  bool modified = false;
  for (int i = 1; i < vec.size(); i++) {
    double diff = vec[i] - vec[i - 1];
    if (diff > M_PI) {
      modified = true;
      for (int j = i; j < vec.size(); j++) {
        vec[j] = vec[j] - 2 * M_PI;
      }
    } else if (diff < -M_PI) {
      modified = true;
      for (int j = i; j < vec.size(); j++) {
        vec[j] = vec[j] + 2 * M_PI;
      }
    }
  }
  return modified;
}

/**
 * @brief Unwrap a phase variable by filtering out differences > pi
 * @param[in] vec Input vector containing a wrapped signal
 * @return Vector of unwrapped signal
 */
template <typename VecType>
VecType getUnwrappedVector(const VecType &vec) {
  VecType vec_unwrapped = vec;
  unwrapVector(vec_unwrapped);
  return vec_unwrapped;
}

/**
 * @brief Selective damping least square matrix inverse
 * @param[in] jacobian Input matrix
 * @return Pseudo-inverse of the input matrix
 */
Eigen::MatrixXd sdlsInv(const Eigen::MatrixXd &jacobian);
}  // namespace math_utils

#endif  // QUAD_MATH_UTILS_H