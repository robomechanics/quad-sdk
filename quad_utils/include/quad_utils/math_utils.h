#ifndef SPIRIT_MATH_UTILS_H
#define SPIRIT_MATH_UTILS_H

// Just include ros to access a bunch of other functions, fuck good code
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <quad_msgs/MultiFootState.h>
#include <quad_msgs/MultiFootPlanContinuous.h>
#include <quad_msgs/RobotState.h>
#include <quad_msgs/RobotStateTrajectory.h>
#include <quad_msgs/RobotPlan.h>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "quad_utils/function_timer.h"
#include "quad_utils/quad_kd.h"

namespace math_utils {

  /**
   * @brief Linearly interpolate data (a + t*(b-a)). DOES NOT CHECK FOR EXTRAPOLATION.
   * @param[in] a
   * @param[in] b 
   * @param[in] t 
   * @return Double for interpolated value.
   */
    inline double lerp(double a, double b, double t) {
      return (a+t*(b-a));
    }

  /**
   * @brief Wrap to [0,2*pi)
   * @param[in] val value to wrap
   * @return Wrapped value
   */
    inline double wrapTo2Pi(double val) {
      return fmod(2*M_PI + fmod(val, 2*M_PI), 2*M_PI);
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
    inline std::vector<double> wrapToPi(std::vector<double> data) {
      std::vector<double> data_wrapped = data;
      for (int i = 0; i < data.size(); i++) {
        data_wrapped[i] = wrapToPi(data[i]);
      }
      return data_wrapped;
    }
  

  /**
   * @brief Interpolate data from column vectors contained in a matrix (vector of row vectors) provided an input vector and query point
   * @param[in] input_vec Input vector
   * @param[in] output_mat Collection of row vectors such that each row corresponds to exactly one element in the input vector
   * @param[in] input_val Query point
   * @return Vector of interpolated values
   */
  std::vector<double> interpMat(std::vector<double> input_vec, 
    std::vector<std::vector<double>> output_mat, double query_point);

  /**
   * @brief Interpolate data from column vectors contained in a matrix (vector of row vectors) provided an input vector and query point
   * @param[in] input_vec Input vector
   * @param[in] output_mat Collection of row vectors such that each row corresponds to exactly one element in the input vector
   * @param[in] input_val Query point
   * @return Vector of interpolated values
   */
  Eigen::Vector3d interpVector3d(std::vector<double> input_vec, std::vector<Eigen::Vector3d> output_mat, double query_point);

  /**
   * @brief Interpolate data from Eigen::Vector3d contained in a matrix (vector of row vectors) provided an input vector and query point
   * @param[in] input_vec Input vector
   * @param[in] output_mat Collection of row vectors such that each row corresponds to exactly one element in the input vector
   * @param[in] input_val Query point
   * @return Vector of interpolated values
   */
  std::vector<Eigen::Vector3d> interpMatVector3d(std::vector<double> input_vec, 
    std::vector<std::vector<Eigen::Vector3d>> output_mat, double query_point);

  /**
   * @brief Obtain the correct int within a parameterized vector of ints
   * @param[in] input_vec Input vector
   * @param[in] output_vec Output vector of ints
   * @param[in] input_val Query point
   * @return Correct output int corresponsing to the query point
   */
  int interpInt(std::vector<double> input_vec,
  std::vector<int> output_vec, double query_point);

  /**
   * @brief Filter a stl vector with a moving average window.
   * @param[in] data Input vector
   * @param[in] window_size the width of the moving window. If even, function will add one to maintain symmetry
   * @return Vector of filtered values
   */
  std::vector<double> movingAverageFilter(std::vector<double> data, int window_size);
  
  /**
   * @brief Differentiate an input vector with the central difference method
   * @param[in] data Input vector
   * @param[in] dt The (constant) timestep between values in data.
   * @return Vector of differentiated signal
   */
  std::vector<double> centralDiff(std::vector<double> data, double dt);

  /**
   * @brief Unwrap a phase variable by filtering out differences > pi
   * @param[in] data Input vector containing a wrapped signal
   * @return Vector of unwrapped signal
   */
  std::vector<double> unwrap(std::vector<double> data);

  /**
   * @brief Selective damping least square matrix inverse
   * @param[in] jacobian Input matrix
   * @return Pseudo-inverse of the input matrix
   */
  Eigen::MatrixXd sdlsInv(const Eigen::MatrixXd &jacobian);
}

#endif // SPIRIT_MATH_UTILS_H