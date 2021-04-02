#ifndef SPIRIT_MATH_UTILS_H
#define SPIRIT_MATH_UTILS_H

// Just include ros to access a bunch of other functions, fuck good code
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <spirit_msgs/MultiFootState.h>
#include <spirit_msgs/MultiFootPlanContinuous.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/RobotStateTrajectory.h>
#include <spirit_msgs/BodyPlan.h>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "spirit_utils/function_timer.h"
#include "spirit_utils/kinematics.h"

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

}

#endif // SPIRIT_MATH_UTILS_H