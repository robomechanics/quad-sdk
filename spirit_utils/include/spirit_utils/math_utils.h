#ifndef SPIRIT_MATH_UTILS_H
#define SPIRIT_MATH_UTILS_H

// Just include ros to access a bunch of other functions, fuck good code
#include <ros/ros.h>
#include <Eigen/Dense>
#include <spirit_msgs/MultiFootState.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/RobotStateTrajectory.h>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "spirit_utils/function_timer.h"

namespace math_utils {

  /**
   * @brief Linearly interpolate data (a + t*(b-a)). DOES NOT CHECK FOR EXTRAPOLATION.
   * @param[in] a
   * @param[in] b 
   * @param[in] t 
   * @return Double for interpolated value.
   */
  inline double lerp(double a, double b, double t);

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
   * @brief Interpolate two headers
   * @param[in] header_1 First header message
   * @param[in] header_2 Second header message
   * @param[in] t_interp Fraction of time between the messages [0,1]
   * @return Interpolated header
   */
  std_msgs::Header interpHeader(std_msgs::Header header_1,
    std_msgs::Header header_2,double t_interp);

    /**
   * @brief Interpolate data between two FootState messages.
   * @param[in] state_1 First FootState message
   * @param[in] state_2 Second FootState message
   * @param[in] t_interp Fraction of time between the messages [0,1]
   * @return Interpolated FootState message
   */
  spirit_msgs::FootState interpFootState(spirit_msgs::FootState state_1,
    spirit_msgs::FootState state_2, double t_interp);

  /**
   * @brief Interpolate data between two RobotState messages.
   * @param[in] state_1 First RobotState message
   * @param[in] state_2 Second RobotState message
   * @param[in] t_interp Fraction of time between the messages [0,1]
   * @return Interpolated RobotState message
   */
  spirit_msgs::RobotState interpRobotState(spirit_msgs::RobotState state_1,
    spirit_msgs::RobotState state_2, double t_interp);

  /**
   * @brief Interpolate data from a robot state trajectory message.
   * @param[in] msg robot state trajectory message
   * @param[in] t Time since beginning of trajectory (will return last state if too large)
   * @return Robot state message
   */
  spirit_msgs::RobotState interpRobotStateTraj(spirit_msgs::RobotStateTrajectory msg, double t);


}

#endif // SPIRIT_MATH_UTILS_H