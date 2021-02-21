#ifndef SPIRIT_MATH_UTILS_H
#define SPIRIT_MATH_UTILS_H

// Just include ros to access a bunch of other functions, fuck good code
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <spirit_msgs/MultiFootState.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/RobotStateTrajectory.h>
#include <spirit_msgs/BodyPlan.h>
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
  void interpHeader(std_msgs::Header header_1,std_msgs::Header header_2, double t_interp,
    std_msgs::Header &interp_header);

  /**
   * @brief Interpolate data between two Odometry messages.
   * @param[in] state_1 First Odometry message
   * @param[in] state_2 Second Odometry message
   * @param[in] t_interp Fraction of time between the messages [0,1]
   * @return Interpolated Odometry message
   */
  void interpOdometry(nav_msgs::Odometry state_1, nav_msgs::Odometry state_2, 
    double t_interp, nav_msgs::Odometry &interp_state);

  /**
   * @brief Interpolate data between two JointState messages.
   * @param[in] state_1 First JointState message
   * @param[in] state_2 Second JointState message
   * @param[in] t_interp Fraction of time between the messages [0,1]
   * @return Interpolated JointState message
   */
  void interpJointState(sensor_msgs::JointState state_1, sensor_msgs::JointState state_2, 
    double t_interp, sensor_msgs::JointState &interp_state);

    /**
   * @brief Interpolate data between two FootState messages.
   * @param[in] state_1 First FootState message
   * @param[in] state_2 Second FootState message
   * @param[in] t_interp Fraction of time between the messages [0,1]
   * @return Interpolated FootState message
   */
  void interpFootState(spirit_msgs::FootState state_1, spirit_msgs::FootState state_2,
    double t_interp, spirit_msgs::FootState &interp_state);

  /**
   * @brief Interpolate data between two RobotState messages.
   * @param[in] state_1 First RobotState message
   * @param[in] state_2 Second RobotState message
   * @param[in] t_interp Fraction of time between the messages [0,1]
   * @return Interpolated RobotState message
   */
  void interpRobotState(spirit_msgs::RobotState state_1, spirit_msgs::RobotState state_2, 
    double t_interp, spirit_msgs::RobotState &interp_state) ;

  /**
   * @brief Interpolate data from a BodyPlan message.
   * @param[in] msg BodyPlan message
   * @param[in] t Time since beginning of trajectory (will return last state if too large)
   * @return nav_msgs::Odometry message
   */
  nav_msgs::Odometry interpBodyPlan(spirit_msgs::BodyPlan msg, double t);

  /**
   * @brief Interpolate data from a robot state trajectory message.
   * @param[in] msg robot state trajectory message
   * @param[in] t Time since beginning of trajectory (will return last state if too large)
   * @return Robot state message
   */
  spirit_msgs::RobotState interpRobotStateTraj(spirit_msgs::RobotStateTrajectory msg, double t);

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