#ifndef COMP_FILTER_H
#define COMP_FILTER_H

#include <robot_driver/estimators/state_estimator.h>

//! Implements complementary filter as an estimator within the ROS framework.
class CompFilterEstimator : public StateEstimator {
 public:
  /**
   * @brief Constructor for CompFilterEstimator
   * @return Constructed object of type CompFilterEstimator
   */
  CompFilterEstimator();

  /**
   * @brief initialize Complementary Filter
   * @param[in] nh ros::NodeHandle to load parameters from yaml file
   */
  void init(ros::NodeHandle& nh);

  /**
   * @brief helper function to filter mocap data
   */
  void mocapCallBackHelper(const geometry_msgs::PoseStamped::ConstPtr& msg,
                           const Eigen::Vector3d& pos);

  /**
   * @brief perform CF update once
   * @param[out] last_robot_state_msg
   */
  bool updateOnce(quad_msgs::RobotState& last_robot_state_msg);

  bool initiated;

 private:
  /// Struct of second-order low/high pass filter with derivative/intergral
  struct Filter {
    // State-space model
    Eigen::Matrix<double, 2, 2> A;
    Eigen::Matrix<double, 2, 1> B;
    Eigen::Matrix<double, 1, 2> C;
    Eigen::Matrix<double, 1, 1> D;

    // Filter states
    std::vector<Eigen::Matrix<double, 2, 1>> x;

    // Filter initialization indicator
    bool init;
  };

  /// Low pass filter
  Filter low_pass_filter;

  /// High pass filter
  Filter high_pass_filter;

  /// High Pass States
  std::vector<double> high_pass_a_;
  std::vector<double> high_pass_b_;
  std::vector<double> high_pass_c_;
  std::vector<double> high_pass_d_;

  /// Low Pass States
  std::vector<double> low_pass_a_;
  std::vector<double> low_pass_b_;
  std::vector<double> low_pass_c_;
  std::vector<double> low_pass_d_;

  /// Best estimate of velocity
  Eigen::Vector3d vel_estimate_;

  /// Best estimate of velocity from mocap diff
  Eigen::Vector3d mocap_vel_estimate_;

  /// Best estimate of imu velocity
  Eigen::Vector3d imu_vel_estimate_;

  /// Last mocap time
  ros::Time last_mocap_time_;

  /// Nodehandle to get param
  ros::NodeHandle nh_;
};
#endif  // COMP_FILTER_H
