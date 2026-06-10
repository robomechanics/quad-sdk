#ifndef COMP_FILTER_H
#define COMP_FILTER_H

#include <robot_driver/estimators/state_estimator.hpp>

//! Implements complementary filter as an estimator within the ROS framework.
class CompFilterEstimator : public StateEstimator {
 public:
  /**
   * @brief Constructor for CompFilterEstimator
   * @return Constructed object of type CompFilterEstimator
   */
  CompFilterEstimator(rclcpp::Node::SharedPtr node, const std::string& robot_ns,
                      std::shared_ptr<quad_utils::QuadKD2> quadKD);

  /**
   * @brief Initialize Complementary Filter
   * @param[in] nh ROS Node Handler used to load parameters from yaml file
   */
  void init() override;

  /**
   * @brief Helper function to filter mocap data
   */
  void mocapCallBackHelper(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                           const Eigen::Vector3d& pos);

  /**
   * @brief Perform CF update once
   * @param[out] last_robot_state_msg
   */
  bool updateOnce(quad_msgs::msg::RobotState& last_robot_state_msg) override;

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
  rclcpp::Time last_mocap_time_;

  /// Lateral (body-y) drift rejection. The mocap body_y carries a slow,
  /// motion-induced solve error (trunk markers occluded by the swinging legs)
  /// that the controller mistakes for real lateral drift. When the robot is
  /// known to walk straight (y_true ~ const), reject that slow drift so it does
  /// not reach the controller. OFF by default; see updateOnce.
  bool lat_drift_reject_ = false;
  /// Low-pass coefficient (0..1) for tracking the slow drift; larger = holds
  /// body_y harder at the start, smaller = preserves more real lateral motion.
  double lat_drift_tau_ = 0.01;
  /// Captured lateral reference (start) and running slow-drift estimate.
  double y_ref_ = 0.0;
  double y_drift_lp_ = 0.0;
  bool y_ref_init_ = false;
};
#endif  // COMP_FILTER_H
