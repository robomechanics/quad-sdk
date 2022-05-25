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
   * @brief initiate Comp_filter
   */
  void init();

  /**
   * @brief initiate Comp_filter
   * @param[in] high_pass_a
   * @param[in] high_pass_b
   * @param[in] high_pass_c
   * @param[in] high_pass_d
   * @param[in] low_pass_a
   * @param[in] low_pass_b
   * @param[in] low_pass_c
   * @param[in] low_pass_d
   */

  void init(const std::vector<double>& high_pass_a,
            const std::vector<double>& high_pass_b,
            const std::vector<double>& high_pass_c,
            const std::vector<double>& high_pass_d,
            const std::vector<double>& low_pass_a,
            const std::vector<double>& low_pass_b,
            const std::vector<double>& low_pass_c,
            const std::vector<double>& low_pass_d);

  /**
   * @brief helper function to filter mocap data
   */
  void mocapCallBackHelper(const geometry_msgs::PoseStamped::ConstPtr& msg,
                           const Eigen::Vector3d& pos,
                           geometry_msgs::PoseStamped::ConstPtr last_mocap_msg_,
                           const double& mocap_rate_,
                           const double& mocap_dropout_threshold_);

  /**
   * @brief update CF state once
   */
  bool updateState();

  /**
   * @brief update CF state once
   * @param[in] fully_populated
   * @param[in] last_imu_msg_
   * @param[in] last_joint_state_msg_
   * @param[in] last_mocap_msg_
   * @param[in] user_rx_data_
   * @param[out] last_robot_state_msg_
   */

  bool updateState(const bool& fully_populated, sensor_msgs::Imu& last_imu_msg_,
                   sensor_msgs::JointState& last_joint_state_msg_,
                   geometry_msgs::PoseStamped::ConstPtr last_mocap_msg_,
                   quad_msgs::RobotState& last_robot_state_msg_);

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

  Filter low_pass_filter;

  Filter high_pass_filter;

  /// Best estimate of velocity
  Eigen::Vector3d vel_estimate_;

  /// Best estimate of velocity from mocap diff
  Eigen::Vector3d mocap_vel_estimate_;

  /// Best estimate of imu velocity
  Eigen::Vector3d imu_vel_estimate_;

  /// Last mocap time
  ros::Time last_mocap_time_;
};
#endif  // COMP_FILTER_H
