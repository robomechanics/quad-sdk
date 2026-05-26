#ifndef EKF_ESTIMATOR_H
#define EKF_ESTIMATOR_H

#include <robot_driver/estimators/state_estimator.hpp>
#include <quad_msgs/msg/foot_contact.hpp>

#include <eigen3/Eigen/Eigen>
#include <vector>

//! Implements online EKF based state estimation
/*!
   EKFEstimator implements all estimator logic. It should expose a constructor
   that does any initialization required and an update method called at some
   frequency.
*/
class EKFEstimator : public StateEstimator {
 public:
  /**
   * @brief Constructor for EKFEstimator
   * @param[in] node ROS2 node shared pointer
   * @param[in] robot_ns Robot namespace
   * @param[in] quadKD Kinematics/dynamics object
   * @return Constructed object of type EKFEstimator
   */
  EKFEstimator(rclcpp::Node::SharedPtr node, const std::string& robot_ns,
               std::shared_ptr<quad_utils::QuadKD2> quadKD);

  /**
   * @brief Initialize EKF estimator parameters and state
   */
  void init() override;

  /**
   * @brief Perform EKF update once
   * @param[out] last_robot_state_msg Robot state message to update
   * @return true if update was successful
   */
  bool updateOnce(quad_msgs::msg::RobotState& last_robot_state_msg) override;

  /**
   * @brief set X as Xin
   */
  void setX(Eigen::VectorXd Xin);

  /**
   * @brief set P as Pin
   */
  void setP(Eigen::MatrixXd Pin);

  /**
   * @brief return the value of X
   */
  Eigen::VectorXd getX();

  /**
   * @brief return the value of X_pre
   */
  Eigen::VectorXd getX_pre();

  /**
   * @brief execute EKF Update step, return state estimate
   * @return state estimate of custom type RobotState
   */
  quad_msgs::msg::RobotState StepOnce();

  /**
   * @brief EKF prediction step
   * @param[in] dt double time interval
   * @param[in] fk Eigen::VectorXd linear acceleration data (3 * 1)
   * @param[in] wk Eigen::VectorXd angular acceleration data (3 * 1)
   * @param[in] R_w_imu Rotation matrix from IMU to world frame
   */
  void predict(const double& dt, const Eigen::VectorXd& fk,
               const Eigen::VectorXd& wk, const Eigen::Matrix3d R_w_imu);

  /**
   * @brief EKF update step
   * @param[in] jk Eigen::VectorXd joint encoder data (12 * 1)
   * @param[in] fk Eigen::VectorXd linear acceleration data (3 * 1)
   * @param[in] vk Eigen::VectorXd joint encoder velocity data (12 * 1)
   * @param[in] wk Eigen::VectorXd imu angular acceleration data (3 * 1)
   * @param[in] qk Eigen::Quaterniond orientation in quaternion
   * @param[in] R_w_imu Rotation matrix from IMU to world frame
   */
  void update(const Eigen::VectorXd& jk, const Eigen::VectorXd& fk,
              const Eigen::VectorXd& vk, const Eigen::VectorXd& wk,
              const Eigen::Quaterniond& qk, const Eigen::Matrix3d& R_w_imu);

  /**
   * @brief Function to set initial robot state for ekf state estimator
   */
  void setInitialState(quad_msgs::msg::RobotState& last_robot_state_msg);

  Eigen::VectorXd quaternionDynamics(const Eigen::VectorXd& w,
                                     const Eigen::VectorXd& q);

  /**
   * @brief calculate skew-symmetric matrix from a vector
   * @param[in] w Eigen::VectorXd angular velocity vector (3 * 1)
   * @return Eigen::MatrixXd skew-symmetric matrix (3 * 3)
   */
  Eigen::MatrixXd calcSkewsym(const Eigen::VectorXd& w);

  /**
   * @brief calculate rodrigues incremental rotation matrix from a vector
   * @param[in] dt time in second
   * @param[in] w Eigen::VectorXd angular velocity vector (3 * 1)
   * @param[in] sub subscript
   * @return Eigen::MatrixXd rodrigues incremental rotation matrix (3 * 3)
   */
  Eigen::MatrixXd calcRodrigues(const double& dt, const Eigen::VectorXd& w,
                                const int& sub);


  // number of states: position (3) + velocity (3) + feet position (12)
  static const int num_state = 18;

  // number of covariances equals number of states
  static const int num_cov = 18;

  // measurement number: feet positions (12) + feet velocities (12) + feet
  // heights (4)
  static const int num_measure = 28;

  const int num_feet = 4;

 private:
  /// Boolean for whether robot layer is hardware (else sim)
  bool is_hardware_;

  /// Update rate for sending and receiving data
  double update_rate_;

  /// Last grf control message
  quad_msgs::msg::GRFArray::SharedPtr last_grf_msg_;

  /// Last contact detection message
  quad_msgs::msg::ContactMode::SharedPtr last_contact_msg_;

  // gravity vector (3 * 1)
  Eigen::VectorXd g;

  // initial state vector (18 * 1)
  Eigen::VectorXd X0;

  // state vector (18 * 1)
  Eigen::VectorXd X;

  // last state vector (18 * 1)
  Eigen::VectorXd last_X;

  // last covariance matrix
  Eigen::MatrixXd last_P;

  // prediction state vector (18 * 1)
  Eigen::VectorXd X_pre;

  // S * C matrix for Kalman gain computation
  Eigen::Matrix<double, num_measure, num_state> SC;

  // Innovation covariance
  Eigen::Matrix<double, num_measure, num_measure> S;

  // measurement matrix
  Eigen::MatrixXd C;

  // S-weighted error
  Eigen::Matrix<double, num_measure, 1> Serror_y;

  // state covariance matrix
  Eigen::MatrixXd P;

  // prediction state covariance matrix
  Eigen::MatrixXd P_pre;

  // state transition matrix
  Eigen::MatrixXd F;

  // process covariance matrix
  Eigen::MatrixXd Q;

  // compensated body acceleration vector (3 * 1)
  Eigen::Vector3d u;

  // measurement covariance matrix
  Eigen::MatrixXd R;

  // foot contact state vector (4*1), (1 - contact, 0 - in air)
  Eigen::VectorXd foot_contact_states;

  // error measurement displacement vector
  Eigen::Matrix<double, num_measure, 1> error_y;

  // measurement generated from leg kinematics
  Eigen::VectorXd y;

  // previous time variable
  rclcpp::Time last_time_;

  // IMU linear acceleration bias (3*3)
  Eigen::MatrixXd bias_acc;

  // IMU linear acceleration noise (3*3)
  Eigen::MatrixXd noise_acc;

  // IMU angular acceleration bias (3*3)
  Eigen::MatrixXd bias_gyro;

  // IMU angular acceleration noise (3*3)
  Eigen::MatrixXd noise_gyro;

  // individual noise at feet (3*3)
  Eigen::MatrixXd noise_feet;

  // noise at feet (3*3)
  Eigen::MatrixXd noise_fk;

  // noise at encoder
  double noise_encoder;

  // imu bias values
  double bias_x_;
  double bias_y_;
  double bias_z_;
  double bias_r_;
  double bias_p_;
  double bias_w_;

  // noise terms
  double na_;   // noise in accelerometer (imu process position)
  double nv_;   // noise in imu process velocity
  double ng_;   // noise in gyro
  double ba_;   // bias in accelerometer
  double bg_;   // bias in gyro
  double nf_;   // noise in imu process foot position
  double nfk_;  // noise in update encoder forward kinematics
  double ne_;   // noise in update encoder velocity
  double nfh_;  // noise in update measurement foot height
  double P0_;   // initial covariance value
  double contact_w_;   // weight on foot contact value
  double thresh_out;   // Innovation norm threshold for outlier rejection
  double foot_radius;  // Toe radius of each foot

  // Binary flag denoting estimator initialization
  bool initialized = true;

  // Debug flag: only when true does the EKF capture/apply the init yaw offset
  // (used for IMU-vs-mocap testing). Defaults off -> init_yaw_offset_ stays 0
  // and the EKF runs on plain IMU yaw.
  bool debug_ = false;

  // Constant world-yaw datum captured once at init: (seed yaw) - (IMU yaw).
  // Applied to the IMU orientation each step so heading is pinned to the same
  // world frame as the position seed, while roll/pitch stay IMU (gravity-true).
  double init_yaw_offset_ = 0.0;

  // Guard: setInitialState runs twice in updateOnce; only the FIRST call sees
  // the mocap seed orientation, so capture the yaw offset only once. Without
  // this the second call recomputes it from the (already IMU-overwritten)
  // orientation and resets it to 0.
  bool init_yaw_offset_set_ = false;

  std::string robot_name_;

  /// Subscriber for ground_truth RobotState messages
  rclcpp::Subscription<quad_msgs::msg::RobotState>::SharedPtr
      state_ground_truth_sub_;

  /// Subscriber for grf messages
  rclcpp::Subscription<quad_msgs::msg::GRFArray>::SharedPtr grf_sub_;

  /// Subscriber for contact detection messages
  rclcpp::Subscription<quad_msgs::msg::ContactMode>::SharedPtr contact_sub_;

  /// Subscriber for imu messages (sim only)
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  /// Subscriber for joint encoder messages (sim only)
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_encoder_sub_;
};
#endif  // EKF_ESTIMATOR_H
