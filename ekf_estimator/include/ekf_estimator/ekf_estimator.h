#ifndef EKF_ESTIMATOR_H
#define EKF_ESTIMATOR_H

#include <quad_msgs/ContactMode.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/quad_kd.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen3/Eigen/Eigen>
#include <vector>

//! Implements online EKF based state estimation
/*!
   EKFEstimator implements all estimator logic. It should expose a constructor
   that does any initialization required and an update method called at some
   frequency.
*/
class EKFEstimator {
 public:
  /**
   * @brief Constructor for EKFEstimator
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type EKFEstimator
   */
  EKFEstimator(ros::NodeHandle nh);

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

 private:
  /**
   * @brief Callback function to handle new ground truth data
   * @param[in] msg ground_truth_msg quad_msgs<RobotState>
   */
  void groundtruthCallback(const quad_msgs::RobotState::ConstPtr& msg);

  /**
   * @brief Callback function to handle new joint encoder data
   * @param[in] joint_encoder_msg sensor_msgs<JointState> containing joint
   * pos,vel,current
   */
  void jointEncoderCallback(const sensor_msgs::JointState::ConstPtr& msg);

  /**
   * @brief Callback function to handle new imu data
   * @param[in] imu_msg sensors_msgs<Imu> containing new imu data
   */
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief Callback function to handle new contact estimates
   * @param[in] msg quad_msgs::ContactMode containing new contact data
   */
  void contactCallback(const quad_msgs::ContactMode::ConstPtr& msg);

  /**
   * @brief execute EKF Update step, return state estimate
   * @return state estimate of custom type RobotState
   */
  quad_msgs::RobotState updateStep();

  /**
   * @brief predict quaternion k+1 from k using dt, angular velocity, and qk
   * @param[in] w Eigen::VectorXd angular velocity vector (3 * 1)
   * @param[in] q Eigen::VectorXd quaternion k vector (4 * 1)
   * @return Eigen vector of quaternion k+1 vector (4 * 1)
   */
  Eigen::VectorXd quaternionDynamics(const Eigen::VectorXd& w,
                                     const Eigen::VectorXd& q);

  /**
   * @brief calculate skew-symmetric matrix from a vector
   * @param[in] w Eigen::VectorXd angular velocity vector (3 * 1)
   * @return Eigen::MatrixXd  skew- symmetric matrix (3 * 3)
   */
  Eigen::MatrixXd calcSkewsym(const Eigen::VectorXd& w);

  /**
   * @brief calculate rodrigues incremental rotation matrix from a vector
   * @param[in] dt time in second
   * @param[in] w Eigen::VectorXd angular velocity vector (3 * 1)
   * @param[in] sub subscript
   * @return Eigen::MatrixXd rodrigues incremental rotation matrix matrix (3 *
   * 3)
   */
  Eigen::MatrixXd calcRodrigues(const double& dt, const Eigen::VectorXd& w,
                                const int& sub);

  /**
   * @brief set sensor noise
   */
  void setNoise();

  // number of states position (3 * 1) + velocity (3 * 1) + quaternion (4 * 1) +
  // feet position (12 * 1) + bias_acc (3 * 1) + bias_gyro (3 * 1)
  static const int num_state = 28;
  // number of covariances equals number of states
  static const int num_cov = 27;
  // measurement number equals feet positions (12)
  static const int num_measure = 12;
  const int num_feet = 4;

  /// Subscriber for ground_truth RobotState messages
  ros::Subscriber state_ground_truth_sub_;

  /// Last state ground_truth
  quad_msgs::RobotState::ConstPtr last_state_msg_;

  /// Subscriber for joint encoder messages
  ros::Subscriber joint_encoder_sub_;

  /// Subscriber for imu messages
  ros::Subscriber imu_sub_;

  /// Subscriber for contact detection messages
  ros::Subscriber contact_sub_;

  /// Publisher for state estimate messages
  ros::Publisher state_estimate_pub_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Update rate for sending and receiving data;
  double update_rate_;

  /// Last state estimate
  quad_msgs::RobotState last_state_est_;

  /// Last contact detection message (should be timestamped!)
  quad_msgs::ContactMode::ConstPtr last_contact_msg_;

  /// Most recent IMU callback (should be timestamped!)
  sensor_msgs::Imu::ConstPtr last_imu_msg_;

  /// Most recent encoder callback (should be timestamped!)
  sensor_msgs::JointState::ConstPtr last_joint_state_msg_;

  /// Maximum amount of time to still use joint state message in EKF data
  double joint_state_msg_time_diff_max_;

  /// QuadKD class
  std::shared_ptr<quad_utils::QuadKD> quadKD_;

  // gravity vector (3 * 1)
  Eigen::VectorXd g;

  // initial state vector (28 * 1)
  Eigen::VectorXd X0;

  // state vector (28 * 1)
  Eigen::VectorXd X;

  // last state vector (28 * 1)
  Eigen::VectorXd last_X;

  // prediction state vector (28 * 1)
  Eigen::VectorXd X_pre;

  // initial covariance matrix (27 * 27)
  Eigen::MatrixXd P0;

  // state covariance matrix (27 * 27)
  Eigen::MatrixXd P;

  // prediction state covariance matrix (27 * 27)
  Eigen::VectorXd P_pre;

  // state transition matrix (27 * 27)
  Eigen::MatrixXd F;

  // process covariance matrix (27 * 27)
  Eigen::MatrixXd Q;

  // measurement matrix (12 * 27)
  Eigen::MatrixXd H;

  // measurement covariance matrix (12 * 12)
  Eigen::MatrixXd R;

  // previous time variable
  ros::Time last_time;

  // IMU linear acceleration bias (3*3)
  Eigen::MatrixXd bias_acc;

  // IMU linear acceleration noise (3*3)
  Eigen::MatrixXd noise_acc;

  // IMU augular acceleration bias (3*3)
  Eigen::MatrixXd bias_gyro;

  // IMU augular acceleration noise (3*3)
  Eigen::MatrixXd noise_gyro;

  // individual noise at feet (3*3)
  Eigen::MatrixXd noise_feet;

  // noise at feet (3*3)
  Eigen::MatrixXd noise_fk;

  // noise at encoder
  double noise_encoder;

  //   // IMU bias in acc x
  //   double bias_x_;

  //   // IMU bias in acc x
  //   double bias_y_;

  //   // IMU bias in acc z
  //   double bias_z_;

  //   // IMU bias in gyro x
  //   double bias_roll_;

  //   // IMU bias in gyro y
  //   double bias_pitch_;

  //   // IMU bias in gyro z
  //   double bias_yaw_;
};
#endif  // EKF_ESTIMATOR_H
