#ifndef EKF_ESTIMATOR_H
#define EKF_ESTIMATOR_H

#include <gtest/gtest.h>
#include <quad_msgs/ContactMode.h>
#include <quad_msgs/GRFArray.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/quad_kd.h>
#include <quad_utils/ros_utils.h>
#include <robot_driver/estimators/state_estimator.h>
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
class EKFEstimator : public StateEstimator {
 public:
  /**
   * @brief Constructor for EKFEstimator
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type EKFEstimator
   */
  EKFEstimator();

  void init(ros::NodeHandle& nh);
  bool updateOnce(quad_msgs::RobotState& last_robot_state_msg_);

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  // void spin(); Commented Out, Doesn't Exist, Spins with Robot Driver

  /**
   * @brief set X as Xin
   */
  void setX(Eigen::VectorXd Xin);

  /**
   * @brief set P as Pin
   */
  void setP(Eigen::MatrixXd Pin);
  /**
   * @brief set lastX as Xin
   */
  void setlast_X(Eigen::VectorXd Xin);

  /**
   * @brief return the value of X
   * @return return X
   */
  Eigen::VectorXd getX();

  /**
   * @brief return the value of last_X
   * @return return last_X
   */
  Eigen::VectorXd getlastX();

  /**
   * @brief return the value of X_pre
   * @return return X_pre
   */
  Eigen::VectorXd getX_pre();

  /**
   * @brief return the value of X
   * @return return X
   */
  Eigen::VectorXd getStates();

  //  private:
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
   * @brief Callback function to handle new grf controls
   * @param[in] msg quad_msgs::GRFArray containing new grf control data
   */
  void grfCallback(const quad_msgs::GRFArray::ConstPtr& msg);

  /**
   * @brief execute EKF Update step, return state estimate
   * @return state estimate of custom type RobotState
   */

  //   void localPlanCallback(const quad_msgs::RobotPlan::ConstPtr& msg);
  quad_msgs::RobotState StepOnce();

  /**
   * @brief EKF prediction step
   * @param[in] dt double time interval
   * @param[in] fk Eigen::VectorXd linear acceleration data (3 * 1)
   * @param[in] wk Eigen::VectorXd angular acceleration data (3 * 1)
   * @param[in] qk Eigen::Quaterniond orientation in quaternion (4 * 1)
   */
  void predict(const double& dt, const Eigen::VectorXd& fk,
               const Eigen::VectorXd& wk, const Eigen::Matrix3d R_w_imu);

  /**
   * @brief EKF update step
   * @param[in] jk Eigen::VectorXd joint encoder data (12 * 1)
   * @param[in] vk Eigen::VectorXd joint encoder velcoity data (12 * 1)
   * @param[in] wk Eigen::VectorXd imu angular acceleration data (3 * 1)
   */
  void update(const Eigen::VectorXd& jk, const Eigen::VectorXd& fk,
              const Eigen::VectorXd& vk, const Eigen::VectorXd& wk,
              const Eigen::Matrix3d R_w_imu);

  /**
   * @brief Function to set initial robot state for ekf state estimator
   */
  void setInitialState(quad_msgs::RobotState& last_robot_state_msg_);

  // void updateAngVel(const double& dt, Eigen::Quaterniond& q, 
  //                   Eigen::Vector3d& last_rpy, const Eigen::VectorXd& wk, Eigen::Vector3d& ang_vel);

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
  static const int num_state = 18;

  // number of covariances equals number of states
  static const int num_cov = 18;

  // measurement number equals feet positions (12)
  static const int num_measure = 28;

  const int num_feet = 4;

  /// Boolean for whether robot layer is hardware (else sim)
  bool is_hardware_;

  /// Subscriber for ground_truth RobotState messages
  ros::Subscriber state_ground_truth_sub_;

  /// Subscriber for joint encoder messages
  ros::Subscriber joint_encoder_sub_;

  /// Subscriber for imu messages
  ros::Subscriber imu_sub_;

  /// Subscriber for grf messages
  ros::Subscriber grf_sub_;

  /// Subscriber for contact detection messages
  ros::Subscriber contact_sub_;

  /// ROS subscriber for local plan
  ros::Subscriber local_plan_sub_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Update rate for sending and receiving data;
  double update_rate_;

  /// Last state estimate
  quad_msgs::RobotState last_robot_ground_truth_;
  /// Most recent local plan
  //   quad_msgs::RobotPlan::ConstPtr last_local_plan_msg_;

  /// Last grf control message
  quad_msgs::GRFArray::ConstPtr last_grf_msg_;

  /// Last contact detection message (should be timestamped!)
  quad_msgs::ContactMode::ConstPtr last_contact_msg_;

  /// Most recent IMU callback (should be timestamped!)
  //   sensor_msgs::Imu::ConstPtr last_imu_msg_;

  /// Most recent encoder callback (should be timestamped!)
  //   sensor_msgs::JointState::ConstPtr last_joint_state_msg_;

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

  // prediction state vector (28 * 1)
  Eigen::Matrix<double, num_measure, num_state> SC;

  Eigen::Matrix<double, num_measure, num_measure> S;

  // rotation matrix generated from imu quaternion (3 * 3)
  Eigen::Matrix3d C1;

  // measurement matrix
  Eigen::MatrixXd C;

  // initial covariance matrix (27 * 27)
  Eigen::Matrix<double, num_measure, 1> Serror_y;

  // state covariance matrix (27 * 27)
  Eigen::MatrixXd P;

  // prediction state covariance matrix (27 * 27)
  Eigen::MatrixXd P_pre;

  // state transition matrix (27 * 27)
  Eigen::MatrixXd F;

  // process covariance matrix (27 * 27)
  Eigen::MatrixXd Q;

  // "control" compensated body acceletation vector (3 * 1)
  Eigen::Vector3d u;

  // measurement matrix (12 * 27)
  Eigen::MatrixXd H;

  // measurement covariance matrix (12 * 12)
  Eigen::MatrixXd R;

  // foot forward kinematics vector (24*1)
  Eigen::VectorXd foot_state;

  // error measurement displacement vector (18 * 1)
  Eigen::Matrix<double, num_measure, 1> error_y;

  // measurement Generated from Leg Kinematics (18 * 1)
  Eigen::VectorXd y;

  Eigen::VectorXd q;

  Eigen::Vector3d last_rpy;

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

  // imu bias
  double bias_x_;

  double bias_y_;

  double bias_z_;

  double bias_r_;

  double bias_p_;

  double bias_w_;

  int counter = 0;

  // noise term
  // noise in accelerometer
  double na_;
  // noise in gyro
  double ng_;
  // bias in accelerometer
  double ba_;
  // bias in gyro
  double bg_;
  // noise in feet
  double nf_;
  // noise in forward kinematics
  double nfk_;
  // noise in encoder
  double ne_;
  // initial covariance value
  double P0_;
  // weight on foot contact value
  double contact_w_;
  // Innovation Norm Threshold, Reject Measurement Update if too large
  double thresh_out;
  // initialized the estimator
  bool initialized = true;
  bool planning = false;
  bool debug = true;

  std::string robot_name_;

  // Record whether we have good imu and joint state data
  bool good_imu = false;
  bool good_joint_state = false;
  bool good_ground_truth_state = false;
};
#endif  // EKF_ESTIMATOR_H
