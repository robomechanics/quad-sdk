#ifndef EKF_H
#define EKF_H

#include <robot_driver/estimators/state_estimator.h>



// defining constants and variables that will be used to calculate EKF process

#define STATE_SIZE 18 // Readings from IMU
#define MEAS_SIZE 28  // Readings from FK
#define PROCESS_NOISE_PIMU 0.01
#define PROCESS_NOISE_VIMU 0.01
#define PROCESS_NOISE_PFOOT 0.01
#define SENSOR_NOISE_PIMU_REL_FOOT 0.001
#define SENSOR_NOISE_VIMU_REL_FOOT 0.1
#define SENSOR_NOISE_ZFOOT 0.001
#define NUM_LEG 4 // Including this definition here unless better suited at another place



//! Implements Extended Kalman Filter as an estimator within the ROS framework.
// Assume orientation from IMU is known and trusted

class EKFEstimator : public StateEstimator {
 public:
  /**
   * @brief Constructor for EKFEstimator
   * @return Constructed object of type EKFEstimator
   */

  EKFEstimator();

  /**
   * @brief Constructor for EKFEstimator with flat ground parameter
   * @return Constructed object of type EKFEstimator based on flat ground parameter
   */
  //EKFEstimator(bool assume_flat_ground_);

  

  /**
   * @brief Initialize EKF
   * @param[in] nh Node Handler to load parameters from yaml file
   */
  void init(ros::NodeHandle& nh);

  /**
   * @brief Perform EKF update once
   * @param[out] last_robot_state_msg_
   */
  bool updateOnce(quad_msgs::RobotState& last_robot_state_msg_);


  

 private:
  /// Nodehandle to get param
  ros::NodeHandle nh_;

  // Parameters and helping matrices for ekf implementation added to quad-sdk
  // From Shuo Yang implementation

  bool filter_initialized = false;
  // state
  // 0 1 2 pos 3 4 5 vel 6 7 8 foot pos FL 9 10 11 foot pos FR 12 13 14 foot pos RL 15 16 17 foot pos RR
  Eigen::Matrix<double, STATE_SIZE, 1> x; // estimation state
  Eigen::Matrix<double, STATE_SIZE, 1> xbar; // estimation state after process update
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P; // estimation state covariance
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar; // estimation state covariance after process update
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A; // estimation state transition
  Eigen::Matrix<double, STATE_SIZE, 3> B; // estimation state transition
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q; // estimation state transition noise

  // observation
  // 0 1 2   FL pos residual
  // 3 4 5   FR pos residual
  // 6 7 8   RL pos residual
  // 9 10 11 RR pos residual
  // 12 13 14 vel residual from FL
  // 15 16 17 vel residual from FR
  // 18 19 20 vel residual from RL
  // 21 22 23 vel residual from RR
  // 24 25 26 27 foot height
  Eigen::Matrix<double, MEAS_SIZE, 1> y; //  observation
  Eigen::Matrix<double, MEAS_SIZE, 1> yhat; // estimated observation
  Eigen::Matrix<double, MEAS_SIZE, 1> error_y; // estimated observation
  Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y; // S^-1*error_y
  Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> C; // estimation state observation
  Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SC; // S^-1*C
  Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R; // estimation state observation noise
  // helper matrices
  Eigen::Matrix<double, 3, 3> eye3; // 3x3 identity
  Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S; // Innovation (or pre-fit residual) covariance
  Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> K; // kalman gain

  
  

  bool assume_flat_ground = false;

  // Variables to read data from IMU
  Eigen::Vector3d fk_; 
  Eigen::Vector3d wk_;
  Eigen::Quaterniond qk_;

  Eigen::Matrix3d rot_ekf_;
  Eigen::Vector3d rpy_ekf;

  // Variables to retrieve information from topics
  Eigen::Vector3d single_joint_state;
  Eigen::Vector3d foot_pos;

  // Variables for varios data points
  sensor_msgs::Imu::ConstPtr last_imu_msg_;

  /// Most recent joint data
  sensor_msgs::JointState::ConstPtr last_joint_state_msg_;

  // Vector for contact force tracking per leg: 1 = in contact , 0 = no contact
  double estimated_contacts[4];
  //std_msgs::UInt8::ConstPtr msg;

};



//Eigen::VectorXd rpy_;
//Eigen::Matrix3d rot_;


#endif  // EKF_H
