#ifndef EKF_ESTIMATOR_H
#define EKF_ESTIMATOR_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <quad_msgs/RobotState.h>
#include <quad_msgs/ContactMode.h>

//! Implements online EKF based state estimation 
/*!
   EKFEstimator implements all estimator logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
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
   * @param[in] ground_truth_msg quad_msgs<RobotState>
   */
  void groundtruthCallback(const quad_msgs::RobotState::ConstPtr& msg);

  /**
   * @brief Callback function to handle new joint encoder data
   * @param[in] joint_encoder_msg sensor_msgs<JointState> containing joint pos,vel,current
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


  /// Subscriber for joint encoder messages
  ros::Subscriber state_ground_truth_pub_;

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

  // state vector
  Eigen::VectorXd x;

  // old state vector
  Eigen::VectorXd last_x;

  // old time
  ros::Time last_time;

  // IMU bias vector
  Eigen::VectorXd imu_bias;

  // IMU bias in acc x
  double bias_x_;

  // IMU bias in acc x
  double bias_y_;

  // IMU bias in acc z
  double bias_z_;

};
#endif // EKF_ESTIMATOR_H
