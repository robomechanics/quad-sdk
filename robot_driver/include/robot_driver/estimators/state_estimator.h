#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <quad_msgs/ContactMode.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/quad_kd.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

//! Implements an abstract class for state estimator.
//! This class provides an interface for different types of estimators
/*!
   StateEstimator provides an abstract state estimator class
*/
class StateEstimator {
 public:
  /**
   * @brief Constructor for StateEstimator
   * @return Constructed object of type StateEstimator
   */
  StateEstimator();

  /**
   * @brief Virtual function for initialize filters, should be defined in
   * derived class
   * @param[in] nh_ ROS Node Ha
   */
  virtual void init(ros::NodeHandle& nh) = 0;

  /**
   * @brief Virtual update function for update robot state, should be defined in
   * derived class
   * @param[out] last_robot_state_msg robot state
   */
  virtual bool updateOnce(quad_msgs::RobotState& last_robot_state_msg, int& control_mode) = 0;

  /**
   * @brief Read IMU data
   * @param[in] last_imu_msg IMU sensor message
   * @param[out] fk Linear acceleration
   * @param[out] wk Angular acceleration
   * @param[out] qk Orientation in quaternion
   */
  void readIMU(const sensor_msgs::Imu& last_imu_msg, Eigen::VectorXd& fk,
               Eigen::VectorXd& wk, Eigen::Quaterniond& qk);

  /**
   * @brief Read joint encoder data
   * @param[in] last_joint_state_msg Joint state sensor message
   * @param[out] jk Joint state in vector (12 * 1)
   * @param[out] vk Joint velocity in vector (12 * 1)
   */
  void readJointEncoder(const sensor_msgs::JointState& last_joint_state_msg,
                        Eigen::VectorXd& jk, Eigen::VectorXd& vk);

  /**
   * @brief Load Mocap data to protected variable
   * @param[in] last_mocap_msg Mocap message
   */
  void loadMocapMsg(geometry_msgs::PoseStamped::ConstPtr last_mocap_msg);

  /**
   * @brief Load imu and joint encoder data to protected variables
   * @param[in] last_imu_msg imu msgs
   * @param[in] last_joint_state_msg joint state msgs
   */
  void loadSensorMsg(sensor_msgs::Imu last_imu_msg,
                     sensor_msgs::JointState last_joint_state_msg);

  void updateLocalPlanMsg(quad_msgs::RobotPlan::ConstPtr msg, int position);

 protected:
  /// Last state estimate
  quad_msgs::RobotState state_est_;

  /// Most recent state estimate (added 02/05)
  quad_msgs::RobotState last_robot_state_msg_;

  /// QuadKD class
  std::shared_ptr<quad_utils::QuadKD> quadKD_;

  /// Last mocap data
  geometry_msgs::PoseStamped::ConstPtr last_mocap_msg_;

  /// Most recent IMU data
  sensor_msgs::Imu last_imu_msg_;

  /// Most recent joint data
  sensor_msgs::JointState last_joint_state_msg_;

  /// Last local plan message
  quad_msgs::RobotPlan::ConstPtr last_local_plan_msg_;

  int control_mode_;
};

#endif  // STATE_ESTIMATOR_H
