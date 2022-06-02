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

#include "robot_driver/hardware_interfaces/hardware_interface.h"
#include "robot_driver/hardware_interfaces/spirit_interface.h"

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

  virtual void init(ros::NodeHandle nh) = 0;

  virtual bool updateOnce(quad_msgs::RobotState& last_robot_state_msg) = 0;
  /**
   * @brief read IMU data
   * @param[in] last_imu_msg_ sensor_msgs::Imu::ConstPtr imu sensor message
   * @param[out] fk Eigen::Vector3d linear acceleration (3 * 1)
   * @param[out] wk Eigen::Vector3d angular acceleration (3 * 1)
   * @param[out] qk Eigen::Quaterniond orientation in quaternion
   */
  void readIMU(const sensor_msgs::Imu::ConstPtr& last_imu_msg_,
               Eigen::Vector3d& fk, Eigen::Vector3d& wk,
               Eigen::Quaterniond& qk);

  /**
   * @brief read joint encoder data
   * @param[in] last_joint_state_msg_ sensor_msgs::JointState::ConstPtr joint
   * state sensor message
   * @param[out] jk Eigen::VectorXd jointstate (12 * 1)
   */
  void readJointEncoder(
      const sensor_msgs::JointState::ConstPtr& last_joint_state_msg_,
      Eigen::VectorXd& jk);

  /**
   * @brief load Mocap data to protected variable
   * @param[in] last_mocap_msg geometry_msgs::PoseStamped::ConstPtr
   */
  void loadMocapMsg(geometry_msgs::PoseStamped::ConstPtr last_mocap_msg);

  /**
   * @brief load imu and joint encoder data to protected variables
   * @param[in] last_imu_msg sensor_msgs::Imu
   * @param[in] last_joint_state_msg sensor_msgs::JointState
   */
  void loadSensorMsg(sensor_msgs::Imu last_imu_msg,
                     sensor_msgs::JointState last_joint_state_msg);

 protected:
  /// Last state estimate
  quad_msgs::RobotState state_est_;

  /// QuadKD class
  std::shared_ptr<quad_utils::QuadKD> quadKD_;

  /// Last mocap data
  geometry_msgs::PoseStamped::ConstPtr last_mocap_msg_;

  /// Most recent IMU data
  sensor_msgs::Imu last_imu_msg_;

  /// Most recent joint data
  sensor_msgs::JointState last_joint_state_msg_;
};

#endif  // STATE_ESTIMATOR_H
