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
   * @brief Pure virtual function for filter initialization
   */
  virtual void init() = 0;

  /**
   * @brief Pure virtual function for performing state estimate update
   * 
   */
  virtual quad_msgs::RobotState updateState() = 0;

  /**
   * @brief read IMU data
   * @param[in] last_imu_msg_ sensor_msgs::Imu::ConstPtr imu sensor message
   * @param[out] fk Eigen::VectorXd linear acceleration (3 * 1)
   * @param[out] wk Eigen::VectorXd angular acceleration (3 * 1)
   * @param[out] qk Eigen::Quaterniond orientation in quaternion
   */
  void readIMU(const sensor_msgs::Imu::ConstPtr& last_imu_msg_,
               Eigen::VectorXd& fk, Eigen::VectorXd& wk,
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

  

 protected:
  /// Last state estimate
  quad_msgs::RobotState state_est_;

  /// QuadKD class
  std::shared_ptr<quad_utils::QuadKD> quadKD_;
};

#endif  // LEG_CONTORLLER_H
