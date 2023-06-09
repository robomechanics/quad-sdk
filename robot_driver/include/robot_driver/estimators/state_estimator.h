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

//#include <local_planner/local_planner.h>

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
  virtual bool updateOnce(quad_msgs::RobotState& last_robot_state_msg) = 0;

  /**
   * @brief Read IMU data
   * @param[in] last_imu_msg IMU sensor message
   * @param[out] fk Linear acceleration
   * @param[out] wk Angular acceleration
   * @param[out] qk Orientation in quaternion
   */
  void readIMU(const sensor_msgs::Imu::ConstPtr& last_imu_msg,
               Eigen::Vector3d& fk, Eigen::Vector3d& wk,
               Eigen::Quaterniond& qk);

  /**
   * @brief Read joint encoder data
   * @param[in] last_joint_state_msg Joint state sensor message
   * @param[out] jk Jointstate in vector (12 * 1)
   */
  void readJointEncoder(
      const sensor_msgs::JointState::ConstPtr& last_joint_state_msg,
      Eigen::VectorXd& jk);

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
