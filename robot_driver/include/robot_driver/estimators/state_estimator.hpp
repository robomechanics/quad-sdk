#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <quad_msgs/msg/contact_mode.hpp>
#include <quad_msgs/msg/foot_contact.hpp>
#include <quad_msgs/msg/robot_state.hpp>
#include <quad_utils/quad_kd2.hpp>
#include <quad_utils/ros_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

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
  StateEstimator(rclcpp::Node::SharedPtr node, const std::string& robot_ns,
                 std::shared_ptr<quad_utils::QuadKD2> quadKD);

  /**
   * @brief Virtual function for initialize filters, should be defined in
   * derived class
   * @param[in] nh_ ROS Node Ha
   */
  virtual void init() = 0;

  /**
   * @brief Virtual update function for update robot state, should be defined in
   * derived class
   * @param[out] last_robot_state_msg robot state
   */
  virtual bool updateOnce(quad_msgs::msg::RobotState& last_robot_state_msg) = 0;

  /**
   * @brief Read IMU data
   * @param[in] last_imu_msg IMU sensor message
   * @param[out] fk Linear acceleration
   * @param[out] wk Angular acceleration
   * @param[out] qk Orientation in quaternion
   */
  void readIMU(const sensor_msgs::msg::Imu::SharedPtr& last_imu_msg,
               Eigen::Vector3d& fk, Eigen::Vector3d& wk,
               Eigen::Quaterniond& qk);

  /**
   * @brief Read joint encoder data
   * @param[in] last_joint_state_msg Joint state sensor message
   * @param[out] jk Jointstate in vector (12 * 1)
   */
  void readJointEncoder(
      const sensor_msgs::msg::JointState::SharedPtr& last_joint_state_msg,
      Eigen::VectorXd& jk);

  /**
   * @brief Load Mocap data to protected variable
   * @param[in] last_mocap_msg Mocap message
   */
  void loadMocapMsg(geometry_msgs::msg::PoseStamped::SharedPtr last_mocap_msg);

  /**
   * @brief Load imu and joint encoder data to protected variables
   * @param[in] last_imu_msg imu msgs
   * @param[in] last_joint_state_msg joint state msgs
   */
  void loadSensorMsg(sensor_msgs::msg::Imu last_imu_msg,
                     sensor_msgs::msg::JointState last_joint_state_msg);

  /**
   * @brief Load measured foot-contact data (Unitree foot-force sensor).
   * Pushed in by robot_driver each tick, rather than subscribed, so the
   * estimator stays synchronized with the IMU/joint sample it was given.
   * @param[in] msg Foot-contact message (raw force + thresholded contact)
   */
  void loadFootContactMsg(const quad_msgs::msg::FootContact& msg);

 protected:
  /// Robot Namespace
  std::string robot_ns_;

  /// Shared Pointer to Node
  rclcpp::Node::SharedPtr node_;

  /// Last state estimate
  quad_msgs::msg::RobotState state_est_;

  /// QuadKD class
  std::shared_ptr<quad_utils::QuadKD2> quadKD_;

  /// Last mocap data
  geometry_msgs::msg::PoseStamped::SharedPtr last_mocap_msg_;

  /// Most recent IMU data
  sensor_msgs::msg::Imu last_imu_msg_;

  /// Most recent joint data
  sensor_msgs::msg::JointState last_joint_state_msg_;

  /// Most recent measured foot-contact data (Unitree foot-force sensor)
  quad_msgs::msg::FootContact last_foot_contact_msg_;

  /// Whether a foot-contact message has been received at least once
  bool foot_contact_received_ = false;
};

#endif  // STATE_ESTIMATOR_H
