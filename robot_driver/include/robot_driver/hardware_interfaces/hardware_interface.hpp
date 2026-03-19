#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include <tf2_eigen/tf2_eigen.hpp>
#include <quad_msgs/msg/grf_array.hpp>
#include <quad_msgs/msg/leg_command.hpp>
#include <quad_msgs/msg/leg_command_array.hpp>
#include <quad_msgs/msg/motor_command.hpp>
#include <quad_msgs/msg/multi_foot_plan_continuous.hpp>
#include <quad_msgs/msg/robot_plan.hpp>
#include <quad_msgs/msg/robot_state.hpp>
#include <quad_utils/math_utils.hpp>
#include <quad_utils/ros_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <cmath>
#include <eigen3/Eigen/Eigen>
#define MATH_PI 3.141592

//! Implements an abstract class for robot hardware interfaces.
/*!
   HardwareInterface provides an abstract robot hardware interface class. The
   virtual functions declared here must be implemented by the derived class.
*/
class HardwareInterface {
 public:
  /**
   * @brief Constructor for HardwareInterface
   * @return Constructed object of type HardwareInterface
   */
  HardwareInterface();

  /**
   * @brief Load the hardware interface
   * @param[in] argc Argument count
   * @param[in] argv Argument vector
   */
  virtual void loadInterface(int argc, char** argv) = 0;

  /**
   * @brief Unload the hardware interface
   */
  virtual void unloadInterface() = 0;

  /**
   * @brief Send commands to the robot
   * @param[in] leg_command_array_msg Message containing leg commands
   * @param[in] user_data Vector containing user data
   * @return boolean indicating success of transmission
   */
  virtual bool send(
      const quad_msgs::msg::LegCommandArray& leg_command_array_msg,
      const Eigen::VectorXd& user_tx_data) = 0;

  /**
   * @brief Recieve data from the robot
   * @param[out] joint_state_msg Message containing joint state information
   * @param[out] imu_msg Message containing imu information
   * @param[out] user_data Vector containing user data
   * @return Boolean for whether data was successfully received
   */
  virtual bool recv(sensor_msgs::msg::JointState& joint_state_msg,
                    sensor_msgs::msg::Imu& imu_msg,
                    Eigen::VectorXd& user_rx_data) = 0;

 protected:
};

#endif  // HARDWARE_INTERFACE_H
