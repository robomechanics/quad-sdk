#ifndef YLO2_INTERFACE_H
#define YLO2_INTERFACE_H

#include <quad_msgs/LegCommandArray.h>
#include <robot_driver/hardware_interfaces/hardware_interface.h>
#include "moteus_driver/YloTwoPcanToMoteus.hpp" // ylo2 library
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>

#include <eigen3/Eigen/Eigen>

#define MATH_PI 3.141592

//! Hardware interface for Ylo2 quadruped from Vincent FOUCAULT.
/*!
   It listens for joint control messages and outputs low level
   commands to moteus controllers, via a PEAK M2 4 canfd controller.
*/
class Ylo2Interface : public HardwareInterface {

 public:
  /**
   * @brief Constructor for Ylo2Interface
   * @return Constructed object of type Ylo2Interface
   */
  Ylo2Interface();

  /**
   * @brief Load the hardware interface
   * @param[in] argc Argument count
   * @param[in] argv Argument vector
   */
  virtual void loadInterface(int argc, char** argv);

  /**
   * @brief Unload the hardware interface
   */
  virtual void unloadInterface();

/**
   * @brief Send commands to the robot
   * @param[in] - leg_command_array_msg Message containing leg commands
   * @param[in] - user_data Vector containing user data
   * @return boolean indicating success of transmission
   */
  virtual bool send(const quad_msgs::LegCommandArray& leg_command_array_msg,
                    const Eigen::VectorXd& user_tx_data);

  /**
   * @brief Recieve data from the robot :
   * @param[out] - joint_state_msg Message
   * @param[out] - imu_msg Message
   * @param[out] - user_data Vector containing user data
   * @return Boolean for whether data was successfully received
   */
  virtual bool recv(sensor_msgs::JointState& joint_state_msg,
                    sensor_msgs::Imu& imu_msg, Eigen::VectorXd& user_rx_data);

  /// Vector of joint names
  std::vector<std::string> joint_names_ = {"8",  "0", "1", "9",  "2", "3",
                                           "10", "4", "5", "11", "6", "7"};

  /// Vector denoting joint indices
  std::vector<int> joint_indices_ = {8, 0, 1, 9, 2, 3, 10, 4, 5, 11, 6, 7};

  /// Vector of kt values for each joint
  std::vector<double> kt_vec_ = {0.546, 0.546, 1.092, 0.546, 0.546, 1.092,
                                 0.546, 0.546, 1.092, 0.546, 0.546, 1.092};

}; // end class

#endif  // YLO2_INTERFACE_H