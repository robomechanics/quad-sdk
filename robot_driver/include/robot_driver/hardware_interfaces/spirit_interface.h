#ifndef SPIRIT_INTERFACE_H
#define SPIRIT_INTERFACE_H

#include <quad_msgs/LegCommandArray.h>
#include <robot_driver/hardware_interfaces/hardware_interface.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen3/Eigen/Eigen>
#include <mblink/mblink.hpp>

using gr::MBLink;

struct LimbCmd_t {
  Eigen::Vector3f pos, vel, tau;
  short kp[3];
  short kd[3];
  bool restart_flag;
};

typedef std::unordered_map<std::string, Eigen::VectorXf> MBData_t;

//! Hardware interface for the Spirit40 quadruped from Ghost Robotics.
/*!
   SpiritInterface listens for joint control messages and outputs low level
   commands to the mainboard over mblink.
*/
class SpiritInterface : public HardwareInterface {
 public:
  /**
   * @brief Constructor for SpiritInterface
   * @return Constructed object of type SpiritInterface
   */
  SpiritInterface();

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
   * @brief Send commands to the robot via the mblink protocol
   * @param[in] leg_command_array_msg Message containing leg commands
   * @param[in] user_data Vector containing user data
   * @return boolean indicating success of transmission
   */
  virtual bool send(const quad_msgs::LegCommandArray& leg_command_array_msg,
                    const Eigen::VectorXd& user_tx_data);

  /**
   * @brief Recieve data from the robot via the mblink protocol
   * @param[out] joint_state_msg Message containing joint state information
   * @param[out] imu_msg Message containing imu information
   * @param[out] user_data Vector containing user data
   * @return Boolean for whether data was successfully received
   */
  virtual bool recv(sensor_msgs::JointState& joint_state_msg,
                    sensor_msgs::Imu& imu_msg, Eigen::VectorXd& user_rx_data);

  /// Pointer to MBLink object
  MBLink mblink_;

  /// Mainboard data
  MBData_t mbdata_;

  /// Vector of joint names
  std::vector<std::string> joint_names_ = {"8",  "0", "1", "9",  "2", "3",
                                           "10", "4", "5", "11", "6", "7"};

  /// Vector denoting joint indices
  std::vector<int> joint_indices_ = {8, 0, 1, 9, 2, 3, 10, 4, 5, 11, 6, 7};

  /// Vector of kt values for each joint
  std::vector<double> kt_vec_ = {0.546, 0.546, 1.092, 0.546, 0.546, 1.092,
                                 0.546, 0.546, 1.092, 0.546, 0.546, 1.092};
};

#endif  // SPIRIT_INTERFACE_H
