#ifndef NEW_PLATFORM_INTERFACE_H
#define NEW_PLATFORM_INTERFACE_H

#include <quad_msgs/LegCommandArray.h>
#include <robot_driver/hardware_interfaces/hardware_interface.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "robot_driver/hardware_interfaces/CANInterface.hpp"
#include "robot_driver/hardware_interfaces/MotorDriver.hpp"
#include <eigen3/Eigen/Eigen>
#include <thread>

//! Hardware interface for the Spirit40 quadruped from Ghost Robotics.
/*!
   SpiritInterface listens for joint control messages and outputs low level
   commands to the mainboard over mblink.
*/
class NewPlatformInterface : public HardwareInterface 
{
 public:
  /**
   * @brief Constructor for SpiritInterface
   * @return Constructed object of type SpiritInterface
   */
  NewPlatformInterface();

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
  
  /// Vector of joint names
  std::vector<std::string> joint_names_0_ = {"1","5","13","16","4","8"};
  std::vector<std::string> joint_names_1_ = {"3","7","11","2","6","10"};

  

  /// Vector of joint names  Spirit
//   std::vector<std::string> joint_names_ = {"8",  "0", "1", "9",  "2", "3",
//                                            "10", "4", "5", "11", "6", "7"};

  /// Vector denoting joint indices
   std::vector<int> joint_indices_0_ = {0x01, 0x05, 0x0D, 0x10, 0x04, 0x08};
   std::vector<int> joint_indices_1_ = {0x03, 0x07, 0x0B, 0x02, 0x06, 0x0A};
                                    
   //Change IDs for new motor command creation implementation
   std::vector<int> motor_ids_0 = {0x01, 0x05, 0x0D, 0x10, 0x04, 0x08}; //Front legs (L_Ab, L_Hip, L_Knee, R_Ab, R_Hip, R_Knee)
   std::vector<int> motor_ids_1 = {0x03, 0x07, 0x0B, 0x02, 0x06, 0x0A}; //Rear legs (L_Ab, L_Hip, L_Knee, R_Ab, R_Hip, R_Knee)

   std::vector<int> motor_dirs_0 = {1, 1, 1, 1, -1, -1};
   std::vector<int> motor_dirs_1 = {1, 1, 1, 1, -1, -1};

  /// Vector of joint names  Spirit
//   std::vector<std::string> joint_names_ = {"8",  "0", "1", "9",  "2", "3",
//                                            "10", "4", "5", "11", "6", "7"};

// //   / Vector of joint names
//   std::vector<std::string> joint_names_1_ = {"3", "7", "11", "2", "6", "10"};
//   std::vector<std::string> joint_names_0_ = {"1", "5", "13", "16", "4", "8"};


//   /// Vector denoting joint indices
//    std::vector<int> joint_indices_1_ = {0x03,0x07,0x0B,0x02,0x06,0x0A};
//    std::vector<int> joint_indices_0_ = {0x01,0x05,0x0D,0x10,0x04,0x08};
                                    
//    //Change IDs for new motor command creation implementation
//    std::vector<int> motor_ids_1 = {0x03, 0x07, 0x0B,0x02, 0x06, 0x0A}; //Front legs (L_Ab, L_Hip, L_Knee, R_Ab, R_Hip, R_Knee)
//    std::vector<int> motor_ids_0 = {0x01, 0x05, 0x0D,0x10, 0x04, 0x08}; //Rear legs (L_Ab, L_Hip, L_Knee, R_Ab, R_Hip, R_Knee)


  //Motor controller objects
  motor_driver::MotorDriver motor_controller0 = motor_driver::MotorDriver(motor_ids_0, "can0", motor_driver::MotorType::AK80_6_V2);;
  motor_driver::MotorDriver motor_controller1 = motor_driver::MotorDriver(motor_ids_1, "can1", motor_driver::MotorType::AK80_6_V2);;

  //Motor state vector
  std::map<int, motor_driver::motorState> motorStates_0;
  std::map<int, motor_driver::motorState> motorStates_1;

  /// Vector of kt values for each joint
  std::vector<double> kt_vec_ = {0.546, 0.546, 1.092, 0.546, 0.546, 1.092,
                                 0.546, 0.546, 1.092, 0.546, 0.546, 1.092};
};

#endif  // NEW_PLATFORM_INTERFACE_H
