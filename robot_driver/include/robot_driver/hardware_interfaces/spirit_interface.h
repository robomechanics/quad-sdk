#ifndef SPIRIT_INTERFACE_H
#define SPIRIT_INTERFACE_H

#pragma once
#include <quad_msgs/LegCommandArray.h>
#include <robot_driver/hardware_interfaces/hardware_interface.h>
#include "moteus_driver/YloTwoPcanToMoteus.hpp" // ylo2 library
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Eigen>
#include <thread> // to call imu subscriber loop under a thread

#define MATH_PI 3.141592

class SpiritInterface : public HardwareInterface {

   private:
   
      /* send a canFD MOVE frame command to send power to motors at very low fftorque, 
      this avoids shocks at moves start, and query informations about moteus controller*/
      bool booting_motors();

      /** @brief Sends a zero command to the robot */
      bool send_zero_command();

      /** @brief Executes the robot's startup routine */
      bool startup_routine();

      // 2*PI
      const float TWO_M_PI = 6.28318531;

      // flag to authorize walk
      bool is_up_flag = false;

      //  robot position is sitted
      std::vector<float>sit_down_joints_pose = { -0.0461273193359375, -0.1825408935546875,  0.371002197265625,  // 3, 1, 2
                                            0.04315185546875,    0.18280029296875,   -0.373260498046875,  // 6, 4, 5
                                            0.0385894775390625, -0.1865081787109375,  0.371063232421875,  // 9, 7, 8
                                           -0.04425048828125,    0.188568115234375,  -0.372222900390625}; // 12, 10, 11

   public:

      SpiritInterface();

      virtual void loadInterface(int argc, char** argv) override;

      virtual void unloadInterface() override;

      virtual bool send(const quad_msgs::LegCommandArray& leg_command_array_msg,
                    const Eigen::VectorXd& user_tx_data) override;

      virtual bool recv(sensor_msgs::JointState& joint_state_msg,
                    sensor_msgs::Imu& imu_msg, Eigen::VectorXd& user_rx_data) override;
  
      /// Vector of joint names
      std::vector<std::string> joint_names_ = {"8",  "0", "1", "9",  "2", "3",
                                           "10", "4", "5", "11", "6", "7"};

      /// Vector denoting joint indices
      std::vector<int> joint_indices_ = {8, 0, 1, 9, 2, 3, 10, 4, 5, 11, 6, 7};

      /// Vector of kt values for each joint
      std::vector<double> kt_vec_ = {0.546, 0.546, 1.092, 0.546, 0.546, 1.092,
                                 0.546, 0.546, 1.092, 0.546, 0.546, 1.092};

      float joint_position = 0.0;
      float joint_velocity = 0.0;
      float joint_fftorque = 0.0;
      float joint_kp       = 0.0;
      float joint_kd       = 0.0;

}; // end class

#endif  // SPIRIT_INTERFACE_H