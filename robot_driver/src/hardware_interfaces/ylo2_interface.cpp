/*
 * Copyright (C) 2023 Vincent FOUCAULT
 * Author: Vincent FOUCAULT
 * email:  elpimous12@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include "robot_driver/hardware_interfaces/ylo2_interface.h"

Ylo2Interface::Ylo2Interface() {
    std::cout<<("[ DEBUG ] Starting Ylo2Interface")<< std::endl;
}

void Ylo2Interface::loadInterface(int argc, char** argv) {
    std::cout<<("[ DEBUG ] Loading Ylo2Interface")<< std::endl;
    Ylo2Interface::startup_routine();
}

void Ylo2Interface::unloadInterface(){
    std::cout<<("[ DEBUG ] UnLoading Ylo2Interface")<< std::endl;
}

/*
 Legs are numbered such that :
 0 = front left, 1 = back left, 2 = front right, and 3 = back right.
in order abad, hip, knee
The joint arrays in spirit_msgs::RobotState are defined such that indices :
 0 = abad0, 1 = hip0, 2 = knee0, 
 3 = abad1, 4 = hip1, 5 = knee1, 
 6 = abad2, 7 = hip2, 8 = knee2, 
 9 = abad3, 10 = hip3, 11 = knee3.  
 
 so, state_msg.joints would be :

  - front_left_abad, front_left_hip, front_left_knee, 
  - back_left_abad, back_left_hip, back_left_knee, 
  - front_right_abad, front_right_hip, front_right_knee, 
  - back_right_abad, back_right_hip, back_right_knee

 ex : calling state_msg.joints.positions[6] ->> front right ABAD

 
*/

/******************************************************************************************
                           ABAD      ---        HIP       ---       KNEE
                       pos vel tor kp kd ...
   moteus_command = { {{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}}, --- FL
                      {{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}}, --- BL
                      {{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}}, --- FR
                      {{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}}, --- BR
                    }
*******************************************************************************************/

YloTwoPcanToMoteus command; // instance of class YloTwoPcanToMoteus

bool Ylo2Interface::startup_routine()
{
  //command.peak_fdcan_board_initialization();
  usleep(200);
  //command.check_initial_ground_pose();
  std::cout << "startup_routine Done." << std::endl;
  usleep(200);
  return true;
}

bool Ylo2Interface::send(const quad_msgs::LegCommandArray& last_leg_command_array_msg, const Eigen::VectorXd& user_tx_data) 
    {
    bool controllers_security_flag = 1; // TODO in case of problem, the flag goes to 0, and so, all commands are 0 ! No torque...

    for (int i = 0; i < 4; ++i) {  // For each leg
      // std::cout << "leg = " << i << std::endl;
      quad_msgs::LegCommand leg_command;
      leg_command = last_leg_command_array_msg.leg_commands.at(i);

      for (int j = 0; j < 3; ++j) {  // For each joint
        // std::cout << "joint = " << j << std::endl;

        /* Envoi de mes commandes via le controller moteus
        xxx = leg_command_heartbeat * leg_command.motor_commands.at(j).pos_setpoint;
        xxx = leg_command_heartbeat * leg_command.motor_commands.at(j).vel_setpoint;
        xxx = leg_command_heartbeat * leg_command.motor_commands.at(j).torque_ff;
        xxx = static_cast<short>( leg_command_heartbeat * leg_command.motor_commands.at(j).kp);
        xxx = static_cast<short>( leg_command_heartbeat * leg_command.motor_commands.at(j).kd);
        */
      }
    }
    return true;
}


bool Ylo2Interface::recv(sensor_msgs::JointState& joint_state_msg, sensor_msgs::Imu& imu_msg, Eigen::VectorXd& user_rx_data) 
    {
    std::cout << "RECEIVE FUNCTION" << std::endl;

    /**
    * @brief Read IMU data
    * @param[in] last_imu_msg IMU sensor message
    * @param[out] Linear acceleration
    * @param[out] Angular acceleration
    * @param[out] Orientation in quaternion
    */

    imu_msg.linear_acceleration.x = 0.000000001;
    imu_msg.linear_acceleration.y = 0.000000001;
    imu_msg.linear_acceleration.z = 0.000000001;
        
    imu_msg.angular_velocity.x = 0.000000001;
    imu_msg.angular_velocity.y = 0.000000001;
    imu_msg.angular_velocity.z = 0.000000001;

    imu_msg.orientation.x = 0.00000001;
    imu_msg.orientation.y = 0.00000001;
    imu_msg.orientation.z = 0.00000001;
    imu_msg.orientation.w = 0.00000001;

    return true;
    }