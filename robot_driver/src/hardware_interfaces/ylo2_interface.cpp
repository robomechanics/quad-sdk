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
  command.peak_fdcan_board_initialization();
  usleep(200);
  command.check_initial_ground_pose();
  std::cout << "startup_routine Done." << std::endl;
  usleep(200);
  return true;
}


bool Ylo2Interface::send(const quad_msgs::LegCommandArray& last_leg_command_array_msg, const Eigen::VectorXd& user_tx_data)
{
    std::cout << "SEND FUNCTION" << std::endl;
    bool controllers_security_flag = 1; // TODO in case of problem, the flag goes to 0, and so, all commands are 0 ! No torque...
    bool restart_flag = (user_tx_data[0] == 1); //TODO remove user_tx_data

    /** SEND COMMANDS TO MOTEUS CONTROLLERS JOINTS.
    * @brief Write moteus controllers datas :
    * @param[out] joint_position
    * @param[out] joint_velocity
    * @param[out] joint_fftorque
    * @param[out] joint_kp
    * @param[out] joint_kd */

    for (unsigned int jj=0; jj<12; ++jj) // for all 12 joints,
    {
      std::cout << "joint = " << jj << std::endl;
      auto ids  = command.motor_adapters_[jj].getIdx(); // moteus controller id
      int port  = command.motor_adapters_[jj].getPort(); // select correct port on Peak canfd board
      auto sign = command.motor_adapters_[jj].getSign(); // in case of joint reverse rotation
      auto leg_index = command.motor_adapters_[jj].getLeg_index(); // for vector position feed
      auto joint_index = command.motor_adapters_[jj].getJoint_index(); // for vector position feed

      joint_position = controllers_security_flag * last_leg_command_array_msg.leg_commands[leg_index].motor_commands[joint_index].pos_setpoint;
      joint_velocity = controllers_security_flag * last_leg_command_array_msg.leg_commands[leg_index].motor_commands[joint_index].vel_setpoint;
      joint_fftorque = controllers_security_flag * last_leg_command_array_msg.leg_commands[leg_index].motor_commands[joint_index].torque_ff;
      joint_kp       = static_cast<short>( last_leg_command_array_msg.leg_commands[leg_index].motor_commands[joint_index].kp);
      joint_kd       = static_cast<short>( last_leg_command_array_msg.leg_commands[leg_index].motor_commands[joint_index].kd);

      command.send_moteus_TX_frame(ids, port, joint_position, joint_velocity, joint_fftorque, joint_kp, joint_kd); 
      usleep(120);
    }
    return true;
}


bool Ylo2Interface::recv(sensor_msgs::JointState& joint_state_msg, sensor_msgs::Imu& imu_msg, Eigen::VectorXd& user_rx_data) 
{
    std::cout << "RECEIVE FUNCTION" << std::endl;

    /** RECEIVE JOINTS VALUES.
    * @brief Read moteus controllers datas :
    * @param[out] joint position
    * @param[out] joint velocity
    * @param[out] joint fftorque */

    for (unsigned int jj=0; jj<12; ++jj)
    {
      // Reset values
      float RX_pos = 0.0;
      float RX_vel = 0.0;
      float RX_tor = 0.0;
      float RX_volt = 0.0;
      float RX_temp = 0.0;
      float RX_fault = 0.0;
      auto ids  = command.motor_adapters_[jj].getIdx();
      int port  = command.motor_adapters_[jj].getPort();
      auto sign = command.motor_adapters_[jj].getSign();

      command.read_moteus_RX_queue(ids, port, RX_pos, RX_vel, RX_tor, RX_volt, RX_temp, RX_fault);  // query values;
      usleep(10);

      joint_state_msg.name[jj]     = joint_names_[jj];
      joint_state_msg.position[jj] = static_cast<double>(sign*(RX_pos*2*M_PI)); // joint turns to radians
      joint_state_msg.velocity[jj] = static_cast<double>(RX_vel);   // measured in revolutions / s
      joint_state_msg.effort[jj]   = static_cast<double>(RX_tor);   // measured in N*m
      usleep(200);
    }

    /** RECEIVE IMU VALUES.
    * @brief Read IMU data
    * @param[in] last_imu_msg IMU sensor message
    * @param[out] Linear acceleration
    * @param[out] Angular acceleration
    * @param[out] Orientation in quaternion */
/*  
    @dOtslash
    Thanks for helping me.

    here, i should receive the imu/data topic content, in format sensor_msgs/Imu (as imuData, for example)
    and i should feed with anything like : 
    imu_msg.linear_acceleration.x = imuData.linear_acceleration.x;
    real imu node is quad_imu.
    imu_msg is needed for the robot controller.
    thanks @dOtslash

    for test
    --------
    imu_msg.linear_acceleration.x = 0.000000001;
    imu_msg.linear_acceleration.y = 0.000000001;
    imu_msg.linear_acceleration.z = 0.000000001;
    imu_msg.angular_velocity.x = 0.000000001;
    imu_msg.angular_velocity.y = 0.000000001;
    imu_msg.angular_velocity.z = 0.000000001;
    imu_msg.orientation.x = -0.0034954973907215144;
    imu_msg.orientation.y = 0.004400924119108486;
    imu_msg.orientation.z = -0.3908544437993119;
    imu_msg.orientation.w = 0.9204353410898671;
*/
    return true;
}