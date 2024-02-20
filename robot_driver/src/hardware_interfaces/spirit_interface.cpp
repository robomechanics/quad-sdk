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

#include "robot_driver/hardware_interfaces/spirit_interface.h"

SpiritInterface::SpiritInterface(){}

YloTwoPcanToMoteus command; // instance of class YloTwoPcanToMoteus

void startImuThread(YloTwoPcanToMoteus* obj) {
    obj->subscribeToImuData();
}

std::thread imuThread(startImuThread, &command); // create thread and run imu subscriber

void SpiritInterface::loadInterface(int argc, char** argv) {
    startup_routine();
    usleep(10000);
    booting_motors();
    usleep(200);
}

void SpiritInterface::unloadInterface(){
    imuThread.join(); // free imu thread before exit
    ROS_INFO("--RELAXING MOTORS AND QUIT--------> OK--");
    usleep(2000);
    command.stop_motors();
    ROS_INFO("--ALL MOTORS RELAXED--------------> OK--");
    usleep(5000);
}

bool SpiritInterface::booting_motors(){
    if (is_up_flag == false){
        for (unsigned int j = 0; j<2; ++j){
            command.security_switch();
            for (size_t i = 0; i < 12; ++i) {
                auto ids  = command.motor_adapters_[i].getIdx();
                int port  = command.motor_adapters_[i].getPort();
                auto joint_position = NAN;
                auto joint_velocity  = command._comm_velocity;
                auto joint_fftorque  = 0.01;
                auto joint_kp        = command._comm_kp;
                auto joint_kd        = command._comm_kd;
                auto target_vel      = 1.0;
                auto target_accel    = 0.8;
                command.send_to_moteus(ids, port, joint_position, joint_velocity, joint_fftorque, joint_kp, joint_kd, target_vel, target_accel);
                usleep(200);
            }
        }
        ROS_INFO("--MOTORS UNDER VOLTAGE------------> OK--");
        return true;
    }
    return false;
}

bool SpiritInterface::startup_routine()
{
  /* initialize GPIO pin, for security switch button */
  command.btnPin = mraa_gpio_init(BTN_PIN);
  mraa_gpio_dir(command.btnPin, MRAA_GPIO_IN);
  command.peak_fdcan_board_initialization();
  usleep(200);
  command.check_initial_ground_pose();
  ROS_INFO("--STARTUP ROUTINE DONE------------> OK--");
  usleep(200);
  return true;
}

bool SpiritInterface::send(
    const quad_msgs::LegCommandArray& last_leg_command_array_msg,
    const Eigen::VectorXd& user_tx_data) {

    //std::cout << "SEND FUNCTION" << std::endl;
    /** SEND COMMANDS TO MOTEUS CONTROLLERS JOINTS.
    * @brief Write moteus controllers datas :
    * @param[out] joint_position
    * @param[out] joint_velocity
    * @param[out] joint_fftorque
    * @param[out] joint_kp
    * @param[out] joint_kd */

    for (unsigned int jj=0; jj<12; ++jj) // for all 12 joints,
    {
      //std::cout << "joint = " << jj << std::endl;
      auto ids  = command.motor_adapters_[jj].getIdx(); // moteus controller id
      int port  = command.motor_adapters_[jj].getPort(); // select correct port on Peak canfd board
      auto sign = command.motor_adapters_[jj].getSign(); // in case of joint reverse rotation
      auto leg_index = command.motor_adapters_[jj].getLeg_index(); // for vector position feed
      auto joint_index = command.motor_adapters_[jj].getJoint_index(); // for vector position feed

      joint_position = (sign*(last_leg_command_array_msg.leg_commands[leg_index].motor_commands[joint_index].pos_setpoint))/TWO_M_PI;
      joint_velocity = (sign*(last_leg_command_array_msg.leg_commands[leg_index].motor_commands[joint_index].vel_setpoint))/TWO_M_PI;
      joint_fftorque = sign*last_leg_command_array_msg.leg_commands[leg_index].motor_commands[joint_index].torque_ff;

      joint_kp       = static_cast<short>( last_leg_command_array_msg.leg_commands[leg_index].motor_commands[joint_index].kp);
      joint_kd       = static_cast<short>( last_leg_command_array_msg.leg_commands[leg_index].motor_commands[joint_index].kd);

      command.send_to_moteus(ids, port, joint_position, joint_velocity, joint_fftorque, joint_kp, joint_kd); 
      usleep(120);
    }
    return true;
}


bool SpiritInterface::recv(sensor_msgs::JointState& joint_state_msg,
                           sensor_msgs::Imu& imu_msg,
                           Eigen::VectorXd& user_rx_data) {

    //std::cout << "RECEIVE FUNCTION" << std::endl;
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
      // servo inversions, and radians to turn/s (moteus protocol)
      joint_state_msg.position[jj] = static_cast<double>(sign*(RX_pos*TWO_M_PI)); // joint turns to radians
      joint_state_msg.velocity[jj] = static_cast<double>(sign*(RX_vel*TWO_M_PI));   // turns in radians / s
      joint_state_msg.effort[jj]   = static_cast<double>(sign*RX_tor);   // measured in N*m
      usleep(200);
    }


    // imu variable is correctly feeded according to quad_sdk need
    // imu_msg.angular_velocity.xyz and imu_msg.linear_acceleration.xyz are OK
    imu_msg = YloTwoPcanToMoteus::ylo3_imu;
    //std::cout << imu_msg.orientation.x << std::endl;

    return true;
}