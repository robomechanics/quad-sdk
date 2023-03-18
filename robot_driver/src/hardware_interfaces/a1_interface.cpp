#include "robot_driver/hardware_interfaces/a1_interface.h"

A1Interface::A1Interface() {}

void A1Interface::loadInterface(int argc, char** argv) {
    while(!A1Interface::startup_routine());
}

void A1Interface::unloadInterface() {}

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

YloTwoPcanToMoteus command_; // instance of class YloTwoPcanToMoteus

bool A1Interface::startup_routine()
{
    /* initialize GPIO pin */
  command_.btnPin = mraa_gpio_init(BTN_PIN);
  mraa_gpio_dir(command_.btnPin, MRAA_GPIO_IN);

  command_.peak_fdcan_board_initialization();
  usleep(200);
  command_.check_initial_ground_pose();
  std::cout << "startup_routine Done." << std::endl;
  usleep(3000000);
  return true;
}


bool A1Interface::send(const quad_msgs::LegCommandArray& last_leg_command_array_msg, const Eigen::VectorXd& user_tx_data) 
{
    //std::cout << "SEND FUNCTION" << std::endl;

    // ask security switch status
    // if pressed, it directly set maxtorque to 0
    command_.security_switch();

    bool restart_flag = (user_tx_data[0] == 1); //TODO remove user_tx_data ?

    /** SEND COMMANDS TO MOTEUS CONTROLLERS JOINTS.
    * @brief Write moteus controllers datas :
    * @param[out] joint_position
    * @param[out] joint_velocity
    * @param[out] joint_fftorque
    * @param[out] joint_kp
    * @param[out] joint_kd */

    for (unsigned int jj=0; jj<12; ++jj) // for all 12 joints,
    {
      //std::cout << "joint = " << jj << std::endl; // 1, 2, 3, 4, 5...
      auto ids  = command_.motor_adapters_[jj].getIdx(); // moteus controller id
      int port  = command_.motor_adapters_[jj].getPort(); // select correct port on Peak canfd board
      auto sign = command_.motor_adapters_[jj].getSign(); // in case of joint reverse rotation
      auto leg_index = command_.motor_adapters_[jj].getLeg_index(); // for vector position feed
      auto joint_index = command_.motor_adapters_[jj].getJoint_index(); // same

      joint_position = (sign*(last_leg_command_array_msg.leg_commands[leg_index].motor_commands[joint_index].pos_setpoint)/(2*M_PI)); // radians to turns (for moteus)
      joint_velocity = last_leg_command_array_msg.leg_commands[leg_index].motor_commands[joint_index].vel_setpoint/(2*M_PI); // radians to turns (for moteus)
      joint_fftorque = last_leg_command_array_msg.leg_commands[leg_index].motor_commands[joint_index].torque_ff;
      joint_kp       = static_cast<short>( last_leg_command_array_msg.leg_commands[leg_index].motor_commands[joint_index].kp);
      joint_kd       = static_cast<short>( last_leg_command_array_msg.leg_commands[leg_index].motor_commands[joint_index].kd);

      //std::cout << "maxtorque is "<< command_._comm_maxtorque << std::endl; // switch from maxtorque value to 0, if security switch is pressed.
      command_.send_moteus_TX_frame(ids, port, joint_position, joint_velocity, joint_fftorque, joint_kp, joint_kd); 
      usleep(80);
    }
    return true;
}

bool A1Interface::recv(sensor_msgs::JointState& joint_state_msg, sensor_msgs::Imu& imu_msg, Eigen::VectorXd& user_rx_data) 
{
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
      auto ids  = command_.motor_adapters_[jj].getIdx();
      int port  = command_.motor_adapters_[jj].getPort();
      auto sign = command_.motor_adapters_[jj].getSign();

      command_.read_moteus_RX_queue(ids, port, RX_pos, RX_vel, RX_tor, RX_volt, RX_temp, RX_fault);  // query values;
      usleep(10);

      joint_state_msg.name[jj]     = joint_names_[jj];
      joint_state_msg.position[jj] = static_cast<double>(sign*(RX_pos*2*M_PI)); // moteus turns to radians
      joint_state_msg.velocity[jj] = static_cast<double>(RX_vel*2*M_PI);   // moteus turns to radians
      joint_state_msg.effort[jj]   = static_cast<double>(RX_tor);   // measured in N*m
      usleep(80);
    }

    /** RECEIVE IMU VALUES.
    * @brief Read IMU data
    * @param[in] last_imu_msg IMU sensor message
    * @param[out] Linear acceleration
    * @param[out] Angular acceleration
    * @param[out] Orientation in quaternion */

    return true;
}

