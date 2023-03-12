#include "robot_driver/hardware_interfaces/new_platform_interface.h"

NewPlatformInterface::NewPlatformInterface() {}

void NewPlatformInterface::loadInterface(int argc, char** argv) {

  //Set zero positions and prepare motors for moving
  motorStates_0 = motor_controller0.setZeroPosition(motor_ids_0);
  motorStates_1 = motor_controller1.setZeroPosition(motor_ids_1);

  //Enable motors
  motorStates_0 = motor_controller0.enableMotor(motor_ids_0);
  motorStates_1 = motor_controller1.enableMotor(motor_ids_1);

  //Merge the front and rear leg mappings
  motorStates.insert(motorStates_0.begin(), motorStates_0.end());
  motorStates.insert(motorStates_1.begin(), motorStates_1.end());

}

void NewPlatformInterface::unloadInterface() {

  //Disable motors and get final positions
  motorStates_0 = motor_controller0.disableMotor(motor_ids_0);
  motorStates_1 = motor_controller1.disableMotor(motor_ids_1);

  //Merge the front and rear leg mappings
  motorStates.insert(motorStates_0.begin(), motorStates_0.end());
  motorStates.insert(motorStates_1.begin(), motorStates_1.end());

}

bool NewPlatformInterface::send(
    const quad_msgs::LegCommandArray& last_leg_command_array_msg
    ,const Eigen::VectorXd& user_tx_data
    ) {

  int leg_command_heartbeat = 1;

  std::map<int, motor_driver::motorCommand> commandMap_0; //Command map for font legs
  std::map<int, motor_driver::motorCommand> commandMap_1; //Command map for rear legs

  int axis = 3;
  float sendPos = 0;

  //For FL, FR
  LimbCmd_t limbcmd[4];
  for (int i = 0; i < 4; ++i) {  // For each leg
    // std::cout << "leg = " << i << std::endl;
    quad_msgs::LegCommand leg_command =
        last_leg_command_array_msg.leg_commands.at(i);

    switch (i){
      case 0: //FL
      case 2: //FR
        for (int j = 0; j < 3; ++j) {  // For each joint
          // std::cout << "joint = " << j << std::endl;
          float pos = leg_command_heartbeat * leg_command.motor_commands.at(j).pos_setpoint;
          float vel = leg_command_heartbeat * leg_command.motor_commands.at(j).vel_setpoint;
          float tau = leg_command_heartbeat * leg_command.motor_commands.at(j).torque_ff;
          float kp = leg_command_heartbeat * leg_command.motor_commands.at(j).kp;
          float kd = leg_command_heartbeat * leg_command.motor_commands.at(j).kd;

          if (leg_command.motor_commands.at(j).pos_setpoint > 0) {
            axis = j;
            sendPos = pos;
          }

          // motor_driver::motorCommand commandStruct = {pos, vel, kp, kd, tau};
          // commandMap_0.insert(std::pair<int, motor_driver::motorCommand> (motor_ids_0[i*3 + j], commandStruct));
        }
        break;
      case 1: //BL
      case 3: //BR
        for (int j = 0; j < 3; ++j) {  // For each joint
          // std::cout << "joint = " << j << std::endl;
          float pos = leg_command_heartbeat * leg_command.motor_commands.at(j).pos_setpoint;
          float vel = leg_command_heartbeat * leg_command.motor_commands.at(j).vel_setpoint;
          float tau = leg_command_heartbeat * leg_command.motor_commands.at(j).torque_ff;
          float kp = leg_command_heartbeat * leg_command.motor_commands.at(j).kp;
          float kd = leg_command_heartbeat * leg_command.motor_commands.at(j).kd;

          if (leg_command.motor_commands.at(j).pos_setpoint > 0) {
            axis = j;
            sendPos = pos;
          }

          // motor_driver::motorCommand commandStruct = {pos, vel, kp, kd, tau};
          // commandMap_1.insert(std::pair<int, motor_driver::motorCommand> (motor_ids_1[i*3 + j], commandStruct));
        }
        break;
      default:
        break;
    }
  }

  //For RL, RR
  if (axis < 3) {
    ROS_INFO("Sending %f command to axis %i\n", sendPos, axis);
  }

  // motorStates_0 = motor_controller0.sendRadCommand(commandMap_0);
  // motorStates_1 = motor_controller1.sendRadCommand(commandMap_1);

  // //Merge the front and rear leg mappings
  // motorStates.insert(motorStates_0.begin(), motorStates_0.end());
  // motorStates.insert(motorStates_1.begin(), motorStates_1.end());

  return true;
}

bool NewPlatformInterface::recv(sensor_msgs::JointState& joint_state_msg,
                           sensor_msgs::Imu& imu_msg,
                           Eigen::VectorXd& user_rx_data) {
  
  // ROS_INFO("Recieving Command to robot\n");
  
  // Add the data corresponding to each joint
  for (int i = 0; i < joint_names_.size(); i++) {
    joint_state_msg.name[i] = joint_names_[i];
    joint_state_msg.position[i] = motorStates[i].position;
    joint_state_msg.velocity[i] = motorStates[i].velocity;
    joint_state_msg.effort[i] = motorStates[i].torque;
  }

  //Also want to add temperature reading once we get that data

  //Need to add IMU update code once we have an IMU


  return true;
}
