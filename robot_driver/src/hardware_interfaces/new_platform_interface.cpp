#include "robot_driver/hardware_interfaces/new_platform_interface.h"

NewPlatformInterface::NewPlatformInterface() {}

void NewPlatformInterface::loadInterface(int argc, char** argv) {
  ROS_INFO("Loading Interface");
  ROS_INFO("0000000000000000");
  //Set zero positions and prepare motors for moving
  // motorStates_0 = motor_controller0.setZeroPosition(motor_ids_0);
  // motorStates_1 = motor_controller1.setZeroPosition(motor_ids_1);
  // std::string posMessage0 = "[";
  // std::string posMessage1 = "[";
  // for (int i = 0; i < 6; i++) {
  //   if (i == 5) {
  //     posMessage0 = posMessage0 + std::to_string(motorStates_0[i].position) + "]";
  //     posMessage1 = posMessage1 + std::to_string(motorStates_1[i].position) + "]";
  //   } else {
  //     posMessage0 = posMessage0 + std::to_string(motorStates_0[i].position) + ", ";
  //     posMessage1 = posMessage1 + std::to_string(motorStates_1[i].position) + ", ";
  //   }
  // }
  // ROS_INFO_STREAM("CAN 0 Positions: " << posMessage0);
  // ROS_INFO_STREAM("CAN 1 Positions: " << posMessage1);
  //Enable motors
  motorStates_0 = motor_controller0.enableMotor(motor_ids_0);
  motorStates_1 = motor_controller1.enableMotor(motor_ids_1);
  ROS_INFO("Motors enabled");
  //Merge the front and rear leg mappings
  // motorStates.insert(motorStates_0.begin(), motorStates_0.end());
  // motorStates.insert(motorStates_1.begin(), motorStates_1.end());

}

void NewPlatformInterface::unloadInterface() {

  //Disable motors and get final positions
  motorStates_0 = motor_controller0.disableMotor(motor_ids_0);
  motorStates_1 = motor_controller1.disableMotor(motor_ids_1);

  // //Merge the front and rear leg mappings
  // motorStates.insert(motorStates_0.begin(), motorStates_0.end());
  // motorStates.insert(motorStates_1.begin(), motorStates_1.end());

}

bool NewPlatformInterface::send(
    const quad_msgs::LegCommandArray& last_leg_command_array_msg
    ,const Eigen::VectorXd& user_tx_data
    ) {

  int leg_command_heartbeat = 1;
  // ROS_INFO("Sending positions");
  std::map<int, motor_driver::motorCommand> commandMap_0; //Command map for font legs
  std::map<int, motor_driver::motorCommand> commandMap_1; //Command map for rear legs

  // // For FL, FR
  for (int i = 0; i < 4; ++i) {  // For each leg
    // std::cout << "leg = " << i << std::endl;
    // ROS_INFO("sending %d start",i);
    quad_msgs::LegCommand leg_command =
        last_leg_command_array_msg.leg_commands.at(i);
    // ROS_INFO("sending %d",i);
    int idx_offset = 0;
    switch (i){
      case 0: //FL
      case 2: //FR
        for (int j = 0; j < 3; ++j) {  // For each joint
          // std::cout << "joint = " << j << std::endl;
          if(i==0){
            idx_offset = 0;  
          }
          if(i==2){
            idx_offset = 3;  
          }
          // float pos = leg_command_heartbeat * leg_command.motor_commands.at(j).pos_setpoint;
          // float vel = leg_command_heartbeat * leg_command.motor_commands.at(j).vel_setpoint;
          // float tau = leg_command_heartbeat * leg_command.motor_commands.at(j).torque_ff;
          // float kp = leg_command_heartbeat * leg_command.motor_commands.at(j).kp;
          // float kd = leg_command_heartbeat * leg_command.motor_commands.at(j).kd;

          float pos = leg_command_heartbeat * motor_dirs_0[idx_offset+j] * leg_command.motor_commands.at(j).pos_setpoint;
          float vel = leg_command_heartbeat * motor_dirs_0[idx_offset+j] *leg_command.motor_commands.at(j).vel_setpoint;
          float tau = leg_command_heartbeat * motor_dirs_0[idx_offset+j] * leg_command.motor_commands.at(j).torque_ff;
          float kp = leg_command_heartbeat * leg_command.motor_commands.at(j).kp;
          float kd = leg_command_heartbeat * leg_command.motor_commands.at(j).kd;

          // if (leg_command.motor_commands.at(j).pos_setpoint > 0) {
          //   axis = j;
          //   sendPos = pos;
          // }
          // ROS_INFO("Pos_Set: %f   Indx: %i",pos,idx_offset+j);
          // motor_driver::motorCommand commandStruct = {pos, vel, 0.2*kp, 0.2*kd, 0.5*tau};
          motor_driver::motorCommand commandStruct = {pos, vel, 0.3*kp, 0.1*kd, 0.0};
          // motor_driver::motorCommand commandStruct = {pos, vel, 5.0, 1.0, tau};
          // motor_driver::motorCommand commandStruct = {pos, vel, kp, kd, tau};
          // motor_driver::motorCommand commandStruct = {0.0, 0.0, 5.0, 1.0, 0.0};
          commandMap_0.insert(std::pair<int, motor_driver::motorCommand> (motor_ids_0[idx_offset+j], commandStruct));
        }
        break;
      case 1: //BL
      case 3: //BR
        for (int j = 0; j < 3; ++j) {  // For each joint
          // std::cout << "joint = " << j << std::endl;
          if(i==1){
            idx_offset = 0;  
          }
          if(i==3){
            idx_offset = 3;  
          }
          // float pos = leg_command_heartbeat * leg_command.motor_commands.at(j).pos_setpoint;
          // float vel = leg_command_heartbeat * leg_command.motor_commands.at(j).vel_setpoint;
          // float tau = leg_command_heartbeat * leg_command.motor_commands.at(j).torque_ff;
          // float kp = leg_command_heartbeat * leg_command.motor_commands.at(j).kp;
          // float kd = leg_command_heartbeat * leg_command.motor_commands.at(j).kd;
          float pos = leg_command_heartbeat * motor_dirs_1[idx_offset+j] * leg_command.motor_commands.at(j).pos_setpoint;
          float vel = leg_command_heartbeat * motor_dirs_1[idx_offset+j] * leg_command.motor_commands.at(j).vel_setpoint;
          float tau = leg_command_heartbeat * motor_dirs_1[idx_offset+j] * leg_command.motor_commands.at(j).torque_ff;
          float kp = leg_command_heartbeat * leg_command.motor_commands.at(j).kp;
          float kd = leg_command_heartbeat * leg_command.motor_commands.at(j).kd;

          // if (leg_command.motor_commands.at(j).pos_setpoint > 0) {
          //   axis = j;
          //   sendPos = pos;
          // }
          // ROS_INFO("Pos_Set: %f   Indx: %i",pos, idx_offset+j);
          // motor_driver::motorCommand commandStruct = {pos, vel, 0.2*kp, 0.2*kd, 0.5*tau};
          motor_driver::motorCommand commandStruct = {pos, vel, 0.3*kp, 0.1*kd, 0.0};
          // motor_driver::motorCommand commandStruct = {pos, vel, kp, kd, tau};
          // motor_driver::motorCommand commandStruct = {pos, vel, 5.0, 1.0, tau};
          // motor_driver::motorCommand commandStruct = {0.0, 0.0, 5.0, 1.0, 0.0};
          commandMap_1.insert(std::pair<int, motor_driver::motorCommand> (motor_ids_1[idx_offset+j], commandStruct));
        }
        break;
      default:
        break;
    }
  }
  // ROS_INFO("Command ready for sending");

  motorStates_0 = motor_controller0.sendRadCommand(commandMap_0);
  motorStates_1 = motor_controller1.sendRadCommand(commandMap_1);
  // ROS_INFO("Motor %i position: %f", motor_ids_0[1], motorStates_0[motor_ids_0[1]].position);
  //Merge the front and rear leg mappings
  // motorStates.insert(motorStates_0.begin(), motorStates_0.end());
  // motorStates.insert(motorStates_1.begin(), motorStates_1.end());

  return true;
}

bool NewPlatformInterface::recv(sensor_msgs::JointState& joint_state_msg,
                           sensor_msgs::Imu& imu_msg,
                           Eigen::VectorXd& user_rx_data) {
  
  // float pos = motorStates[1].position;
  // ROS_INFO("%f", pos);
  
  // Cycle through both CAN busses
  for (int i = 0; i < joint_names_0_.size(); i++) {
    joint_state_msg.name[i] = joint_names_0_[i];
    joint_state_msg.position[i] = motorStates_0[motor_ids_0[i]].position;
    joint_state_msg.velocity[i] = motorStates_0[motor_ids_0[i]].velocity;
    joint_state_msg.effort[i] = motorStates_0[motor_ids_0[i]].torque;

    joint_state_msg.name[i+joint_names_0_.size()] = joint_names_1_[i];
    joint_state_msg.position[i+joint_names_0_.size()] = motorStates_1[motor_ids_1[i]].position;
    joint_state_msg.velocity[i+joint_names_0_.size()] = motorStates_1[motor_ids_1[i]].velocity;
    joint_state_msg.effort[i+joint_names_0_.size()] = motorStates_1[motor_ids_1[i]].torque;
  }

  // ROS_INFO("Motor %s position: %f", joint_state_msg.name[1], joint_state_msg.position[1]);

  return true;
}
