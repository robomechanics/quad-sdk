#include "robot_driver/hardware_interfaces/new_platform_interface.h"
#include <thread>

NewPlatformInterface::NewPlatformInterface() {}

struct threadCommandStruct {
    std::vector<int> motorIds;
    motor_driver::MotorDriver *motorController;
    motor_driver::motorCommand commandStruct;
    std::map<int, motor_driver::motorState> motorStates;
} commandStruct_0, commandStruct_1;

std::map<int, motor_driver::motorCommand> createMotorCommand(std::vector<int> motorIds, motor_driver::motorCommand commandStruct) {
	
	int numMotors = motorIds.size();
	std::map<int, motor_driver::motorCommand> commandMap;
	for (int i = 0; i < numMotors; i++) {
		// cout<<"Desired Position: "<<commandStruct.p_des;
		commandMap.insert(std::pair<int, motor_driver::motorCommand>(motorIds[i], commandStruct));
	}
	
	return commandMap;
}

void sendMotorCommand_0(threadCommandStruct commandStruct_thread) {
    std::map<int, motor_driver::motorCommand> commandMap = createMotorCommand(commandStruct_thread.motorIds, commandStruct_thread.commandStruct);
    commandStruct_thread.motorStates = commandStruct_thread.motorController->sendRadCommand(commandMap);
}

void sendMotorCommand_1(threadCommandStruct commandStruct_thread) {
    std::map<int, motor_driver::motorCommand> commandMap = createMotorCommand(commandStruct_thread.motorIds, commandStruct_thread.commandStruct);
    commandStruct_thread.motorStates = commandStruct_thread.motorController->sendRadCommand(commandMap);
}

// Get time stamp in microseconds.
uint64_t get_time_in_microseconds()
{
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
                  now().time_since_epoch()).count();
    return us; 
}

void printMotorIds(std::vector<int> motorIds, int canBusNum) {
	int numMotors = motorIds.size();
    ROS_INFO("CAN %d. Motor Ids: {", canBusNum);
	for (int i = 0; i < numMotors; i++) {
        ROS_INFO("%d", motorIds[i]);
	}
    ROS_INFO("}");
}

void printMotorStates(std::vector<int> motorIds, std::map<int, motor_driver::motorState> motorStates) {
	int numMotors = motorIds.size();
	for (int i = 0; i < numMotors; i++) {
        ROS_INFO("Motor State ID: %d. Position: %d. Velocity: %d, Torque: %d", motorStates[motorIds[i]].motor_id, 
                motorStates[motorIds[i]].position, motorStates[motorIds[i]].velocity, motorStates[motorIds[i]].torque);
	}
}

void NewPlatformInterface::loadInterface(int argc, char** argv) {

    ROS_INFO("Loading Interface");
    ROS_INFO("0000000000000000");

    //Set zero positions and prepare motors for moving
    // motorStates_0 = motor_controller0.setZeroPosition(motor_ids_0);
    // motorStates_1 = motor_controller1.setZeroPosition(motor_ids_1);

    //Enable motors
    motorStates_0 = motor_controller0.enableMotor(motor_ids_0);
    motorStates_1 = motor_controller1.enableMotor(motor_ids_1);
    ROS_INFO("Motors enabled");

    ROS_INFO("CAN 0 Positions: ");
    printMotorStates(motor_ids_0, motorStates_0);
    ROS_INFO("CAN 1 Positions: ");
    printMotorStates(motor_ids_1, motorStates_1);

}

void NewPlatformInterface::unloadInterface() {

    //Disable motors and get final positions
    motorStates_0 = motor_controller0.disableMotor(motor_ids_0);
    motorStates_1 = motor_controller1.disableMotor(motor_ids_1);

    ROS_INFO("CAN 0 Positions: ");
    printMotorStates(motor_ids_0, motorStates_0);
    ROS_INFO("CAN 1 Positions: ");
    printMotorStates(motor_ids_1, motorStates_1);

}

bool NewPlatformInterface::send(
    const quad_msgs::LegCommandArray& last_leg_command_array_msg
    ,const Eigen::VectorXd& user_tx_data
    ) {

    int leg_command_heartbeat = 1;

    std::vector<std::thread> cmd_threads;

    auto startT = get_time_in_microseconds();

    // For FL, FR
    for (int i = 0; i < 4; ++i) {  // For each leg

        quad_msgs::LegCommand leg_command = last_leg_command_array_msg.leg_commands.at(i);

        int idx_offset = 0;

        switch (i){
        case 0: //FL
        case 2: //FR
            for (int j = 0; j < 3; ++j) {  // For each joint
            if(i==0){
                idx_offset = 0;  
            }
            if(i==2){
                idx_offset = 3;  
            }

            float pos = leg_command_heartbeat * motor_dirs_0[idx_offset+j] * leg_command.motor_commands.at(j).pos_setpoint;
            float vel = leg_command_heartbeat * motor_dirs_0[idx_offset+j] *leg_command.motor_commands.at(j).vel_setpoint;
            float tau = leg_command_heartbeat * motor_dirs_0[idx_offset+j] * leg_command.motor_commands.at(j).torque_ff;
            float kp = leg_command_heartbeat * leg_command.motor_commands.at(j).kp;
            float kd = leg_command_heartbeat * leg_command.motor_commands.at(j).kd;

            // ROS_INFO("Pos_Set: %f. Index: %d", pos, idx_offset+j);
            
            motor_driver::motorCommand commandStruct = {pos, vel, 0.3*kp, 0.1*kd, 0.0};

            commandStruct_0.motorIds = motor_ids_0;
            commandStruct_0.motorController = &motor_controller0;
            commandStruct_0.commandStruct = commandStruct;
            commandStruct_0.motorStates = motorStates_0;
            
            // Initialize thread for motor driver execution
            cmd_threads.emplace_back(&sendMotorCommand_0, commandStruct_0);
            }
            break;

        case 1: //BL
        case 3: //BR
            for (int j = 0; j < 3; ++j) {  // For each joint
            if(i==1){
                idx_offset = 0;  
            }
            if(i==3){
                idx_offset = 3;  
            }
    
            float pos = leg_command_heartbeat * motor_dirs_1[idx_offset+j] * leg_command.motor_commands.at(j).pos_setpoint;
            float vel = leg_command_heartbeat * motor_dirs_1[idx_offset+j] * leg_command.motor_commands.at(j).vel_setpoint;
            float tau = leg_command_heartbeat * motor_dirs_1[idx_offset+j] * leg_command.motor_commands.at(j).torque_ff;
            float kp = leg_command_heartbeat * leg_command.motor_commands.at(j).kp;
            float kd = leg_command_heartbeat * leg_command.motor_commands.at(j).kd;

            // ROS_INFO("Pos_Set: %f. Index: %d", pos, idx_offset+j);

            motor_driver::motorCommand commandStruct = {pos, vel, 0.3*kp, 0.1*kd, 0.0};

            commandStruct_1.motorIds = motor_ids_1;
            commandStruct_1.motorController = &motor_controller1;
            commandStruct_1.commandStruct = commandStruct;
            commandStruct_1.motorStates = motorStates_1;
            
            //Initialize thread for motor driver execution
            cmd_threads.emplace_back(&sendMotorCommand_1, commandStruct_1);
            }
            break;

        default:
            break;
        }
    }

    for (auto& t : cmd_threads) 
    {
        if (t.joinable()) {
            t.join();
        }
    }

    auto endT = get_time_in_microseconds();
    auto dt = (endT - startT);
    std::cout << "Time Taken for Command: " << double(dt/1e6) << std::endl;

    ROS_INFO("CAN 0 Positions: ");
    printMotorStates(motor_ids_0, motorStates_0);
    ROS_INFO("CAN 1 Positions: ");
    printMotorStates(motor_ids_1, motorStates_1);

    return true;
    }

    bool NewPlatformInterface::recv(sensor_msgs::JointState& joint_state_msg,
                            sensor_msgs::Imu& imu_msg,
                            Eigen::VectorXd& user_rx_data) {
    
    // Cycle through both CAN busses
    for (int i = 0; i < joint_names_0_.size(); i++) {
        joint_state_msg.name[i] = joint_names_0_[i];
        joint_state_msg.position[i] = motorStates_0[motor_ids_0[i]].position;
        joint_state_msg.velocity[i] = motorStates_0[motor_ids_0[i]].velocity;
        joint_state_msg.effort[i] = motorStates_0[motor_ids_0[i]].torque;
    }
    
    for (int j = 0; j < joint_names_1_.size(); j++) {
        joint_state_msg.name[j+joint_names_0_.size()] = joint_names_1_[j];
        joint_state_msg.position[j+joint_names_0_.size()] = motorStates_1[motor_ids_1[j]].position;
        joint_state_msg.velocity[j+joint_names_0_.size()] = motorStates_1[motor_ids_1[j]].velocity;
        joint_state_msg.effort[j+joint_names_0_.size()] = motorStates_1[motor_ids_1[j]].torque;
    }

    // ROS_INFO("Motor %c Position Received: %f", joint_state_msg.name[1], joint_state_msg.position[1]);

    return true;
    }
