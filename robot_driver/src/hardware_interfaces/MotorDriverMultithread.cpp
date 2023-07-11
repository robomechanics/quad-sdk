#include "robot_driver/hardware_interfaces/MotorDriver.hpp"
#include <thread>

namespace motor_driver
{
    bool sendCANFrame(int can_id, unsigned char* CANMsg)
    {
        struct can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = 8;
        memcpy(frame.data, CANMsg, 8);

        // std::cout << "can_id in sendCANFrame: " << can_id << std::endl;

        auto socket_descrp_ = 0;  // being hardcoded might cause problems 

        // find better way to do this after proof of concept -> proven now fix!
        if (can_id == 10 || can_id == 6 || can_id == 11 || can_id == 7 || can_id == 3) {  // moved motor2 to can1
            // std::cout << "socket descrp = 3 (ie can0)" <<std::endl;
            socket_descrp_ = 3 ;
        }
        else {
            // std::cout << "socket descrp = 4 (ie can1)" <<std::endl;
            socket_descrp_ = 4;
        }

        if (write(socket_descrp_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            perror("CANInterface: Error writing to CAN Interface.");
            return false;
        }
        else
        {
            return true;
        }
    }

    MotorDriver::MotorDriver(const std::vector<int>& motor_ids, const char* motor_can_socket, MotorType motor_type=MotorType::AK80_6_V1p1) 
        : motor_type_(motor_type), motor_ids_(motor_ids), motor_CAN_interface_(motor_can_socket)
    {
        // Set Motor Parameters According to Motor Type

        switch (motor_type_)
        {
            case MotorType::AK80_6_V1:
                std::cout << "Using Motor Type AK80-6 V1" << std::endl;
                current_params_ = default_params::AK80_6_V1_params;
                break;
            case MotorType::AK80_6_V1p1:
                std::cout << "Using Motor Type AK80-6 V1.1" << std::endl;
                current_params_ = default_params::AK80_6_V1p1_params;
                break;
            case MotorType::AK80_6_V2:
                std::cout << "Using Motor Type AK80-6 V2" << std::endl;
                current_params_ = default_params::AK80_6_V2_params;
                break;
            case MotorType::AK80_9_V1p1:
                std::cout << "Using Motor Type AK80-9 V1.1" << std::endl;
                current_params_ = default_params::AK80_9_V1p1_params;
                break;
            case MotorType::AK80_9_V2:
                std::cout << "Using Motor Type AK80-9 V2" << std::endl;
                current_params_ = default_params::AK80_9_V2_params;
                break;
            case MotorType::AK70_10V1p1:
                std::cout << "Using Motor Type AK70-10 V1.1" << std::endl;
                current_params_ = default_params::AK70_10_V1p1_params;
                break;
            case MotorType::AK10_9_V1p1:
                std::cout << "Using Motor Type AK10-9 V1.1" << std::endl;
                current_params_ = default_params::AK10_9_V1p1_params;
                break;
            default:
                perror("Specified Motor Type Not Found!!");
        }

        // Initialize all Motors to not enabled.
        // TODO: Enable enabled check better across multiple objects of this class.
        for (int motor_id : motor_ids_)
            is_motor_enabled_[motor_id] = false;
    }


    std::map<int, motorState> MotorDriver::enableMotor(const std::vector<int>& enable_motor_ids)
    {
        std::map<int, motorState> motor_state_map;
        
        motorState state;
        for (int motor_id : enable_motor_ids)
        {
            // TODO: Add check that enable motor id is in initialized motor_ids_ vector
            // using std::find
            motor_CAN_interface_.sendCANFrame(motor_id, default_msgs::motorEnableMsg);
            usleep(motorReplyWaitTime);
            if (motor_CAN_interface_.receiveCANFrame(CAN_reply_msg_))
            {
                state = decodeCANFrame(CAN_reply_msg_);
                is_motor_enabled_[motor_id] = true;
            }
            else
            {
                perror("MotorDriver::enableMotor() Unable to Receive CAN Reply.");
            }

            if (motor_id != state.motor_id)
                perror("MotorDriver::enableMotor() Received message does not have the same motor id!!");

            motor_state_map[motor_id] = state;
        }
        return motor_state_map;
    }


    std::map<int, motorState> MotorDriver::disableMotor(const std::vector<int>& disable_motor_ids)
    {
        std::map<int, motorState> motor_state_map;
        
        motorState state;
        for (int motor_id : disable_motor_ids)
        {
            // TODO: Add check that enable motor id is in initialized motor_ids_ vector
            // using std::find.
            // TODO: Enable enabled check better across multiple objects of this class.
            // if (is_motor_enabled_[disable_motor_ids[iterId]])
            // {
            //     std::cout << "MotorDriver::disableMotor() Motor seems to already be in disabled state. \
            //                   Did you want to really do this?" << std::endl;
            // }

            // Bugfix: To remove the initial kick at motor start.
            // The current working theory is that the motor "remembers" the last command. And this
            // causes an initial kick as the motor controller starts. The fix is then to set the 
            // last command to zero so that this does not happen. For the user, the behaviour does
            // not change as zero command + disable is same as disable.
            bool return_val = encodeCANFrame(default_msgs::zeroCmdStruct, CAN_msg_);
            motor_CAN_interface_.sendCANFrame(motor_id, CAN_msg_);
            usleep(motorReplyWaitTime);
            
            if (motor_CAN_interface_.receiveCANFrame(CAN_reply_msg_))
            {
                state = decodeCANFrame(CAN_reply_msg_);
            }
            else
            {
                perror("MotorDriver::disableMotor() Unable to Receive CAN Reply.");
            }

            // Do the actual disabling after zero command.
            motor_CAN_interface_.sendCANFrame(motor_id, default_msgs::motorDisableMsg);
            usleep(motorReplyWaitTime);
            if (motor_CAN_interface_.receiveCANFrame(CAN_reply_msg_))
            {
                state = decodeCANFrame(CAN_reply_msg_);
                is_motor_enabled_[motor_id] = false;
            }
            else
            {
                perror("MotorDriver::disableMotor() Unable to Receive CAN Reply.");
            }

            if (motor_id != state.motor_id)
                perror("MotorDriver::disableMotor() Received message does not have the same motor id!!");
                
            motor_state_map[motor_id] = state;
        }
        return motor_state_map;
    }


    std::map<int, motorState> MotorDriver::setZeroPosition(const std::vector<int>& zero_motor_ids)
    {
        std::map<int, motorState> motor_state_map;

        motorState state;
        for (int motor_id : zero_motor_ids)
        {
            // TODO: Add check that enable motor id is in initialized motor_ids_ vector
            // using std::find
            // TODO: Enable enabled check better across multiple objects of this class.
            // if (is_motor_enabled_[motor_id])
            // {
            //     std::cout << "MotorDriver::setZeroPosition() Motor in disabled state.\
            //                   Did you want to really do this?" << std::endl;
            // }
            motor_CAN_interface_.sendCANFrame(motor_id, default_msgs::motorSetZeroPositionMsg);
            usleep(motorReplyWaitTime);
            if (motor_CAN_interface_.receiveCANFrame(CAN_reply_msg_))
            {
                state = decodeCANFrame(CAN_reply_msg_);
                motor_state_map[motor_id] = state;
            }
            else
            {
                perror("MotorDriver::setZeroPosition() Unable to Receive CAN Reply.");
            }

            while (state.position > (1 * (pi / 180)))
            {
                motor_CAN_interface_.sendCANFrame(motor_id, default_msgs::motorSetZeroPositionMsg);
                usleep(motorReplyWaitTime);
                if (motor_CAN_interface_.receiveCANFrame(CAN_reply_msg_))
                {
                    state = decodeCANFrame(CAN_reply_msg_);
                    motor_state_map[motor_id] = state;
                }
                else
                {
                    perror("MotorDriver::setZeroPosition() Unable to Receive CAN Reply.");
                }
            }
        }
       return motor_state_map;
    }


    std::map<int, motorState> MotorDriver::sendRadCommand(const std::map<int, motorCommand>& motor_rad_commands)
    {
        motorState state;
        std::map<int, motorState> motor_state_map;
        int cmd_motor_id;

        std::vector<std::thread> v;
        for (const std::pair<int, motorCommand>& command_pair : motor_rad_commands)
        {
            cmd_motor_id = command_pair.first;
            const motorCommand& cmd_to_send = command_pair.second;

            bool return_val = encodeCANFrame(cmd_to_send, CAN_msg_);
            // TODO: Enable enabled check better across multiple objects of this class.
            // if (is_motor_enabled_[cmd_motor_id])
            // {
            //     std::cout << "MotorDriver::sendRadCommand() Motor in disabled state.\
            //                   Did you want to really do this?" << std::endl;
            // }

            // v.emplace_back(&CAN_interface::CANInterface::sendCANFrame, cmd_motor_id, CAN_msg_);
            v.emplace_back(&sendCANFrame, cmd_motor_id, CAN_msg_);
            // motor_CAN_interface_.sendCANFrame(cmd_motor_id, CAN_msg_);
            // usleep(motorReplyWaitTime);
        }

        for (auto& t : v) 
        {
            t.join();
        }
                
        for (auto& t : v) 
        {
            
            if (motor_CAN_interface_.receiveCANFrame(CAN_reply_msg_))
            {
                state = decodeCANFrame(CAN_reply_msg_);
                motor_state_map[cmd_motor_id] = state;
            }
            else
            {
                perror("MotorDriver::sendRadCommand() Unable to Receive CAN Reply.");
            }
        }

        return motor_state_map;
    }

    std::map<int, motorState> MotorDriver::sendDegreeCommand(const std::map<int, motorCommand>& motor_deg_commands)
    {
        std::map<int, motorCommand> motor_rad_commands = motor_deg_commands;

        for (auto& command_pair : motor_rad_commands)
        {
            command_pair.second.p_des *= (pi / 180);
            command_pair.second.v_des *= (pi / 180);
        }

        return sendRadCommand(motor_rad_commands);
    }

    const motorParams& MotorDriver::getMotorParams() const
    {
        return current_params_;
    }

    void MotorDriver::setMotorParams(const motorParams& new_params)
    {   
        current_params_ = new_params;
    }

    motorState MotorDriver::decodeCANFrame(const unsigned char* CAN_reply_msg) const
    {
        // unpack ints from can buffer
        int id = CAN_reply_msg[0];
        int p_int = (CAN_reply_msg[1] << 8) | CAN_reply_msg[2];
        int v_int = (CAN_reply_msg[3] << 4) | (CAN_reply_msg[4] >> 4);
        int i_int = ((CAN_reply_msg[4] & 0xF) << 8) | CAN_reply_msg[5];
        // convert unsigned ints to floats
        float p = uint_to_float(p_int, current_params_.P_MIN, current_params_.P_MAX, 16);
        float v = uint_to_float(v_int, current_params_.V_MIN, current_params_.V_MAX, 12);
        float i = uint_to_float(i_int, -current_params_.T_MAX, current_params_.T_MAX, 12); // here -T_MAX, in encode T_MIN

        motorState state {
            .motor_id = id,
            .position = p * current_params_.AXIS_DIRECTION,
            .velocity = v * current_params_.AXIS_DIRECTION,
            .torque = i * current_params_.AXIS_DIRECTION
        };

        return state;
    }

    bool MotorDriver::encodeCANFrame(const motorCommand& cmd_to_send, unsigned char* CAN_msg) const
    {
        float p_des = cmd_to_send.p_des * current_params_.AXIS_DIRECTION;
        float v_des = cmd_to_send.v_des * current_params_.AXIS_DIRECTION;
        float tau_ff = cmd_to_send.tau_ff * current_params_.AXIS_DIRECTION;

        // Apply Saturation based on the limits      
        p_des = fminf(fmaxf(current_params_.P_MIN, p_des), current_params_.P_MAX);
        v_des = fminf(fmaxf(current_params_.V_MIN, v_des), current_params_.V_MAX);
        tau_ff = fminf(fmaxf(current_params_.T_MIN, tau_ff), current_params_.T_MAX);
        float kp = fminf(fmaxf(current_params_.KP_MIN, cmd_to_send.kp), current_params_.KP_MAX);
        float kd = fminf(fmaxf(current_params_.KD_MIN, cmd_to_send.kd), current_params_.KD_MAX);

        // convert floats to unsigned ints
        int p_int = float_to_uint(p_des, current_params_.P_MIN, current_params_.P_MAX, 16);            
        int v_int = float_to_uint(v_des, current_params_.V_MIN, current_params_.V_MAX, 12);
        int kp_int = float_to_uint(kp, current_params_.KP_MIN, current_params_.KP_MAX, 12);
        int kd_int = float_to_uint(kd, current_params_.KD_MIN, current_params_.KD_MAX, 12);
        int t_int = float_to_uint(tau_ff, current_params_.T_MIN, current_params_.T_MAX, 12);

        // pack ints into the can message
        CAN_msg[0] = p_int >> 8;                                       
        CAN_msg[1] = p_int & 0xFF;
        CAN_msg[2] = v_int >> 4;
        CAN_msg[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
        CAN_msg[4] = kp_int & 0xFF;
        CAN_msg[5] = kd_int >> 4;
        CAN_msg[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
        CAN_msg[7] = t_int & 0xff;

        return true;
    }


    int MotorDriver::float_to_uint(float x, float x_min, float x_max, int bits) const
    {
        /// Converts a float to an unsigned int, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return (int) ((x-offset) * ((float)((1 << bits)-1)) / span);
    }


    float MotorDriver::uint_to_float(int x_int, float x_min, float x_max, int bits) const
    {
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits)-1)) + offset;
    }

} // motor driver namespace