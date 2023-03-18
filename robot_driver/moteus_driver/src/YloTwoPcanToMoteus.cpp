/*
Copyright (c) 08/2022, Vincent FOUCAULT
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "moteus_driver/YloTwoPcanToMoteus.hpp"

YloTwoPcanToMoteus::YloTwoPcanToMoteus()
{

  pcanPorts_.resize(4); // resize the pcanports_ vector to the number or real ports.

  // NOTE: we should load that from file
  motor_adapters_.resize(12);  // exact motors order, on Ylo2, under Quad-Sdk controller

/******************************************************************************************
                           ABAD      ---        HIP       ---       KNEE
                       pos vel tor kp kd ...
   moteus_command = { {{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}}, --- FL
                    ...}
*******************************************************************************************/


  //                   IDX                             SIGN                         PCAN BOARD PORTS
  // LF
  /*HAA*/ motor_adapters_[0].setIdx(3);  motor_adapters_[0].setSign(-1); motor_adapters_[0].setLeg_index(0); 
                                        motor_adapters_[0].setJoint_index(0); motor_adapters_[0].setPort(PCAN_DEV1);

  /*HFE*/ motor_adapters_[1].setIdx(1);  motor_adapters_[1].setSign(1); motor_adapters_[1].setLeg_index(0); 
                                        motor_adapters_[1].setJoint_index(1); motor_adapters_[1].setPort(PCAN_DEV1);

  /*KFE*/ motor_adapters_[2].setIdx(2);  motor_adapters_[2].setSign(1); motor_adapters_[2].setLeg_index(0); 
                                        motor_adapters_[2].setJoint_index(2); motor_adapters_[2].setPort(PCAN_DEV1);

  // LH
  /*HAA*/ motor_adapters_[3].setIdx(9);  motor_adapters_[3].setSign(1);  motor_adapters_[3].setLeg_index(1); 
                                        motor_adapters_[3].setJoint_index(0); motor_adapters_[3].setPort(PCAN_DEV4);

  /*HFE*/ motor_adapters_[4].setIdx(7);  motor_adapters_[4].setSign(1); motor_adapters_[4].setLeg_index(1); 
                                        motor_adapters_[4].setJoint_index(1); motor_adapters_[4].setPort(PCAN_DEV4);

  /*KFE*/ motor_adapters_[5].setIdx(8);  motor_adapters_[5].setSign(1); motor_adapters_[5].setLeg_index(1); 
                                        motor_adapters_[5].setJoint_index(2); motor_adapters_[5].setPort(PCAN_DEV4);

  // RF
  /*HAA*/ motor_adapters_[6].setIdx(6);  motor_adapters_[6].setSign(1); motor_adapters_[6].setLeg_index(2); 
                                        motor_adapters_[6].setJoint_index(0); motor_adapters_[6].setPort(PCAN_DEV3);

  /*HFE*/ motor_adapters_[7].setIdx(4);  motor_adapters_[7].setSign(-1);  motor_adapters_[7].setLeg_index(2); 
                                        motor_adapters_[7].setJoint_index(1); motor_adapters_[7].setPort(PCAN_DEV3);

  /*KFE*/ motor_adapters_[8].setIdx(5);  motor_adapters_[8].setSign(-1);  motor_adapters_[8].setLeg_index(2); 
                                        motor_adapters_[8].setJoint_index(2); motor_adapters_[8].setPort(PCAN_DEV3);

  // RH
  /*HAA*/ motor_adapters_[9].setIdx(12);  motor_adapters_[9].setSign(-1);  motor_adapters_[9].setLeg_index(3); 
                                        motor_adapters_[9].setJoint_index(0); motor_adapters_[9].setPort(PCAN_DEV2);

  /*HFE*/ motor_adapters_[10].setIdx(10); motor_adapters_[10].setSign(-1); motor_adapters_[10].setLeg_index(3); 
                                        motor_adapters_[10].setJoint_index(1); motor_adapters_[10].setPort(PCAN_DEV2);

  /*KFE*/ motor_adapters_[11].setIdx(11); motor_adapters_[11].setSign(-1); motor_adapters_[11].setLeg_index(3); 
                                        motor_adapters_[11].setJoint_index(2); motor_adapters_[11].setPort(PCAN_DEV2);

  // The default ID for the power_dist is '32'
  motor_adapters_[32].setIdx(32); motor_adapters_[32].setPort(PCAN_DEV3);

  /* ------------------------ TX STOP PACKAGE ------------------------------ */
  _stop.ID      = 0x00;
  _stop.MSGTYPE = PCAN_MESSAGE_BRS | PCAN_MESSAGE_EXTENDED | PCAN_MESSAGE_FD;
  _stop.DLC     = 7;
  _stop.DATA[0] = 0x01; // Write uint8 (0x00) | Write 1 register (0x01)
  _stop.DATA[1] = 0x00; // Register to write: MODE
  _stop.DATA[2] = 0x00; // Value to write: STOPPED MODE
  _stop.DATA[3] = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
  _stop.DATA[4] = 0x01; // Starting register: POSITION, VELOCITY, TORQUE
  _stop.DATA[5] = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
  _stop.DATA[6] = 0x0D; // Starting register: VOLTAGE, TEMPERATURE, FAULT

  /* ------------------------- TX ZERO PACKAGE ------------------------------ */
  _zero.ID       = 0x00;
  _zero.MSGTYPE  = PCAN_MESSAGE_BRS | PCAN_MESSAGE_EXTENDED | PCAN_MESSAGE_FD;
  _zero.DLC      = 9; // 12 bytes ... ex : 0db0029a99193e    for a zero pos = 0.15
  _zero.DATA[0]  = 0x0D; // write float (0x0C) | write 1 register (0x01)
  _zero.DATA[1]  = 0xB0; // Register to write: 0x130(REZERO)
  _zero.DATA[2]  = 0x02;
  _zero.DATA[7]  = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
  _zero.DATA[8]  = 0x01; // Starting register: POSITION, VELOCITY, TORQUE
  _zero.DATA[9]  = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
  _zero.DATA[10] = 0x0D;// Starting register: VOLTAGE, TEMPERATURE, FAULT
  _zero.DATA[11] = 0x50;// pad unused bytes to 0x50

  /* --------------------------TX POS PACKAGE -------------------------------*/
  moteus_tx_msg.ID       = 0x00;
  moteus_tx_msg.MSGTYPE  = PCAN_MESSAGE_BRS | PCAN_MESSAGE_EXTENDED | PCAN_MESSAGE_FD;
  moteus_tx_msg.DLC      = 14; // 12 = 24 bytes, 13 = 32 bytes, 14 = 48 bytes
  moteus_tx_msg.DATA[0]  =  0x01; // WRITE_REGISTERS - Type.INT8 1 registers
  moteus_tx_msg.DATA[1]  =  0x00; // Starting at reg 0x000(MODE)
  moteus_tx_msg.DATA[2]  =  0x0a; // Reg 0x000(MODE) = 10(POSITION)
  moteus_tx_msg.DATA[3]  =  0x0c; // WRITE_REGISTERS - Type.F32
  moteus_tx_msg.DATA[4]  =  0x06; // 6 registers
  moteus_tx_msg.DATA[5]  =  0x20; // Starting at reg 0x020 POSITION
  moteus_tx_msg.DATA[6]  = _comm_position;
  moteus_tx_msg.DATA[10] = _comm_velocity;
  moteus_tx_msg.DATA[14] = _comm_fftorque;
  moteus_tx_msg.DATA[18] = _comm_kp;
  moteus_tx_msg.DATA[22] = _comm_kd;
  moteus_tx_msg.DATA[26] = _comm_maxtorque;
  moteus_tx_msg.DATA[30] = 0x1F; // READ_REGISTERS - F32 3 registers
  moteus_tx_msg.DATA[31] = 0x01; // Starting at reg 0x001(POSITION)
  moteus_tx_msg.DATA[32] = 0x13; // READ_REGISTERS - INT8 3 registers
  moteus_tx_msg.DATA[33] = 0x0D; // Starting at reg 0x00d(VOLTAGE)
  moteus_tx_msg.DATA[34] = 0x50; // Padding a NOP byte (moteus protocol)
  moteus_tx_msg.DATA[35] = 0x50; //    ...................
  moteus_tx_msg.DATA[36] = 0x50;
  moteus_tx_msg.DATA[37] = 0x50;
  moteus_tx_msg.DATA[38] = 0x50;
  moteus_tx_msg.DATA[39] = 0x50;
  moteus_tx_msg.DATA[40] = 0x50;
  moteus_tx_msg.DATA[41] = 0x50;
  moteus_tx_msg.DATA[42] = 0x50;
  moteus_tx_msg.DATA[43] = 0x50;
  moteus_tx_msg.DATA[44] = 0x50;
  moteus_tx_msg.DATA[45] = 0x50;
  moteus_tx_msg.DATA[46] = 0x50;
  moteus_tx_msg.DATA[47] = 0x50;

  /* ------------------- TX POWER BOARD PACKAGE -----------------------------*/
  _power_board_tx_msg.ID      = 0x00;
  _power_board_tx_msg.MSGTYPE = PCAN_MESSAGE_BRS | PCAN_MESSAGE_EXTENDED | PCAN_MESSAGE_FD;
  _power_board_tx_msg.DLC     = 8;    // 8 = 8 bytes; 9 = 12 bytes
  _power_board_tx_msg.DATA[0] = 0x05; // write 1 int8 register
  _power_board_tx_msg.DATA[1] = 0x03; // register 3 = Lock Time
  _power_board_tx_msg.DATA[2] = 0x4E; // timing 20s
  _power_board_tx_msg.DATA[3] = 0x20;
  _power_board_tx_msg.DATA[4] = 0x17; // READ_REGISTERS - INT16, 3 registers
  _power_board_tx_msg.DATA[5] = 0x00; // Starting register: 0x000 STATE, FAULT CODE, SWITCH STATUS
  _power_board_tx_msg.DATA[6] = 0x17; // READ_REGISTERS - INT16, 3 registers
  _power_board_tx_msg.DATA[7] = 0x10; // Starting register: 0x010 Output Voltage, Output Current, Temperature

  //--------------------------------------------------------------------------------------------
}

YloTwoPcanToMoteus::~YloTwoPcanToMoteus()
{
}

bool YloTwoPcanToMoteus::security_switch(){ // read Gpio port state.
        if (mraa_gpio_read(YloTwoPcanToMoteus::btnPin) == -1){
            ROS_INFO("ERROR IN MRAA LIB WITH GPIO !!! \n---> See YloTwoPcanToMoteus.cpp into security_switch function.");
            return true; // GPIO board not ready or button pressed
        }
        if (mraa_gpio_read(YloTwoPcanToMoteus::btnPin) == 0){
            ROS_INFO("SECUTITY SWITCH PRESSED !!! \n---> Motors are stopped now !!!.");
            YloTwoPcanToMoteus::_comm_maxtorque = 0.0;  // cutting any motor power, Need to relaunch to reset - TODO reinitialize it in rosparam ?!
            return false;
        }
        if (mraa_gpio_read(YloTwoPcanToMoteus::btnPin) == 1){
            //ROS_INFO("Working OK.");
            return false;
        }
}


bool YloTwoPcanToMoteus::init_and_reset(){
  for (unsigned int p = 0; p < 4; ++p){
    // open ports
    Status = CAN_InitializeFD(pcanPorts_[p], BitrateFD);
    CAN_GetErrorText(Status, 0, strMsg); // check the Status return state. If correct, it should return 0.
	  if (Status){std::cout << "Error: can't initialize " << pcanPorts_[p] << " port. Status = " << strMsg << std::endl;
      return(false);}
    usleep(200);

    //reset ports
    Status = CAN_Reset(pcanPorts_[p]);
    CAN_GetErrorText(Status, 0, strMsg);
    if(Status){std::cout << "Error: can't reset_buffer. " << pcanPorts_[p] << " port. Status = " << strMsg << std::endl;
      return(false);}
  }
  return(true);
}

bool YloTwoPcanToMoteus::send_moteus_stop_order(int id, int port){
    _stop.ID = 0x8000 | id;
    Status = CAN_WriteFD(port, &_stop);
    usleep(100);
    CAN_GetErrorText(Status, 0, strMsg);
    if(Status != PCAN_ERROR_OK){std::cout << "Error: can't stop motor " << id << " Status = " << strMsg << std::endl;
      return(false);}
    return(true);
}

bool YloTwoPcanToMoteus::send_moteus_TX_frame(int id, int port, float pos, float vel, float fftorque, float kp, float kd){
    _comm_position = pos;
    _comm_velocity = vel ;
    _comm_fftorque = fftorque; // in radians
    _comm_kp       = kp;
    _comm_kd       = kd;
    moteus_tx_msg.ID = 0x8000 | id;
    memcpy(&moteus_tx_msg.DATA[06], &_comm_position, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[14], &_comm_velocity, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[14], &_comm_fftorque, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[18], &_comm_kp, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[22], &_comm_kd, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[26], &_comm_maxtorque, sizeof(float));
    //std::cout << "torque = " << _comm_fftorque << std::endl;
	//std::copy(std::begin(moteus_tx_msg.DATA), std::end(moteus_tx_msg.DATA), std::ostream_iterator<int>(std::cout, " "));
	//std::cout << "" << std::endl;
    Status = CAN_WriteFD(port, &moteus_tx_msg);
    usleep(200);
    CAN_GetErrorText(Status, 0, strMsg);
    if(Status == PCAN_ERROR_OK){return(true);}
    std::cout << "error into send_moteus_TX_frame() YloTwoPcanToMoteus function : id=" << id << ". Status = " << strMsg << std::endl;
    return(false);
}

bool YloTwoPcanToMoteus::read_moteus_RX_queue(int id, int port, float& position, float& velocity, float& torque, float& voltage, float& temperature, float& fault){
    moteus_rx_msg.ID = 0x8000 | id;
    Status = CAN_ReadFD(port,&moteus_rx_msg, NULL); // read can port
    usleep(200);
    //std::cout<<("read_moteus_RX_queue : ");
	  //std::copy(std::begin(moteus_rx_msg.DATA), std::end(moteus_rx_msg.DATA), std::ostream_iterator<int>(std::cout, " "));
	  //std::cout << "" << std::endl;
    CAN_GetErrorText(Status, 0, strMsg);
    if(Status != PCAN_ERROR_QRCVEMPTY){ // rx queue feeded.
        memcpy(&_position, &moteus_rx_msg.DATA[MSGRX_ADDR_POSITION], sizeof(float));
        memcpy(&_velocity, &moteus_rx_msg.DATA[MSGRX_ADDR_VELOCITY], sizeof(float));
        memcpy(&_torque,   &moteus_rx_msg.DATA[MSGRX_ADDR_TORQUE],   sizeof(float));
        memcpy(&_voltage, &moteus_rx_msg.DATA[MSGRX_ADDR_VOLTAGE], sizeof(float));
        memcpy(&_temperature, &moteus_rx_msg.DATA[MSGRX_ADDR_TEMPERATURE], sizeof(float));
        memcpy(&_fault,   &moteus_rx_msg.DATA[MSGRX_ADDR_FAULT],   sizeof(float));
        position    = _position;   
        velocity    = _velocity;
        torque      = _torque;
        voltage     = _voltage;
        temperature = _temperature;
        fault       = _fault;
        return true;}
    else
        return false;
}

/*  ZERO - Set Output Nearest
    When sent, this causes the servo to select a whole number of internal motor rotations 
    so that the final position is as close to the given position as possible*/
bool YloTwoPcanToMoteus::send_moteus_zero_order(int id, int port, float zero_position){
    _zero.ID = 0x8000 | id;
    _comm_position = zero_position;
    memcpy(&_zero.DATA[3], &_comm_position, sizeof(float));
    Status = CAN_WriteFD(port,&_zero);
    usleep(200);
    CAN_GetErrorText(Status, 0, strMsg);
    if(Status == PCAN_ERROR_OK){
        return true;}
    else{
        ROS_INFO("--ERROR IN WRITING : send_moteus_zero_order()--");
        return false;}   
}

/* POWER BOARD */
/* WRITE */
bool YloTwoPcanToMoteus::send_power_board_order(){
    int ids  = 32;
    int port  = PCAN_DEV3;
    _power_board_tx_msg.ID = 0x8000 | ids;
    //std::copy(std::begin(_power_board_tx_msg.DATA), std::end(_power_board_tx_msg.DATA), std::ostream_iterator<int>(std::cout, " "));
	//std::cout << "" << std::endl;
    Status = CAN_WriteFD(port, &_power_board_tx_msg);
    usleep(200);
    CAN_GetErrorText(Status, 0, strMsg);
    if(Status == PCAN_ERROR_OK){
        return(true);}
    else{
        std::cout << "error into send_power_board_order(). Status = " << strMsg << std::endl;
        return false;}
}


/* READ   TODO */
bool YloTwoPcanToMoteus::read_power_board_RX_queue(float& state, float& fault_code, float& switch_status, float& out_volt, float& out_curr, float& board_temp){
    int ids  = 32;
    int port  = PCAN_DEV3;
    _power_board_rx_msg.ID = 0x8000 | ids;
    Status = CAN_ReadFD(port,&_power_board_rx_msg, NULL); // read can port
    usleep(200);
    //std::copy(std::begin(_power_board_rx_msg.DATA), std::end(_power_board_rx_msg.DATA), std::ostream_iterator<int>(std::cout, " "));
	//std::cout << "" << std::endl;
    CAN_GetErrorText(Status, 0, strMsg);
    std::cout << "status " << Status << std::endl;
    if(Status != PCAN_ERROR_QRCVEMPTY){ // rx queue feeded.

        //memcpy(&_state, &_power_board_rx_msg.DATA[MSGPBRX_ADDR_STATE], sizeof(float));
        //memcpy(&_fault_code, &_power_board_rx_msg.DATA[MSGPBRX_ADDR_FAULT_CODE], sizeof(float));
        //memcpy(&_switch_status,   &_power_board_rx_msg.DATA[MSGPBRX_ADDR_SWITCH_STATUS],   sizeof(float));
        memcpy(&_out_volt,   &_power_board_rx_msg.DATA[MSGPBRX_ADDR_OUT_VOLTAGE],   sizeof(float));
        memcpy(&_out_curr,   &_power_board_rx_msg.DATA[MSGPBRX_ADDR_OUT_CURRENT],   sizeof(float));
        memcpy(&_board_temp,   &_power_board_rx_msg.DATA[MSGPBRX_ADDR_TEMPERATURE],   sizeof(float));
        state = _state;   
        fault_code = _fault_code;
        switch_status = _switch_status;
        out_volt = _out_volt;
        out_curr = _out_curr;
        board_temp = _board_temp;

        return true;
    }    
    else {
        std::cout << "error into read_power_board_RX_queue(). Status = " << strMsg << std::endl;
        return false; }
}

bool YloTwoPcanToMoteus::peak_fdcan_board_initialization(){
    if(!YloTwoPcanToMoteus::init_and_reset()){ // run and check the return of the function
        all_moteus_controllers_ok = false;
        ROS_INFO("--PEAK BOARD ERROR - can't send Initialization frame to can port--");
        return false;}
    else {
        ROS_INFO("--MOTEUS INITIALIZATION & RESET-> OK--");
        usleep(200);
        stop_motors();
        all_moteus_controllers_ok = true;
        usleep(200);
        return true;}
}

bool YloTwoPcanToMoteus::stop_motors(){
    for(unsigned int i=0; i<12; ++i){
        auto ids = YloTwoPcanToMoteus::motor_adapters_[i].getIdx();
        int port  = YloTwoPcanToMoteus::motor_adapters_[i].getPort();
        // send a stop order, to avoid damages, and query its values.
        if(!YloTwoPcanToMoteus::send_moteus_stop_order(ids, port)){
            all_moteus_controllers_ok = false;
            ROS_INFO("-- PEAK BOARD ERROR - can't send Stop_command to id %d --", ids);
            return false;}
    }
    all_moteus_controllers_ok = true;
    ROS_INFO("--MOTEUS MOTORS STOPPED --------> OK--");
    usleep(200);
    return true;

}

bool YloTwoPcanToMoteus::check_initial_ground_pose(){
    // startup.
    int count = 0; // check zero for all 12 motors
    std::cout << ("\n-------------------------------------------------------------") << std::endl;
    std::cout << ("--  Zeroing joints. angle_joint tolerance is < 9 degrees  --") << std::endl;
    std::cout << ("---------------------------------------------   Waiting :  --\n") << std::endl;

    while(count != 12){
        // --- LOOPING WITH THE 12 MOTORS UNTIL SUCCESS---
        count = 0;
        for(unsigned int i=0; i<12; ++i){
            auto ids = YloTwoPcanToMoteus::motor_adapters_[i].getIdx();
            int port  = YloTwoPcanToMoteus::motor_adapters_[i].getPort();
            auto target_joint_position = initial_ground_joints_pose[i];
                
            // --- SENDING ZERO COMMAND ---
            if(!YloTwoPcanToMoteus::send_moteus_zero_order(ids, port, target_joint_position)){
                ROS_INFO("--- Error in send_moteus_zero_order() process. ---");
                can_error = true;
                return false;}
            usleep(200);

            // --- QUERYING VALUES ---
            if(!YloTwoPcanToMoteus::read_moteus_RX_queue(ids, port, RX_pos, RX_vel, RX_tor, RX_volt, RX_temp, RX_fault)){
                ROS_INFO("--- Error in read_moteus_RX_queue() process, on id %d. ---", ids);
                can_error = true;
                return false;}
            usleep(200);

            // --- CHECKING JOINT STARTUP ANGLE ---
            ROS_INFO("--Controleur ID : %d ; actual position : %f ; zero target position : %f ; diff : %f", 
                        ids, RX_pos, target_joint_position, (std::abs(std::abs(RX_pos) - std::abs(target_joint_position))));
            if(std::abs(std::abs(RX_pos) - std::abs(target_joint_position)) > std::abs(calibration_error)){
                is_calibrated = false;
                // ROS_INFO("-- Bad initial pose. Check motor %d.",ids);
                count -=1;
            }
            count +=1;
        }
        usleep(200);
    }

    std::cout << ("") << std::endl;
    ROS_INFO("--ROBOT CALIBRATION CHECKED ----> OK--");
    std::cout << ("") << std::endl;
    return true;
}