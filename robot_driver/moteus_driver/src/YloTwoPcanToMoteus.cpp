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
  motor_adapters_.resize(12);  // exact motors order, on Ylo2

  //                   IDX                             SIGN                         PCAN BOARD PORTS

  /*HAA*/ motor_adapters_[0].setIdx(3);  motor_adapters_[0].setSign(-1); motor_adapters_[0].setLeg_index(0); 
                                        motor_adapters_[0].setJoint_index(0); motor_adapters_[0].setPort(PCAN_DEV1);

  /*HFE*/ motor_adapters_[1].setIdx(1);  motor_adapters_[1].setSign(1); motor_adapters_[1].setLeg_index(0); 
                                        motor_adapters_[1].setJoint_index(1); motor_adapters_[1].setPort(PCAN_DEV1);

  /*KFE*/ motor_adapters_[2].setIdx(2);  motor_adapters_[2].setSign(1); motor_adapters_[2].setLeg_index(0); 
                                        motor_adapters_[2].setJoint_index(2); motor_adapters_[2].setPort(PCAN_DEV1);

  // LH
  /*HAA*/ motor_adapters_[3].setIdx(9);  motor_adapters_[3].setSign(1);  motor_adapters_[3].setLeg_index(1); 
                                        motor_adapters_[3].setJoint_index(0); motor_adapters_[3].setPort(PCAN_DEV3);

  /*HFE*/ motor_adapters_[4].setIdx(7);  motor_adapters_[4].setSign(1); motor_adapters_[4].setLeg_index(1); 
                                        motor_adapters_[4].setJoint_index(1); motor_adapters_[4].setPort(PCAN_DEV3);

  /*KFE*/ motor_adapters_[5].setIdx(8);  motor_adapters_[5].setSign(1); motor_adapters_[5].setLeg_index(1); 
                                        motor_adapters_[5].setJoint_index(2); motor_adapters_[5].setPort(PCAN_DEV3);

  // RF
  /*HAA*/ motor_adapters_[6].setIdx(6);  motor_adapters_[6].setSign(1); motor_adapters_[6].setLeg_index(2); 
                                        motor_adapters_[6].setJoint_index(0); motor_adapters_[6].setPort(PCAN_DEV2);

  /*HFE*/ motor_adapters_[7].setIdx(4);  motor_adapters_[7].setSign(-1);  motor_adapters_[7].setLeg_index(2); 
                                        motor_adapters_[7].setJoint_index(1); motor_adapters_[7].setPort(PCAN_DEV2);

  /*KFE*/ motor_adapters_[8].setIdx(5);  motor_adapters_[8].setSign(-1);  motor_adapters_[8].setLeg_index(2); 
                                        motor_adapters_[8].setJoint_index(2); motor_adapters_[8].setPort(PCAN_DEV2);

  // RH
  /*HAA*/ motor_adapters_[9].setIdx(12);  motor_adapters_[9].setSign(-1);  motor_adapters_[9].setLeg_index(3); 
                                        motor_adapters_[9].setJoint_index(0); motor_adapters_[9].setPort(PCAN_DEV4);

  /*HFE*/ motor_adapters_[10].setIdx(10); motor_adapters_[10].setSign(-1); motor_adapters_[10].setLeg_index(3); 
                                        motor_adapters_[10].setJoint_index(1); motor_adapters_[10].setPort(PCAN_DEV4);

  /*KFE*/ motor_adapters_[11].setIdx(11); motor_adapters_[11].setSign(-1); motor_adapters_[11].setLeg_index(3); 
                                        motor_adapters_[11].setJoint_index(2); motor_adapters_[11].setPort(PCAN_DEV4);

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
  moteus_tx_msg.DLC      = 14; // 12 = 24 bytes, 13 = 32 bytes, 14 = 48 bytes, 15 = 64 bytes
  moteus_tx_msg.DATA[0]  =  0x01; // WRITE_REGISTERS - Type.INT8 1 registers
  moteus_tx_msg.DATA[1]  =  0x00; // Starting at reg 0x000(MODE)
  moteus_tx_msg.DATA[2]  =  0x0A; // Reg 0x000(MODE) = 10(POSITION)

  moteus_tx_msg.DATA[3]  =  0x0C; // WRITE_REGISTERS - Type FLOAT
  moteus_tx_msg.DATA[4]  =  0x0A; // 5 registers
  moteus_tx_msg.DATA[5]  =  0x20; // Starting at reg 0x020 POSITION
  moteus_tx_msg.DATA[6]  = _comm_position; // FLOAT
  moteus_tx_msg.DATA[10] = _comm_velocity;
  moteus_tx_msg.DATA[14] = _comm_fftorque;
  moteus_tx_msg.DATA[18] = _comm_kp;
  moteus_tx_msg.DATA[22] = _comm_kd; 
  moteus_tx_msg.DATA[26] = _comm_maxtorque;
  moteus_tx_msg.DATA[30] = _comm_stop_mode;
  moteus_tx_msg.DATA[34] = _comm_watchdog_timeout;
  moteus_tx_msg.DATA[38] = _comm_target_vel;
  moteus_tx_msg.DATA[42] = _comm_target_acc;

  moteus_tx_msg.DATA[46] = 0x1F; // READ_REGISTERS - FLOAT
  moteus_tx_msg.DATA[47] = 0x01; // Starting at reg 0x001(POSITION)
  moteus_tx_msg.DATA[48] = 0x1F; // READ_REGISTERS - FLOAT
  moteus_tx_msg.DATA[49] = 0x0D; // Starting at reg 0x00d(VOLTAGE)
  moteus_tx_msg.DATA[50] = 0x50; // Padding a NOP byte (moteus protocol)
  moteus_tx_msg.DATA[51] = 0x50;
  moteus_tx_msg.DATA[52] = 0x50;
  moteus_tx_msg.DATA[53] = 0x50;
  moteus_tx_msg.DATA[54] = 0x50;
  moteus_tx_msg.DATA[55] = 0x50;
  moteus_tx_msg.DATA[56] = 0x50;
  moteus_tx_msg.DATA[57] = 0x50;
  moteus_tx_msg.DATA[58] = 0x50;
  moteus_tx_msg.DATA[59] = 0x50;
  moteus_tx_msg.DATA[60] = 0x50;
  moteus_tx_msg.DATA[61] = 0x50;
  moteus_tx_msg.DATA[62] = 0x50;
  moteus_tx_msg.DATA[63] = 0x50;

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

sensor_msgs::Imu YloTwoPcanToMoteus::ylo3_imu;

bool YloTwoPcanToMoteus::security_switch(){ // read Gpio port state.
        if (mraa_gpio_read(YloTwoPcanToMoteus::btnPin) == -1){
            ROS_INFO("ERROR IN MRAA LIB WITH GPIO !!! \n---> See YloTwoPcanToMoteus.cpp into security_switch function.");
            return true; // GPIO board not ready or button pressed
        }
        if (mraa_gpio_read(YloTwoPcanToMoteus::btnPin) == 1){
            //ROS_INFO("Working OK.");
            return true;
        }
        _comm_maxtorque = 0.0;
        ROS_INFO("SECUTITY SWITCH PRESSED !!! \n---> Motors are stopped now !!!.");
        return false;
}


bool YloTwoPcanToMoteus::Can_reset(){
    for (unsigned int p = 0; p < 4; ++p){
        //reset ports
        Status = CAN_Reset(pcanPorts_[p]);
        CAN_GetErrorText(Status, 0, strMsg);
        if(Status){std::cout << "Error: can't reset_buffer. " << pcanPorts_[p] << " port. Status = " << strMsg << std::endl;
            return(false);}
    }
    return(true);
}


bool YloTwoPcanToMoteus::Can_init(){
  for (unsigned int p = 0; p < 4; ++p){
    // open ports
    do{ Status = CAN_InitializeFD(pcanPorts_[p], BitrateFD); usleep(10); }
    while(Status != PCAN_ERROR_OK);
  }
  return(true);
}


void YloTwoPcanToMoteus::subscribeToImuData() {
    ros::NodeHandle nh;
    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 1, &YloTwoPcanToMoteus::imuCallback, this);
    ros::Rate rate(100); // 100 hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}


void YloTwoPcanToMoteus::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    ylo3_imu = *imu_msg;
}


sensor_msgs::Imu YloTwoPcanToMoteus::getYlo3Imu() {
        return ylo3_imu;
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

// send a position order, using stop replacement/update mode, with vel and acc targets.
bool YloTwoPcanToMoteus::send_to_moteus(int id, int port, float pos, float vel, float fftorque, float kp, float kd, float target_vel, float target_accel){
    _comm_position          = pos;
    _comm_velocity          = NAN;
    _comm_fftorque          = fftorque; // in radians
    _comm_kp                = kp;
    _comm_kd                = kd;
    _comm_stop_mode         = NAN;
    _comm_watchdog_timeout  = 10;       // new mode : https://jpieper.com/2023/08/22/new-hold-position-watchdog-timeout-mode-for-moteus/
    _comm_target_vel        = target_vel;
    _comm_target_acc        = target_accel;

    moteus_tx_msg.ID        = 0x8000 | id;   
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_POSITION],    &_comm_position, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_VELOCITY],    &_comm_velocity, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_FFTORQUE],    &_comm_fftorque, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_KP],          &_comm_kp, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_KD],          &_comm_kd, sizeof(float));
    
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_MAXTORQUE],   &_comm_maxtorque, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_STOP_MODE],   &_comm_stop_mode, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_WATCHDOG_],   &_comm_watchdog_timeout, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_TARG_VEL],    &_comm_target_vel, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_TARG_ACC],    &_comm_target_acc, sizeof(float));
    
    //std::cout << "write" << std::endl;
	//std::copy(std::begin(moteus_tx_msg.DATA), std::end(moteus_tx_msg.DATA), std::ostream_iterator<float>(std::cout, " "));
	//std::cout << "" << std::endl;

    do{ Status = CAN_WriteFD(port, &moteus_tx_msg);}
    while(Status != PCAN_ERROR_OK);
    return(true);
}

// send a position order, using stop replacement/update mode, with vel and acc targets.
bool YloTwoPcanToMoteus::send_to_moteus(int id, int port, float pos, float vel, float fftorque, float kp, float kd){
    _comm_position          = pos;
    _comm_velocity          = vel;
    _comm_fftorque          = fftorque; // in radians
    _comm_kp                = kp;
    _comm_kd                = kd;
    _comm_stop_mode         = NAN;
    _comm_watchdog_timeout  = 10;
    _comm_target_vel        = NAN;
    _comm_target_acc        = NAN;
    moteus_tx_msg.ID        = 0x8000 | id;  
     
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_POSITION],    &_comm_position, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_VELOCITY],    &_comm_velocity, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_FFTORQUE],    &_comm_fftorque, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_KP],          &_comm_kp, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_KD],          &_comm_kd, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_MAXTORQUE],   &_comm_maxtorque, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_STOP_MODE],   &_comm_stop_mode, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_WATCHDOG_],   &_comm_watchdog_timeout, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_TARG_VEL],    &_comm_target_vel, sizeof(float));
    memcpy(&moteus_tx_msg.DATA[MSGTX_ADDR_TARG_ACC],    &_comm_target_acc, sizeof(float));

    do{ Status = CAN_WriteFD(port, &moteus_tx_msg);}
    while(Status != PCAN_ERROR_OK);
    return(true);
}

bool YloTwoPcanToMoteus::read_moteus_RX_queue(int id, int port, float& position, float& velocity, float& torque, float& voltage, float& temperature, float& fault){
    moteus_rx_msg.ID = 0x8000 | id;

    do{ Status = CAN_ReadFD(port,&moteus_rx_msg, NULL); 
        usleep(10); }
    while(Status != PCAN_ERROR_OK); // is return frame received ?

    if(Status != PCAN_ERROR_QRCVEMPTY){ // rx queue feeded.
        memcpy(&_position, &moteus_rx_msg.DATA[MSGRX_ADDR_POSITION], sizeof(float));
        memcpy(&_velocity, &moteus_rx_msg.DATA[MSGRX_ADDR_VELOCITY], sizeof(float));
        memcpy(&_torque,   &moteus_rx_msg.DATA[MSGRX_ADDR_TORQUE],   sizeof(float));
        memcpy(&_voltage, &moteus_rx_msg.DATA[MSGRX_ADDR_VOLTAGE], sizeof(int8_t));
        memcpy(&_temperature, &moteus_rx_msg.DATA[MSGRX_ADDR_TEMPERATURE], sizeof(int8_t));
        memcpy(&_fault,   &moteus_rx_msg.DATA[MSGRX_ADDR_FAULT],   sizeof(int8_t));
        position    = _position;   
        velocity    = _velocity;
        torque      = _torque;
        voltage     = _voltage;
        temperature = _temperature;
        fault       = _fault;
        
        //std::cout << "read" << std::endl;
        //std::copy(std::begin(moteus_tx_msg.DATA), std::end(moteus_tx_msg.DATA), std::ostream_iterator<float>(std::cout, " "));
	    //std::cout << "" << std::endl;

        return true;}
    else{
        std::cout << "### RX queue is empty for ID " << id << "." << std::endl;
        return false;
    }
}

bool YloTwoPcanToMoteus::read_moteus_RX_queue(int id, int port, float& voltage, float& temperature, float& fault){
    moteus_rx_msg.ID = 0x8000 | id;

    do{ Status = CAN_ReadFD(port,&moteus_rx_msg, NULL); 
        usleep(10); }
    while(Status != PCAN_ERROR_OK); // is return frame received ?

    if(Status != PCAN_ERROR_QRCVEMPTY){ // rx queue feeded.
        memcpy(&_voltage, &moteus_rx_msg.DATA[MSGRX_ADDR_VOLTAGE], sizeof(int8_t));
        memcpy(&_temperature, &moteus_rx_msg.DATA[MSGRX_ADDR_TEMPERATURE], sizeof(int8_t));
        memcpy(&_fault,   &moteus_rx_msg.DATA[MSGRX_ADDR_FAULT],   sizeof(int8_t));
        voltage     = _voltage;
        temperature = _temperature;
        fault       = _fault;

        //std::cout << "----------> RX frame: " << id << std::endl;
	    //std::copy(std::begin(moteus_rx_msg.DATA), std::end(moteus_rx_msg.DATA), std::ostream_iterator<int>(std::cout, " "));
	    //std::cout << "" << std::endl;
        // a décrypter avec : /home/ylo2/Documents/decode_Moteus_can_frame.py

        return true;}
    else{
        std::cout << "### RX queue is empty for ID " << id << "." << std::endl;
        return false;
    }
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
    usleep(50);
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
    if(!YloTwoPcanToMoteus::Can_init()){ // run and check the return of the function
        all_moteus_controllers_ok = false;
        ROS_INFO("--PEAK BOARD ERROR - can't send Initialization frame to can port--");
        return false;}

    usleep(200);

    if(!YloTwoPcanToMoteus::Can_reset()){
        all_moteus_controllers_ok = false;
        ROS_INFO("--PEAK BOARD ERROR - can't send reset frame to can port--");
        return false;}

    usleep(200);

    ROS_INFO("--PEAK INITIALIZATION AND RESET---> OK--");
    stop_motors();
    usleep(200);

    all_moteus_controllers_ok = true;
    return true;
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
    ROS_INFO("--CONTROLLERS STOPPED & RESET ----> OK--");
    usleep(200);
    return true;

}

bool YloTwoPcanToMoteus::check_initial_ground_pose(){
    // startup.
    int count = 0; // check zero for all 12 motors
    while(count != 12){
        // --- LOOPING WITH THE 12 MOTORS UNTIL SUCCESS---
        count = 0;
        for(unsigned int i=0; i<12; ++i){
            auto ids = YloTwoPcanToMoteus::motor_adapters_[i].getIdx();
            int port  = YloTwoPcanToMoteus::motor_adapters_[i].getPort();
            auto target_joint_position = sit_down_joints_pose[i];
                
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
            float diff = std::abs(std::abs(RX_pos) - std::abs(target_joint_position));
            if(diff > std::abs(calibration_error)){
                ROS_INFO("--Ylo2 joint ID : %d ; pos : %f ; target : %f ; diff : %f", ids, RX_pos, target_joint_position, diff);
                is_calibrated = false;
                count -=1;
            }
            usleep(200);
            count +=1;
        }
        usleep(200);
    }

    ROS_INFO("--ROBOT ZERO CALIBRATION ---------> OK--");
    std::cout << ("") << std::endl;
    return true;
}
