#ifndef PCANTOMOTEUS_HPP
#define PCANTOMOTEUS_HPP

#include <iostream>
#include <ostream> 
#include <iterator>
#include <ctime>
#include <vector>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <ros/ros.h>
#include <PCANBasic.h> // Peak m2canFd board lib
#include "mraa/common.hpp"
#include "mraa/gpio.hpp" // for GPIO security switch

// define GPIO switch port
#define BTN_PIN      29

//define pcan 4 ports to their respective physical adress
#define PCAN_DEV1	PCAN_PCIBUS1
#define PCAN_DEV2	PCAN_PCIBUS2
#define PCAN_DEV3	PCAN_PCIBUS3
#define PCAN_DEV4	PCAN_PCIBUS4

/* moteus controllers TX bytes adress */
#define MSGTX_ADDR_POSITION     0x06 
#define MSGTX_ADDR_VELOCITY     0x0A
#define MSGTX_ADDR_FFTORQUE     0x0E
#define MSGTX_ADDR_KP           0x12
#define MSGTX_ADDR_KD           0x16
#define MSGTX_ADDR_MAXTORQUE    0x1A

/* moteus controllers RX bytes adress */
#define MSGRX_ADDR_POSITION     0x02
#define MSGRX_ADDR_VELOCITY     0x06
#define MSGRX_ADDR_TORQUE       0x0A
#define MSGRX_ADDR_VOLTAGE      0x16
#define MSGRX_ADDR_TEMPERATURE  0x1A
#define MSGRX_ADDR_FAULT        0x1E


// about moteus controllers faults errors:
/*  32 - calibration fault - the encoder was not able to sense a magnet during calibration
    33 - motor driver fault - the most common reason for this is undervoltage, moteus attempted to draw more current than the supply could provide. Other electrical faults may also report this error, the drv8323 diagnostic tree has more information.
    34 - over voltage - the bus voltage exceeded servo.max_voltage. This can happen due to misconfiguration, or if the controller regenerated power with a supply that cannot sink power and no flux braking was configured.
    35 - encoder fault - the encoder readings are not consistent with a magnet being present.
    36 - motor not configured - the moteus_tool --calibrate procedure has not been run on this motor.
    37 - pwm cycle overrun - an internal firmware error
    38 - over temperature - the maximum configured temperature has been exceeded
    39 - outside limit - an attempt was made to start position control while outside the bounds configured by servopos.position_min and servopos.position_max.*/
/* power board RX bytes adress */ // TODO check adresses

#define MSGPBRX_ADDR_STATE          0x02 // 2 bytes per value (int16)   
#define MSGPBRX_ADDR_FAULT_CODE     0x04  
#define MSGPBRX_ADDR_SWITCH_STATUS  0x06
#define MSGPBRX_ADDR_OUT_VOLTAGE    0x08
#define MSGPBRX_ADDR_OUT_CURRENT    0x0A
#define MSGPBRX_ADDR_TEMPERATURE    0x0C
#define MSGPBRX_ADDR_ENERGY         0x0E

// about moteus power board faults errors:
/* TODO */

// a structure for ylo2 controllers setup
struct MotorAdapter{
  public:
    MotorAdapter(){
      idx_          = -1;
      sign_         =  1;
      port_         = -1;
      leg_index_    =  0;
      joint_index_  =  0;
    }

    MotorAdapter(int idx, int sign, int port, int leg_index, int joint_index){
      idx_          = idx;
      sign_         = sign;
      port_         = port;
      leg_index_    = leg_index;
      joint_index_  = joint_index;
    }

    const int& getIdx()                   {return idx_;}
    const int& getSign()                  {return sign_;}
    const int& getPort()                  {return port_;}
    const int& getLeg_index()             {return leg_index_;}
    const int& getJoint_index()           {return joint_index_;}

    void setIdx(int idx)                  {idx_         = idx;}
    void setSign(int sign)                {sign_        = sign;}
    void setPort(int port)                {port_        = port;}
    void setLeg_index(int leg_index)      {leg_index_   = leg_index;}
    void setJoint_index(int joint_index)  {joint_index_ = joint_index;}
    
  private:
    int idx_;
    int sign_;
    int port_;
    int leg_index_;
    int joint_index_;
};

// the YloTwoPcanToMoteus class
class YloTwoPcanToMoteus{
  public:

    /* SECURITY RED SWITCH
      GPIO usage under Up extreme i7, with mraa lib
      wires diagram : 
        - black is ground (pin 1); 
        - red is +3.3vcc with 10k resistor (pin 6); 
        - white is gpio read (pin 29)*/
    bool security_switch();

    /* PEAK BOARD M2 4 CANFD PORTS
       initialize and reset all 4 ports*/
    bool init_and_reset();

    /* zero each moteus controller*/
    bool send_moteus_zero_order(int id, int port, float position);

    /* send a canFD STOP frame command, 
       and query informations about moteus controller*/
    bool send_moteus_stop_order(int id, int port);

    /* send a canFD TORQUE frame
       and query informations about moteus controller*/
    bool send_moteus_TX_frame(int id, int port, float pos, float vel, float fftorque, float kp, float kd);

    /* query canFD RX Queue, 
       and read ID params*/
    bool read_moteus_RX_queue(int id, int port, float& position, float& velocity, float& torque, float& voltage, float& temperature, float& fault);

    /* send a canFD command frame
       ask informations about moteus power board*/
    bool send_power_board_order();

    /* query power board to read values*/
    bool read_power_board_RX_queue(float& state, float& fault_code, float& switch_status, float& out_volt, float& out_curr, float& board_temp);

    /*  initialize Peak canFD board,
            reset the 4 ports.*/
    bool peak_fdcan_board_initialization();

    /*  send a stop order to all motors  */
    bool stop_motors();

    /*1/ zero joints with initial_ground_joints_pose vector
      2/ check angle error between asked rezero, and read position
      3/ loop until success
      4/ stand up robot */
    bool check_initial_ground_pose();

    uint32_t _id; // ID of a moteus controller
    YloTwoPcanToMoteus();
    virtual ~YloTwoPcanToMoteus();

    std::vector<MotorAdapter> motor_adapters_;

    bool all_moteus_controllers_ok = true;
    bool can_error = false; // error in Peak FDCAN send order or query
    bool is_calibrated = true; // is the robot in right startup pose

    //for moteus RX
    float RX_mode  = 0;
    float RX_pos   = 0.0;
    float RX_vel   = 0.0;
    float RX_tor   = 0.0;
    float RX_volt  = 0.0;
    float RX_temp  = 0.0;
    float RX_fault = 0;

    float _comm_position      = 0.0; // NAN for torque mode
    float _comm_velocity      = 0.0;
    float _comm_fftorque      = 0.0; // variable Tau
    float _comm_kp            = 0.0;
    float _comm_kd            = 0.0;
    float _comm_maxtorque     = 1.0; // Max possible torque is NAN value

    // for mraa library GPIO (security switch)
    mraa_gpio_context btnPin; //  Will be used to represnt the button pin     //TODO create private class

    // for use with security button status
    bool security_button_flag = 1;

  private:
    
    // for pcanbasic library
    TPCANStatus Status; // the return of a command, to check success

    // Define the compatible Moteus FD Bitrate string
    TPCANBitrateFD BitrateFD = (char*) "f_clock_mhz = 80, nom_brp = 1, nom_tseg1 = 50, nom_tseg2 = 29, nom_sjw = 10, data_brp = 1, data_tseg1 = 8, data_tseg2 = 7, data_sjw = 12";
    TPCANTimestampFD timestamp;

    TPCANMsgFD _stop; // the stop canFD message 
    TPCANMsgFD _zero; // the stop canFD message 
    TPCANMsgFD moteus_tx_msg; // the canFD message to send order to a moteus controller
    TPCANMsgFD moteus_rx_msg; // the canFD message to read moteus controller
    TPCANMsgFD _power_board_tx_msg; // the canFD message to ask moteus power board
    TPCANMsgFD _power_board_rx_msg; // the canFD message to read moteus power board

    char strMsg[256];
    std::vector<int> pcanPorts_ = {PCAN_DEV1, PCAN_DEV2, PCAN_DEV3, PCAN_DEV4};

    int idx_;
    int sign_;
    int port_;
    int leg_index_;
    int joint_index_;
    int stop_pos_low_;
    int stop_pos_high_;

    /* query variables for moteus controllers */
    float _position     = 0.0;
    float _velocity     = 0.0;
    float _torque       = 0.0;
    float _voltage      = 0.0;
    float _temperature  = 0.0;
    float _fault        = 0.0;

    /* query variables for moteus power board */
    float _state          = 0.0;
    float _fault_code     = 0.0;
    float _switch_status  = 0.0;
    float _lock_time      = 0.0;
    float _boot_time      = 0.0;
    float _out_volt       = 0.0;
    float _out_curr       = 0.0;
    float _board_temp     = 0.0;
    float _energy         = 0.0;

      //  zero position of controllers, to check
    std::vector<float>initial_ground_joints_pose = {-0.058065, -0.1830422580242157, 0.3962658643722534,  //  3, 1, 2
                                                     0.055634, -0.18712398409843445, 0.4280807077884674,  //  9, 7, 8
                                                     0.058340, 0.1817300021648407, -0.4297744333744049,  //  6, 4, 5
                                                    -0.062968, 0.18036942183971405, -0.4263542890548706}; // 12, 10, 11

    float calibration_error = 0.02;
};

#endif // PCANTOMOTEUS_HPP