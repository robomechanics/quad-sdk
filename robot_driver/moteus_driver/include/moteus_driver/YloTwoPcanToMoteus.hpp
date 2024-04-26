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
#include <sensor_msgs/Imu.h>

#include <PCANBasic.h> // Peak m2canFd board lib

#include "mraa/common.hpp" // for GPIO security switch
#include "mraa/gpio.hpp"

#include <cmath> // pour la fonction champ_standup

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
#define MSGTX_ADDR_STOP_MODE    0x1E 
#define MSGTX_ADDR_WATCHDOG_    0x22 
#define MSGTX_ADDR_TARG_VEL     0x26 
#define MSGTX_ADDR_TARG_ACC     0x2A

/* moteus controllers RX bytes adress */
#define MSGRX_ADDR_POSITION     0x02
#define MSGRX_ADDR_VELOCITY     0x06
#define MSGRX_ADDR_TORQUE       0x0A
#define MSGRX_ADDR_VOLTAGE      0x0E
#define MSGRX_ADDR_TEMPERATURE  0x12
#define MSGRX_ADDR_FAULT        0x16

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
      wires diagram : 
        - black is ground (pin 1); 
        - red is +3.3vcc with 10k resistor (pin 6); 
        - white is gpio read (pin 29)*/
    bool security_switch();

    /* PEAK BOARD M2 4 CANFD PORTS
       initialize all 4 ports*/
    bool Can_init();

    /* reset all 4 ports*/
    bool Can_reset();

    /* Imu functions */
    static sensor_msgs::Imu ylo3_imu; // the imu variable
    void subscribeToImuData();
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    sensor_msgs::Imu getYlo3Imu();

    /* zero a single moteus controller*/
    bool send_moteus_zero_order(int id, int port, float position);

    /* send a single canFD STOP frame command to clear any faults, 
       and query informations about moteus controller*/
    bool send_moteus_stop_order(int id, int port);

    /* sending a velocity command */
    bool send_to_moteus(int id, int port, float pos, float vel, float fftorque, float kp, float kd, float target_vel, float target_accel);
    bool send_to_moteus(int id, int port, float pos, float vel, float fftorque, float kp, float kd);

    /* query a single canFD RX Queue, 
       and read ID params*/
    bool read_moteus_RX_queue(int id, int port, float& position, float& velocity, float& torque, float& voltage, float& temperature, float& fault);
    bool read_moteus_RX_queue(int id, int port, float& voltage, float& temperature, float& fault);

    /* send a single canFD command frame
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
      3/ loop until success */
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

    //for moteus TX
    float _comm_position          = 0.0; // NAN for torque mode
    float _comm_velocity          = 0.0;
    float _comm_fftorque          = 1.0; // variable Tau
    float _comm_kp                = 10.0;
    float _comm_kd                = 1.0;
    float _comm_maxtorque         = 0.10; // Max possible torque is NAN value
    float _comm_stop_mode         = NAN;
    float _comm_watchdog_timeout  = 10;
    float _comm_target_vel        = 0.6;
    float _comm_target_acc        = 0.3;


    // for mraa library GPIO (security switch)
    mraa_gpio_context btnPin; //  Will be used to represnt the button pin

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


    //  robot position is sitted
    std::vector<float>sit_down_joints_pose = { -0.0461273193359375, -0.1825408935546875,  0.371002197265625,  // 3, 1, 2
                                                0.0385894775390625, -0.1865081787109375,  0.371063232421875,  // 9, 7, 8
                                                0.04315185546875,    0.18280029296875,   -0.373260498046875,  // 6, 4, 5
                                               -0.04425048828125,    0.188568115234375,  -0.372222900390625}; // 12, 10, 11


    //  stand up target pose (en tours/s) - pattes un peu trop écartées, dues a leur position au sol !
    std::vector<float>standup_joints_pose = {        0.000582622, 0.1629, -0.28942,
                                                     0.000582622, -0.1629, 0.28942,
                                                    -0.000582622, 0.1629, -0.28942,
                                                    -0.000582622, -0.1629, 0.28942};

    float calibration_error = 0.03; // 10.8 degrees  ca passe aussi avec 0.01 (3.6 degrees)
};

#endif // PCANTOMOTEUS_HPP