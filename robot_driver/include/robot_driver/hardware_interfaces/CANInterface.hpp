#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace Recieve_CAN_Frame {
    enum {
        FAILED,
        SUCCESS,
        TIMEOUT
    };
}

namespace CAN_interface
{
    class CANInterface{

    public:
        CANInterface(const char* socketName);

        ~CANInterface();
        
        bool sendCANFrame(int can_id, const unsigned char* CANMsg);
        int receiveCANFrame(unsigned char* CANMsg, std::chrono::milliseconds timeout_time);

    // private:    
        int socket_descrp_; // File descriptor for the socket as everything in Linux/Unix is a file.
    };

}
#endif // CAN_INTERFACE_HPP